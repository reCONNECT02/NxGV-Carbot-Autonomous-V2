"""BLOCK 03 - Turn three views into road (V4 perception.js).

Front (Astra Pro) + left + right MIPI images are warped into one top-down
vehicle grid; each cell takes the strongest view (weight 1/(eps+depth^2)),
pixels are classified road / paint / other, and the connected road region is
grown (4-neighbour) from a seed near the car.

Lens distortion (pinhole or fisheye) and the top-down warp are ONE cv2.remap
per camera: the remap tables are built once from the intrinsics
(cameras.yaml sensors.<s>.intrinsics_file, calibration step 3) and the mount
(cameras.yaml mounts.<role>, calibration step 4). Before step 3 an ideal
pinhole from mounts.<role>.hfov_deg is used and the node reports WARN.

In : 3 camera images (topics from cameras.yaml roles)
Out: LocalGrid /carbot/perception/road_grid (base_link)
     debug JPEGs (only encoded while someone subscribes, throttled):
     warped (3 per-camera warps), stitched (colour BEV), mask (V4 colours),
     overlay/<role> (road mask drawn back onto that camera's footage).
"""
import time
from typing import Dict, Optional

import rclpy
from carbot_common import topics as T
from carbot_common.data import camera_role_topics, load_data
from carbot_common.node import CarbotNode
from carbot_common.qos import SENSOR
from carbot_interfaces.msg import LocalGrid, NodeStatus
from sensor_msgs.msg import CompressedImage, Image

from .camera_model import camera_setup, intrinsics_for
from .road_mask import (BodyBox, Classify, GridSpec, OverlayLookup, RoadMask, Seed,
                        bev_view, build_camera_map, mask_colours, warped_strip)
from .ros_image import image_to_bgr, jpeg_msg, resize_to_width, u8

REQUIRED = ['process_rate_hz', 'input_downscale_width', 'grid.cells', 'grid.resolution_m',
            'grid.origin_x_m', 'grid.origin_y_m', 'stitch.weight_eps', 'stitch.min_depth_m',
            'stitch.border_px', 'stitch.exclude_body',
            'classify.road_max_luma', 'classify.road_max_chroma', 'classify.paint_min_luma',
            'seed.x_min_m', 'seed.x_max_m', 'seed.half_width_m', 'camera_timeout_s',
            'debug.enabled', 'debug.jpeg_quality', 'debug.max_rate_hz', 'debug.overlay_width',
            'debug.bev_scale', 'data.cameras', 'vehicle.car_length_m', 'vehicle.car_width_m',
            'vehicle.rear_overhang_m', 'frames.base']


class RoadPerception(CarbotNode):

    def __init__(self):
        super().__init__('road_perception', '03', REQUIRED)
        self.cameras = load_data(self, 'cameras')
        self.topics = camera_role_topics(self.cameras)
        self.grid = GridSpec(int(self.p('grid.cells')), float(self.p('grid.resolution_m')),
                             float(self.p('grid.origin_x_m')), float(self.p('grid.origin_y_m')))
        self.mask = RoadMask(self.grid, self._seed())
        self.frame_id = str(self.p('frames.base'))
        self.latest: Dict[str, Optional[Image]] = {r: None for r in self.topics}
        self.rx_time: Dict[str, float] = {r: -1.0 for r in self.topics}
        self.rx_count: Dict[str, int] = {r: 0 for r in self.topics}
        self.map_error: Dict[str, str] = {}
        self.overlays: Dict[str, OverlayLookup] = {}
        self.last_debug = 0.0
        self.proc_ms = 0.0
        self._rate_t0 = time.monotonic()
        self._rates: Dict[str, float] = {r: 0.0 for r in self.topics}

        for role, topic in self.topics.items():
            self.sub(Image, topic, lambda m, r=role: self._on_image(r, m), SENSOR)
        self.pub_grid = self.create_publisher(LocalGrid, T.ROAD_GRID, 10)
        self.pub_dbg = {
            'warped': self.create_publisher(CompressedImage, T.PERCEPTION_DEBUG_WARPED, 1),
            'stitched': self.create_publisher(CompressedImage, T.PERCEPTION_DEBUG_STITCHED, 1),
            'mask': self.create_publisher(CompressedImage, T.PERCEPTION_DEBUG_MASK, 1)}
        self.pub_overlay = {r: self.create_publisher(CompressedImage, T.perception_overlay(r), 1)
                            for r in T.CAMERA_ROLES}
        self.create_timer(1.0 / max(float(self.p('process_rate_hz')), 0.5), self._process)
        self.set_status(NodeStatus.WARN, 'WAITING_INPUT', 'no camera frames yet')
        self.get_logger().info(f'[block 03] cameras: {self.topics}')

    # ---------------------------------------------------------------- params
    def _seed(self) -> Seed:
        return Seed(float(self.p('seed.x_min_m')), float(self.p('seed.x_max_m')),
                    float(self.p('seed.half_width_m')))

    def _classify(self) -> Classify:   # re-read every cycle: live tuning in calibrate mode
        return Classify(float(self.p('classify.road_max_luma')),
                        float(self.p('classify.road_max_chroma')),
                        float(self.p('classify.paint_min_luma')))

    def _body(self) -> Optional[BodyBox]:
        if not bool(self.p('stitch.exclude_body')):
            return None
        length, rear = float(self.p('vehicle.car_length_m')), float(self.p('vehicle.rear_overhang_m'))
        return BodyBox(rear, length - rear, float(self.p('vehicle.car_width_m')) / 2.0)

    # ---------------------------------------------------------------- input
    def _on_image(self, role: str, msg: Image) -> None:
        self.latest[role] = msg          # keep only the newest; decode in the timer
        self.rx_time[role] = time.monotonic()
        self.rx_count[role] += 1

    def _ensure_map(self, role: str, width: int, height: int) -> bool:
        m = self.mask.maps.get(role)
        if m is not None and (m.width, m.height) == (width, height):
            return True
        try:
            intr = intrinsics_for(self.cameras, role, width, height)
            _, _, mount, _ = camera_setup(self.cameras, role)
            self.mask.maps[role] = build_camera_map(
                role, self.grid, intr, mount, self._body(), float(self.p('stitch.min_depth_m')),
                float(self.p('stitch.weight_eps')), float(self.p('stitch.border_px')))
            self.overlays.pop(role, None)
            self.map_error.pop(role, None)
            cells = int(self.mask.maps[role].valid.sum())
            self.get_logger().info(
                f'[block 03] {role}: map built at {width}x{height}, {intr.model} '
                f'({intr.source}, hfov {intr.hfov_deg():.1f} deg), sees {cells} grid cells')
            if cells == 0:
                self.get_logger().warn(f'[block 03] {role} sees NO grid cells: check mounts.{role}')
            return True
        except Exception as e:  # noqa: BLE001  (a bad calibration file must not kill the node)
            self.map_error[role] = str(e)
            self.get_logger().error(f'[block 03] {role}: {e}', throttle_duration_sec=5.0)
            return False

    # ---------------------------------------------------------------- main loop
    def _process(self) -> None:
        now = time.monotonic()
        timeout = float(self.p('camera_timeout_s'))
        down_w = int(self.p('input_downscale_width'))
        self.mask.set_seed(self._seed())
        images, stamps = {}, []
        for role, msg in self.latest.items():
            if msg is None or now - self.rx_time[role] > timeout:
                continue
            try:
                img = image_to_bgr(msg)
            except ValueError as e:
                self.map_error[role] = str(e)
                continue
            small = resize_to_width(img, down_w)
            if not self._ensure_map(role, small.shape[1], small.shape[0]):
                continue
            images[role] = small
            st = msg.header.stamp
            if st.sec or st.nanosec:
                stamps.append((st.sec, st.nanosec))
        self._update_rates(now)
        if not images:
            self.set_status(NodeStatus.ERROR, 'NO_CAMERAS', self._detail(None))
            return
        t0 = time.perf_counter()
        res = self.mask.process(images, self._classify())
        self.proc_ms = 0.8 * self.proc_ms + 0.2 * (time.perf_counter() - t0) * 1000.0

        grid = LocalGrid()
        if stamps:
            sec, nsec = max(stamps)
            grid.header.stamp.sec, grid.header.stamp.nanosec = int(sec), int(nsec)
        else:
            grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.frame_id
        grid.resolution_m = float(self.grid.res)
        grid.origin_x_m = float(self.grid.x0)
        grid.origin_y_m = float(self.grid.y0)
        grid.rows = grid.cols = int(self.grid.cells)
        grid.kind = u8(res.kind)
        grid.grown = u8(res.grown)
        grid.source_camera = u8(res.source)
        grid.coverage = float(res.coverage)
        grid.connected = int(res.connected)
        self.pub_grid.publish(grid)

        self._publish_debug(res, images, grid.header.stamp, now)
        stale = [r for r in self.topics if r not in images]
        assumed = [r for r, m in self.mask.maps.items() if m.intr.source != 'calibrated']
        uncal = not bool(self.cameras.get('extrinsics_calibrated', False))
        level = NodeStatus.WARN if (stale or assumed or uncal or self.map_error) else NodeStatus.OK
        self.set_status(level, 'RUNNING', self._detail(res, stale, assumed, uncal))

    def _update_rates(self, now: float) -> None:
        dt = now - self._rate_t0
        if dt >= 2.0:
            for r in self.topics:
                self._rates[r] = self.rx_count[r] / dt
                self.rx_count[r] = 0
            self._rate_t0 = now

    def _detail(self, res, stale=(), assumed=(), uncal=False) -> str:
        cams = ' '.join(f'{r}:{self._rates[r]:.0f}Hz' + ('(STALE)' if r in stale else '')
                        for r in self.topics)
        parts = [cams]
        if res is not None:
            parts.append(f'coverage {res.coverage:.2f} road {res.connected} proc {self.proc_ms:.1f}ms')
        if assumed:
            parts.append('ASSUMED intrinsics: ' + ','.join(assumed))
        if uncal:
            parts.append('mounts not calibrated (step 4)')
        if self.map_error:
            parts.append('errors: ' + '; '.join(f'{k}: {v}' for k, v in self.map_error.items()))
        return ' | '.join(parts)

    # ---------------------------------------------------------------- debug (lazy)
    def _publish_debug(self, res, frames, stamp, now) -> None:
        if not bool(self.p('debug.enabled')):
            return
        wanted = {k: p for k, p in self.pub_dbg.items() if p.get_subscription_count() > 0}
        wanted_ov = {r: p for r, p in self.pub_overlay.items()
                     if p.get_subscription_count() > 0 and r in frames}
        if not wanted and not wanted_ov:
            return
        if now - self.last_debug < 1.0 / max(float(self.p('debug.max_rate_hz')), 0.1):
            return
        self.last_debug = now
        q = int(self.p('debug.jpeg_quality'))
        scale = int(self.p('debug.bev_scale'))
        renders = {'warped': lambda: warped_strip(res, list(self.topics), max(1, scale - 1)),
                   'stitched': lambda: bev_view(res.bgr, scale),
                   'mask': lambda: bev_view(mask_colours(res), scale)}
        for key, pub in wanted.items():
            m = jpeg_msg(renders[key](), stamp, self.frame_id, q)
            if m is not None:
                pub.publish(m)
        ow = int(self.p('debug.overlay_width'))
        for role, pub in wanted_ov.items():
            lk = self.overlays.get(role)
            cm = self.mask.maps[role]
            if lk is None or lk.width != ow:
                lk = self.overlays[role] = OverlayLookup(self.grid, cm.intr, cm.mount, ow)
            m = jpeg_msg(lk.draw(frames[role], res), stamp, f'cam_{role}', q)
            if m is not None:
                pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = RoadPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
