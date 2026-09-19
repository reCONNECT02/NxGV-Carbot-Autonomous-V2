"""BLOCK 03 - Turn three views into road (V4 perception.js).

Front + left rear-quarter + right rear-quarter images are warped into one
top-down vehicle grid (flat-ground pinhole projection); each cell takes the
strongest view (weight 1/(0.1+depth^2)), pixels are classified road / paint /
other, and the connected road region is grown (4-neighbour) from a seed near
the car. Output: LocalGrid on /carbot/perception/road_grid.
Phase 2 implements the algorithm. Camera roles come from cameras.yaml.
"""
from carbot_common import topics as T
from carbot_common.data import subscribe_cameras
from carbot_common.stub import BlockSpec, run_stub
from carbot_interfaces.msg import LocalGrid
from sensor_msgs.msg import CompressedImage

SPEC = BlockSpec(
    node='road_perception', block='03', title='Warp -> Stitch -> Road Mask', phase=2,
    required=['process_rate_hz', 'grid.cells', 'grid.resolution_m', 'grid.origin_x_m',
              'grid.origin_y_m', 'stitch.weight_eps', 'stitch.min_depth_m',
              'classify.road_max_luma', 'classify.road_max_chroma', 'classify.paint_min_luma',
              'seed.x_min_m', 'seed.x_max_m', 'seed.half_width_m', 'camera_timeout_s',
              'data.cameras', 'vehicle.car_length_m'],
    pubs=[(LocalGrid, T.ROAD_GRID, 10),
          (CompressedImage, T.PERCEPTION_DEBUG_WARPED, 1),
          (CompressedImage, T.PERCEPTION_DEBUG_STITCHED, 1),
          (CompressedImage, T.PERCEPTION_DEBUG_MASK, 1)]
    + [(CompressedImage, T.perception_overlay(r), 1) for r in T.CAMERA_ROLES],
)


def main(args=None):
    run_stub(SPEC, extra=lambda n: subscribe_cameras(n), args=args)
