"""Astra colour camera ONLY (BACKLOG #48): no depth, IR, D2C alignment, point clouds or camera TF.

    ros2 launch carbot_bringup astra_rgb.launch.py width:=640 height:=480 fps:=15

The base astra_mini.launch.py also runs the depth + IR streams, hardware depth-to-colour alignment,
PointCloudXyz + PointCloudXyzrgb and a 10 Hz camera TF, none of which Carbot reads (only
/camera/color/image_raw is used; the camera TFs come from cameras.yaml mounts). It kept the
astra_camera_container at ~67 % of a core on risabot1. width / height / fps have NO default: they
come from cameras.yaml sensors.astra (stack.py and camera_restart.py pass them), so a missing key
is a loud error. Everything else is the base params/astra_mini_params.yaml template unchanged.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def astra_params(width: int, height: int, fps: int) -> dict:
    """Driver parameters for a colour-only Astra (pure, unit tested)."""
    if min(width, height, fps) <= 0:
        raise ValueError(f'astra width/height/fps must be positive, got {width}x{height}@{fps}')
    return {
        'camera_name': 'camera',
        'color_width': int(width), 'color_height': int(height), 'color_fps': int(fps),
        'enable_color': True,
        'enable_ir': False, 'ir_mode': False, 'ir_width': 320, 'ir_height': 240, 'ir_fps': 30,
        'enable_depth': False, 'depth_width': 320, 'depth_height': 240, 'depth_fps': 30,
        'depth_align': False,
        'serial_number': '', 'number_of_devices': 1,
        'uvc_camera.enable': True, 'uvc_camera.format': 'yuyv', 'uvc_camera.vid': 0x0,
        'uvc_camera.pid': 0x0, 'uvc_camera.retry_count': 100,
        'color_roi.x': -1, 'color_roi.y': -1, 'color_roi.width': -1, 'color_roi.height': -1,
        'depth_roi.x': -1, 'depth_roi.y': -1, 'depth_roi.width': -1, 'depth_roi.height': -1,
        'depth_scale': 1,
        'publish_tf': False, 'tf_publish_rate': 10.0,
        'reconnect_timeout': 6.0,
    }


def _launch(context):
    cfg = context.launch_configurations
    params = astra_params(int(cfg['width']), int(cfg['height']), int(cfg['fps']))
    return [ComposableNodeContainer(
        name='astra_camera_container', namespace='',      # same name: the kill patterns still match
        package='rclcpp_components', executable='component_container',
        composable_node_descriptions=[
            ComposableNode(package='astra_camera', plugin='astra_camera::OBCameraNodeFactory',
                           name='camera', namespace='camera', parameters=[params])],
        output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('width', description='colour width, from cameras.yaml sensors.astra.width'),
        DeclareLaunchArgument('height', description='colour height, from cameras.yaml sensors.astra.height'),
        DeclareLaunchArgument('fps', description='colour fps, from cameras.yaml sensors.astra.fps'),
        OpaqueFunction(function=_launch),
    ])
