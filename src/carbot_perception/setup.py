from setuptools import setup

package_name = 'carbot_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RISA team',
    maintainer_email='risabot@todo.todo',
    description='Blocks 03 (warp, stitch, road mask) and 04 (local memory) plus GUI/rosbag camera streams.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'road_perception = carbot_perception.road_perception:main',
            'local_memory = carbot_perception.local_memory:main',
            'camera_preview = carbot_perception.camera_preview:main',
        ],
    },
)
