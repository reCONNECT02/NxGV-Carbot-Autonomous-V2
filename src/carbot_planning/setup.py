from setuptools import setup

package_name = 'carbot_planning'

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
    description='Blocks 01 and 07-13: prior map, global route, mission logic, corridor, local/parking/recovery planners, path tracker.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'track_map_server = carbot_planning.track_map_server:main',
            'global_planner = carbot_planning.global_planner:main',
            'mission_logic = carbot_planning.mission_logic:main',
            'corridor = carbot_planning.corridor:main',
            'local_planner = carbot_planning.local_planner:main',
            'parking_planner = carbot_planning.parking_planner:main',
            'recovery_planner = carbot_planning.recovery_planner:main',
            'path_tracker = carbot_planning.path_tracker:main',
        ],
    },
)
