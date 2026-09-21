from setuptools import setup

package_name = 'uwb_localization'

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
    description='Parses the micro-ROS UWB tag JSON (/uwb3/input_json) into per-anchor ranges for block 06. UWB never reaches the servo.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uwb_ranges = uwb_localization.uwb_ranges:main',
            'calib_uwb = uwb_localization.calib_uwb:main',                  # step 10
        ],
    },
)
