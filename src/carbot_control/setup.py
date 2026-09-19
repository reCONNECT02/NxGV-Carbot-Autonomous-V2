from setuptools import setup

package_name = 'carbot_control'

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
    description='Blocks 14-16: safety checks, the single command owner (actuator mapping to the base servo_controller) and the tunnel bridge.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_monitor = carbot_control.safety_monitor:main',
            'command_owner = carbot_control.command_owner:main',
            'tunnel_bridge = carbot_control.tunnel_bridge:main',
        ],
    },
)
