from setuptools import setup

package_name = 'carbot_localization'

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
    description='Blocks 05 (smooth local pose, never jumps) and 06 (coarse UWB-aided course position).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'local_pose = carbot_localization.local_pose:main',
            'global_pose = carbot_localization.global_pose:main',
        ],
    },
)
