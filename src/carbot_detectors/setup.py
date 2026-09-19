from setuptools import setup

package_name = 'carbot_detectors'

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
    description='Traffic light (red/green), speed-bump sign and boom gate (open/closed) recognition on the RDK X5 BPU.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bpu_detector = carbot_detectors.bpu_detector:main',
        ],
    },
)
