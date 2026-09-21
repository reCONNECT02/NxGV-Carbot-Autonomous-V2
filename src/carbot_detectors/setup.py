from glob import glob

from setuptools import setup

package_name = 'carbot_detectors'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # BPU model (tools/bpu_model/unified14 has the package it came from)
        ('share/' + package_name + '/models', glob('models/*.bin')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RISA team',
    maintainer_email='risabot@todo.todo',
    description='Traffic light, boom gate and sign recognition on the RDK X5 BPU (YOLO11n unified14).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bpu_detector = carbot_detectors.bpu_detector:main',
        ],
    },
)
