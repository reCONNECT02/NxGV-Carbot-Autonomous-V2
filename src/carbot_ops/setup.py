from setuptools import setup

package_name = 'carbot_ops'

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
    description='Operations: calibration wizard, race supervisor (preflight, START), run recorder, system monitor, scoreboard.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_wizard = carbot_ops.calibration_wizard:main',
            'race_supervisor = carbot_ops.race_supervisor:main',
            'run_recorder = carbot_ops.run_recorder:main',
            'system_monitor = carbot_ops.system_monitor:main',
            'scoreboard = carbot_ops.scoreboard:main',
        ],
    },
)
