from setuptools import setup

package_name = 'carbot_gui'

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
    description='Browser GUI (calibrate / race modes). Phase 7 extends the base risabot_automode dashboard; phase 1 ships a graph-status page.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_server = carbot_gui.gui_server:main',
        ],
    },
)
