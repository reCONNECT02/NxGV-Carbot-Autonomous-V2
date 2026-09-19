import os
from glob import glob

from setuptools import setup

package_name = 'carbot_bringup'


def tree(src):
    """(install_dir, [files]) for every folder below src (keeps sub-folders)."""
    out = []
    for d, _, files in os.walk(src):
        if files:
            out.append((os.path.join('share', package_name, d),
                        [os.path.join(d, f) for f in sorted(files)]))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
    ] + tree('config'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RISA team',
    maintainer_email='risabot@todo.todo',
    description='Launch entry points, YAML parameters/data and root helper scripts.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
