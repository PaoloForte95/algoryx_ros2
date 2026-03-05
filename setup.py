from setuptools import setup, find_packages
import os
from glob import glob
from collections import defaultdict

package_name = 'algoryx_ros2'

def install_tree(src_root, dst_root):
    """
    Install all files under src_root into dst_root, preserving the directory tree.
    Example: meshes/panda/visual/link1.dae
      -> share/<pkg>/meshes/panda/visual/link1.dae
    """
    files_by_dir = defaultdict(list)
    for path in glob(os.path.join(src_root, "**", "*"), recursive=True):
        if os.path.isfile(path):
            rel_dir = os.path.dirname(os.path.relpath(path, src_root))
            install_dir = os.path.join(dst_root, src_root, rel_dir)  # keep src_root name
            files_by_dir[install_dir].append(path)
    return list(files_by_dir.items())

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ] + install_tree('meshes', os.path.join('share', package_name)),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pofe',
    maintainer_email='paolo.forte@oru.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},

    scripts=[
        'scripts/urdf_panda_ros2.py',
    ],

    entry_points={
        'console_scripts': [
            'sim_bridge = algoryx_ros2.sim_bridge:main',
            'controller_bridge = algoryx_ros2.controller_bridge:main',
        ],
    },
)
