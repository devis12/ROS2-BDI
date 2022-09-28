from os import walk
from os import path
from setuptools import setup

package_name = 'litter_world'

CURR_DIR = path.dirname(path.realpath(__file__))
init_files = []
for root, dirs, files in walk(path.join(CURR_DIR, 'init')):
    for file in files:
        init_files.append(('share/' + package_name + '/init', ['init/'+file]))

asset_files = []
for root, dirs, files in walk(path.join(CURR_DIR, 'asset')):
    for file in files:
        asset_files.append(('share/' + package_name + '/asset', ['asset/'+file]))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        *init_files,
        *asset_files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='devis',
    maintainer_email='devis.dalmoro@unitn.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    include_package_data=True,
    entry_points={
        'console_scripts': [
            "litter_world_ros2_controller = litter_world.litter_world_ros2_controller:main",
        ],
    },
)
