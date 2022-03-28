from setuptools import setup

package_name = 'webots_ros2_simulations'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))

data_files.append(('share/' + package_name + '/launch', ['launch/simple_robot.launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/simple_world.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/my_simple_robot.urdf']))

data_files.append(('share/' + package_name + '/launch', ['launch/gantry_robot.launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/blocksworld.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/gantry_robot.urdf']))

data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='devis',
    maintainer_email='devis.dalmoro@unitn.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_simple_robot_driver = ' + package_name + '.simple_robot_driver:main',
            'gantry_robot_driver = ' + package_name + '.gantry_robot_driver:main',
        ],
    },
)
