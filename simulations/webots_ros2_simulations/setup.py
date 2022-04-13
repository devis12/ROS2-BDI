from setuptools import setup

package_name = 'webots_ros2_simulations'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))

data_files.append(('share/' + package_name + '/launch', ['launch/simple_robot.launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/simple_world.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/my_simple_robot1.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/my_simple_robot2.urdf']))

data_files.append(('share/' + package_name + '/launch', ['launch/blocks_world.launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/blocksworld.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/blocksworld_no_box_gps.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/gripper_a_robot.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/carrier_a_robot.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/carrier_b_robot.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/carrier_c_robot.urdf']))
boxes = ['a1', 'a2', 'b1', 'b2', 'c1', 'c2']
for box in boxes:
    data_files.append(('share/' + package_name + '/resource', ['resource/box_' + box + '.urdf']))

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
            'gripper_robot_driver = ' + package_name + '.gripper_robot_driver:main',
            'carrier_robot_driver = ' + package_name + '.carrier_robot_driver:main',
        ],
    },
)
