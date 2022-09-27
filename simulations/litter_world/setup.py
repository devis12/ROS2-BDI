from setuptools import setup

package_name = 'litter_world'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/init', ['init/problem4x4.json']),
        ('share/' + package_name + '/init', ['init/problem7x7.json']),
        ('share/' + package_name + '/init', ['init/problem8x8.json']),
        ('share/' + package_name + '/asset', ['asset/paper_agent.png']),
        ('share/' + package_name + '/asset', ['asset/plastic_agent.png']),
        ('share/' + package_name + '/asset', ['asset/person.png']),
        ('share/' + package_name + '/asset', ['asset/plastic_bin.png']),
        ('share/' + package_name + '/asset', ['asset/paper_bin.png']),
        ('share/' + package_name + '/asset', ['asset/plastic_litter.png']),
        ('share/' + package_name + '/asset', ['asset/paper_litter.png']),
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
