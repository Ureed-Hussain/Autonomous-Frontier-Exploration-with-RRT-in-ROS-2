from setuptools import find_packages, setup

package_name = 'autonomous_explorer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', [
            'launch/autonomous_explorer_launch.py',
            'launch/turtlebot3_world.launch.py'
        ]),
        ('share/' + package_name + '/world', [
            'world/box.world'
        ]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uhreed',
    maintainer_email='ureedhussain214@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer_node = autonomous_explorer.explorer_node:main',
            'frontier_detector = autonomous_explorer.frontier_detector:main',
        ],
    },
)
