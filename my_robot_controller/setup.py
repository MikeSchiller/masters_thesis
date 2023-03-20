from setuptools import setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mike',
    maintainer_email='schimi02@thu.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            
            "draw_circle = my_robot_controller.draw_circle:main",
            "code_mayer = my_robot_controller.code_mayer2:main", 
            "min_pub = my_robot_controller.min_pub2:main",
            "test_node = my_robot_controller.my_first_node:main",
            "pose_subsrciber = my_robot_controller.pose_subscriber:main",
            "simple_servo_sub = my_robot_controller.simple_servo_sub:main",
            "keyboard_pub = my_robot_controller.keyboard_pub:main",
            "turtle_controller = my_robot_controller.turtle_controller:main",
            "vl53l5cx_node_pub = my_robot_controller.vl53l5cx_node:main",
            "US_test = my_robot_controller.US_test:main",
            "simple_autonomous = my_robot_controller.simple_autonomous:main",
            "Radar_simple = my_robot_controller.Radar_simple:main",
            "processRadar = my_robot_controller.processRadar:main",
            "dualUS = my_robot_controller.dualUS:main",
            "two_Distance_Autonomous = my_robot_controller.two_distance_autonomous:main",
            "gps_module= my_robot_controller.gps_module:main",
            "gps_data_stuff= my_robot_controller.gps_data_stuff:main",
            "gps_translation= my_robot_controller.gps_translation:main",
            "gps_drive_controller = my_robot_controller.gps_drive_controller:main",
            "cmps_pub = my_robot_controller.cmps_pub:main"
            "Car_to_coords = my_robot_controller.cartocoord:main"
        ],
    },
)
