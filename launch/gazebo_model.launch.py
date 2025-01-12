import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():

        # this name has to match the robot name in the Xacro file
        robotXacroName="robot"

        # this is the name of the package
        namePackage = "lawn_robot_project"

        # relative path to the xacro file defining the model
        modelFileRelativePath = "description/robot.urdf.xacro"

        pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)

        # Uncomment to use custom built world
        worldFileRelativePath = "worlds/garden.sdf"
        pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)

        robotDescription = xacro.process_file(pathModelFile).toxml()

        gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),
                                                                                                       'launch','gz_sim.launch.py'))

        guiFileRelativePath = "worlds/gui.config"
        # for custom world model
        gazeboLaunch = IncludeLaunchDescription(
                gazebo_rosPackageLaunch, 
                launch_arguments={
                        'gz_args': f'-r -v -v4 {pathWorldFile} ',               #--gui-config {guiFileRelativePath}
                        'on_exit_shutdown': 'true'
                }.items()
        )
        
        #gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-r -v -v4 empty.sdf'], 'on_exit_shutdown': 'true'}.items())

        # Gazebo node
        spawnModelNodeGazebo = Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                        '-name', robotXacroName,
                        '-topic', 'robot_description',
                ],
                output='screen',
        )

        nodeRobotStatePublisher = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description' : robotDescription,
                             'use_sim_time' : True}]
        )

        bridge_params = os.path.join(
                get_package_share_directory(namePackage),
                'parameters',
                'bridge_parameters.yaml'
        )

        start_gazebo_ros_bridge_cmd = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                        '--ros-args',
                        '-p',
                        f'config_file:={bridge_params}',
                ],
                output = 'screen',
        )

        # Adding the LaserScanToRange Node for both left and right ultrasonic sensors
        laser_scan_to_range_node = Node(
                package='lawn_robot_project',  # Replace with your package name
                executable='scan_to_range.py',  # Replace with your Python script name (without the .py extension)
                name='scan_to_range',
                output='screen',
        )

        ekf_node = Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                os.path.join(get_package_share_directory(namePackage), 'parameters', 'ekf.yaml'),
                {'use_sim_time': True}
                ]
        )

        launchDescriptionObject = LaunchDescription()

        launchDescriptionObject.add_action(gazeboLaunch)

        launchDescriptionObject.add_action(spawnModelNodeGazebo)
        launchDescriptionObject.add_action(nodeRobotStatePublisher)
        launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)
        launchDescriptionObject.add_action(laser_scan_to_range_node)
        launchDescriptionObject.add_action(ekf_node)

        return launchDescriptionObject
        