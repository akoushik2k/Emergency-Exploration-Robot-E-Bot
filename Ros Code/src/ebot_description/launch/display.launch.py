from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os
import launch_ros.descriptions
import launch
import xacro
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Generate a launch description for bringing up the robot in a ROS2 environment.

    This launch file handles setting up the robot description, loading it in RViz2, and publishing 
    necessary transformations and state publishers.
    """
    
    ####### Data Input Section ##########
    # Define URDF and XACRO file names and package containing the robot description
    urdf_file = 'ebot_description.urdf'
    xacro_file = "ebot_description.urdf.xacro"
    package_description = "ebot_description"
    use_urdf = False

    # Define initial robot position and orientation
    position = [0.0, 0.0, 0.5]
    orientation = [0.0, 0.0, 0.0]

    # Define robot base name for robot description publication
    robot_base_name = "ebot"
    ####### End Data Input Section ##########

    # Locate the package share directory
    pkg_share = launch_ros.substitutions.FindPackageShare(package=package_description).find(package_description)

    # Define path to RViz config file
    default_rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("ebot_description"), "rviz", "display_default.rviz"]
    )

    # Determine the robot description file path based on URDF or XACRO preference
    if use_urdf:
        # Load URDF file if specified
        robot_desc_path = os.path.join(get_package_share_directory(
            package_description), "robot", urdf_file)
    else:
        # Load XACRO file if specified
        robot_desc_path = os.path.join(get_package_share_directory(
            package_description), "urdf", xacro_file)

    # Process XACRO file to generate the robot description XML
    robot_desc = xacro.process_file(robot_desc_path)
    xml = robot_desc.toxml()

    # RVIZ configuration file path setup
    rviz_config_dir = PathJoinSubstitution(
        [FindPackageShare("ebot_description"), "rviz", "display_default.rviz"]
    )

    # RViz2 Node setup
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    # Robot Description Publisher Node
    # Publishes the robot description XML on the /robot_description topic
    publish_robot_description = Node(
        package='ebot_description',
        executable='robot_description_publisher.py',
        name='robot_description_publisher',
        output='screen',
        arguments=['-xml_string', xml,
                   '-robot_description_topic', '/robot_description'
                   ]
    )

    # Robot State Publisher Node
    # Publishes the current state of the robot using the robot description
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_state_publisher= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': xml}],
        output="screen"
    )

    # Joint State Publisher Node
    # Publishes joint states if no GUI is used
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    # Joint State Publisher GUI Node
    # Provides a graphical interface for manually setting joint states
    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )
    
    # Static Transform Publisher Node
    # Publishes a static transform between the map frame and a dummy link
    tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['1', '0', '0', '0', '0', '0', '1', '/map', '/dummy_link']
    )
    
    return LaunchDescription([
        # Declare launch arguments for configuring the launch environment
        launch.actions.DeclareLaunchArgument(
            name='gui', 
            default_value='True',
            description='Flag to enable joint_state_publisher_gui'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig', 
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='True',
            description='Flag to enable use_sim_time'
        ),
        
        # Add nodes to the launch description
        publish_robot_description,
        joint_state_publisher_node,
        robot_state_publisher,
        rviz_node,
        joint_state_gui,
        tf,
    ])
