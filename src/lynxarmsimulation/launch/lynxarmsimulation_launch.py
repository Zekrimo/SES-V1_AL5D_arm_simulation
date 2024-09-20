import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the package
    pkg_path = get_package_share_directory('lynxarmsimulation')
    
    # Define the path to the URDF and RViz config file
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'lynxmotion_arm.urdf')
    rviz_config_file_path = os.path.join(pkg_path, 'rviz','lynxarmsimulation.rviz')

    # Load the URDF file content
    with open(urdf_file_path, 'r') as urdf_file: robot_urdf = urdf_file.read()
    
    ##arm
    arm_state_publisher_node = Node(
        package='lynxarmsimulation',
        executable='ArmStatePublisher',
        name='ArmStatePublisher',
        parameters=[{'robot_description': robot_urdf}],
        
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}],
    )

    ##simulation
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_file_path],
    )
    

    ## cup
    # redefine the path to the URDF file
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'cup.urdf')
    
    # Load the URDF file content
    with open(urdf_file_path, 'r') as urdf_file: cup_urdf = urdf_file.read()
    
    # cup state publisher
    cup_state_publisher = Node(
        package='cupsimulation',
        executable='CupStatePublisher',
        name='CupStatePublisher',
        parameters=[{'cup_description': cup_urdf}],
    )
    
    cup_state_subscriber = Node(
        package='cupsimulation',
        executable='CupStateSubscriber',
        name='CupStateSubscriber',
    )



    # Create and return the LaunchDescription
    ld = LaunchDescription()
    #ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(arm_state_publisher_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(cup_state_publisher)
    ld.add_action(cup_state_subscriber)


    return ld
