import launch
import launch_ros.actions

def generate_launch_description():
    
    # Nodes
    robot_patrol_node = launch_ros.actions.Node(
        package='robot_patrol',
        executable='patrol',
        arguments=[],
        output='screen',
    )

    return launch.LaunchDescription([
        robot_patrol_node
    ])