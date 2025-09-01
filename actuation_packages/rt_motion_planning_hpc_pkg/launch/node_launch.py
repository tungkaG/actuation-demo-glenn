import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rt_motion_planning_hpc_pkg',
            executable='node',
            name='node',
            output='screen',
        )
    ])