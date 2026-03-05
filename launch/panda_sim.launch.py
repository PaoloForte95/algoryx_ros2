from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import os

def generate_launch_description():
    prefix = get_package_prefix('algoryx_ros2')
    script_path = os.path.join(prefix, 'lib', 'algoryx_ros2', 'urdf_panda_ros2.py')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    config_path = os.path.join(get_package_share_directory('algoryx_ros2'), 'config', 'moveit_controllers.yaml')

    panda_urdf_path = os.path.join(get_package_share_directory('algoryx_ros2'), 'urdf', 'panda.urdf')
    with open(panda_urdf_path, "r") as f:
     robot_description = f.read()

    
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="moveit_resources_panda_moveit_config")
        .robot_description(file_path=panda_urdf_path)
        .trajectory_execution(file_path=config_path)
        .to_moveit_configs()
)


    urdf_proc = ExecuteProcess(
        cmd=['python3', script_path],
        output='screen'
    )

    sim_bridge_node = Node(
        package='algoryx_ros2',
        executable='sim_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            ],
    )

    controller_bridge_node = Node(
        package='algoryx_ros2',
        executable='controller_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            ],
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("algoryx_ros2"),
        "config",
        "panda_sim.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
           moveit_config.to_dict(),
           {'use_sim_time': use_sim_time},
           {"start_state_max_bounds_error": 0.01}],
        remappings=[("joint_states", "/agx_joint_states")],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {'use_sim_time': use_sim_time},
            {"robot_description": robot_description}
            ],
        remappings=[("joint_states", "/agx_joint_states")])

    # joint_state_publisher_gui = Node(
    #                                 package="joint_state_publisher_gui",
    #                                 executable="joint_state_publisher_gui",
    #                                 name="joint_state_publisher_gui",
    #                                 parameters=[{"robot_description": robot_description},
    #                                             {'use_sim_time': use_sim_time}],
    #                                 remappings=[("joint_states", "/agx_joint_states")])

    #TF
    world2robot_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("joint_states", "/agx_joint_states")],
    )
    
    ld = LaunchDescription()

    ld.add_action(urdf_proc)
    ld.add_action(sim_bridge_node)
    ld.add_action(controller_bridge_node)
    ld.add_action(rviz_node)
    ld.add_action(world2robot_tf_node)
    ld.add_action(robot_state_publisher)
    #ld.add_action(joint_state_publisher_gui)
    ld.add_action(TimerAction(period=5.0, actions=[move_group_node]))

    return ld

