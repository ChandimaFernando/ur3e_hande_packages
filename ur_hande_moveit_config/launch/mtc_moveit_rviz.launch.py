import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition

from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml
from launch_ros.descriptions import ParameterValue

import pdb

def get_rviz_config():

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_hande_moveit_config"), "config", "moveit.rviz"]
    )

    return rviz_config_file

def get_robot_description_semantic():

    robot_description_semantic_content = ParameterValue(Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_hande_moveit_config"), "config", "ur.srdf"]
            ),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            "",
            " ",
        ])
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    return robot_description_semantic

def get_robot_description_kinematics():

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("ur_hande_moveit_config"), "config", "kinematics.yaml"]
    )
    return robot_description_kinematics

# def get_planning_description():
#     planning_params = PathJoinSubstitution(
#         [FindPackageShare("ur_description"), "config", "ur3e", "ompl_planning.yaml"]
#     )
#     return { "planner_description": planning_params}


def get_robot_description():
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur3e_hande_description"), "config", "ur3e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur3e_hande_description"), "config", "ur3e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur3e_hande_description"), "config", "ur3e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur3e_hande_description"), "config", "ur3e", "visual_parameters.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur3e_hande_description"), "urdf", "ur.urdf.xacro"]),
            " ",
            "robot_ip:=192.168.56.101",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
           "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur3e",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}
    return robot_description


def generate_launch_description():

    rviz_config_file = get_rviz_config()
    robot_description_semantic = get_robot_description_semantic()

    robot_description_kinematics = get_robot_description_kinematics()
    robot_description = get_robot_description()
    
    # ------------------------------------------- #
    #Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur_hande_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)
    # pdb.set_trace()
    # Trajectory Execution Configuration
    # controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    controllers_yaml = load_yaml("ur_hande_moveit_config", "config/moveit_controllers.yaml")

    # the scaled_joint_trajectory_controller does not work on fake hardware
    # controllers_yaml = load_yaml("ur_hande_moveit_config", "config/moveit_controllers.yaml")
    # controllers_yaml = load_yaml("ur_moveit_config", "config/rcontrollers.yaml")

    # the scaled_joint_trajectory_controller does not work on fake hardware
    # change_controllers = context.perform_substitution(use_fake_hardware)
    # if change_controllers == "true":
        
    controllers_yaml["scaled_joint_trajectory_controller"]["default"] = True
    # controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # warehouse_ros_config = {
    #     "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
    #     "warehouse_host": warehouse_sqlite_path,
    # }

    # ----------------------------------------- #

    # Servo node for realtime control
    servo_yaml = load_yaml("ur_hande_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    return LaunchDescription([

        Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            # robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
            move_group_capabilities,
            #warehouse_ros_config,
        ],
        ),

        
        Node(
            package="rviz2",
            condition=IfCondition("true"),
            executable="rviz2",
            name="rviz2_moveit",
            output="log",
            arguments=["-d", rviz_config_file],
            parameters=[
                robot_description,
                robot_description_semantic,
                ompl_planning_pipeline_config,
                robot_description_kinematics,
        ]
        ),

        Node(
        package="moveit_servo",
        condition=IfCondition("true"),
        executable="servo_node_main",
        arguments=[{'use_intra_process_comms' : True}],
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
        ],
        output="screen",
        ),
        
            # Static TF
    	Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "right_finger"],
    	)


    ])
