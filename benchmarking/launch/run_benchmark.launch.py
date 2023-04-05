import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    moveit_ros_benchmarks_config = (
        ParameterBuilder("moveit_resources_benchmarking")
        .yaml(
            parameter_namespace="benchmark_config",
            file_path="config/run_benchmark.yaml",
        )
        .to_dict()
    )

    moveit_configs = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .planning_pipelines("ompl", ["ompl"])
        .moveit_cpp(
            file_path=get_package_share_directory("moveit_resources_benchmarking")
            + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
    )

    # Load additional OMPL pipeline
    ompl_planning_pipeline_config2 = {
        "ompl_2": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """\
                default_planner_request_adapters/AddTimeOptimalParameterization \
                default_planner_request_adapters/FixWorkspaceBounds \
                default_planner_request_adapters/FixStartStateBounds \
                default_planner_request_adapters/FixStartStateCollision \
                default_planner_request_adapters/FixStartStatePathConstraints \
              """,
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "moveit_resources_panda_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config2["ompl_2"].update(ompl_planning_yaml)

    # Add third OMPL config
    # Load additional OMPL pipeline
    ompl_planning_pipeline_config3 = {
        "ompl_3": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """\
                default_planner_request_adapters/AddTimeOptimalParameterization \
                default_planner_request_adapters/FixWorkspaceBounds \
                default_planner_request_adapters/FixStartStateBounds \
                default_planner_request_adapters/FixStartStateCollision \
                default_planner_request_adapters/FixStartStatePathConstraints \
              """,
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config3["ompl_3"].update(ompl_planning_yaml)

    # Add fourth config
    # Load additional OMPL pipeline
    ompl_planning_pipeline_config4 = {
        "ompl_4": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """\
                default_planner_request_adapters/AddTimeOptimalParameterization \
                default_planner_request_adapters/FixWorkspaceBounds \
                default_planner_request_adapters/FixStartStateBounds \
                default_planner_request_adapters/FixStartStateCollision \
                default_planner_request_adapters/FixStartStatePathConstraints \
              """,
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config4["ompl_4"].update(ompl_planning_yaml)

    # Add fourth config
    # Load additional OMPL pipeline
    ompl_planning_pipeline_config5 = {
        "ompl_5": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """\
                default_planner_request_adapters/AddTimeOptimalParameterization \
                default_planner_request_adapters/FixWorkspaceBounds \
                default_planner_request_adapters/FixStartStateBounds \
                default_planner_request_adapters/FixStartStateCollision \
                default_planner_request_adapters/FixStartStatePathConstraints \
              """,
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config5["ompl_5"].update(ompl_planning_yaml)

    # Add fourth config
    # Load additional OMPL pipeline
    ompl_planning_pipeline_config6 = {
        "ompl_6": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """\
                default_planner_request_adapters/AddTimeOptimalParameterization \
                default_planner_request_adapters/FixWorkspaceBounds \
                default_planner_request_adapters/FixStartStateBounds \
                default_planner_request_adapters/FixStartStateCollision \
                default_planner_request_adapters/FixStartStatePathConstraints \
              """,
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config6["ompl_6"].update(ompl_planning_yaml)

    sqlite_database = (
        get_package_share_directory("moveit_resources_benchmarking")
        + "/databases/panda_kitchen_test_db.sqlite"
    )

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "benchmark_config": {
            "warehouse": {
                "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
                "host": sqlite_database,
                "port": 33828,
                "scene_name": "",  # If scene name is empty, all scenes will be used
                "queries_regex": ".*",
            },
        },
    }

    # moveit_ros_benchmark demo executable
    moveit_ros_benchmarks_node = Node(
        name="moveit_run_benchmark",
        package="moveit_ros_benchmarks",
        # prefix='gdb --ex=run --args',
        executable="moveit_run_benchmark",
        output="screen",
        parameters=[
            moveit_ros_benchmarks_config,
            moveit_configs.to_dict(),
            warehouse_ros_config,
            ompl_planning_pipeline_config2,
            ompl_planning_pipeline_config3,
            ompl_planning_pipeline_config4,
            ompl_planning_pipeline_config5,
            ompl_planning_pipeline_config6,
        ],
    )

    return LaunchDescription([moveit_ros_benchmarks_node])
