from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ur10e", package_name="my_robot_moveit_config")
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner"])  # <-- set default pipeline
        .to_moveit_configs()
    )

    moveit_config.planning_pipelines["pilz_industrial_motion_planner"] = {
        "planning_plugin": "pilz_industrial_motion_planner::PlanningContextLoader",
        "request_adapters": (
            "default_planning_request_adapters/AddTimeOptimalParameterization "
            "default_planning_request_adapters/FixWorkspaceBounds "
            "default_planning_request_adapters/FixStartStateBounds "
            "default_planning_request_adapters/FixStartStateCollision "
            "default_planning_request_adapters/FixStartStatePathConstraints"
        ),
        "start_state_max_bounds_error": 0.1,
    }

    return generate_demo_launch(moveit_config)
