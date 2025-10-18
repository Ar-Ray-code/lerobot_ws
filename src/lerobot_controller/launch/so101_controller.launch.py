import os
import json
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Arguments
    is_sim_arg = DeclareLaunchArgument("is_sim", default_value="True")
    usb_port_arg = DeclareLaunchArgument("usb_port", default_value="/dev/ttyACM0")
    calib_json_arg = DeclareLaunchArgument(
        "calib_json", default_value=os.path.join(
                    get_package_share_directory("lerobot_controller"),
                    "config",
                    "example_calib.json",
                ),
    )

    def setup_nodes(context, *args, **kwargs):
        # Read args at runtime
        is_sim_val = LaunchConfiguration("is_sim").perform(context)

        # Simulation branch: only spawn controllers (Gazebo provides controller_manager)
        if is_sim_val == "True":
            joint_state_broadcaster_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
            )

            arm_controller_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller", "--controller-manager", "/controller_manager"],
            )

            gripper_controller_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
            )

            return [
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
                gripper_controller_spawner,
            ]

        # Hardware branch: build robot_description with feetech ros2_control + calibration
        usb_port_val = LaunchConfiguration("usb_port").perform(context)
        calib_path = LaunchConfiguration("calib_json").perform(context)

        so101_hw_urdf = os.path.join(
            get_package_share_directory("lerobot_description"),
            "urdf",
            "so101_hw.urdf.xacro",
        )

        # default offsets (ticks)
        offsets = {"1": 0, "2": 0, "3": 0, "4": 0, "5": 0, "6": 0}
        try:
            p = Path(calib_path)
            if p.exists():
                with p.open("r") as f:
                    data = json.load(f)
                unit = None
                if isinstance(data, dict):
                    unit = data.get("unit") or data.get("offsets_unit")
                src = data.get("offsets", data) if isinstance(data, dict) else {}
                if isinstance(src, dict):
                    for k in list(offsets.keys()):
                        if k in src:
                            val = src[k]
                            try:
                                if unit:
                                    u = str(unit).lower()
                                    if u in ("tick", "ticks", "step", "steps"):
                                        offsets[k] = int(round(float(val)))
                                    elif u in ("rad", "radian", "radians"):
                                        offsets[k] = int(round(float(val) * 4096.0 / (2.0 * 3.141592653589793)))
                                    elif u in ("deg", "degree", "degrees"):
                                        offsets[k] = int(round(float(val) * 4096.0 / 360.0))
                                    else:
                                        offsets[k] = int(round(float(val)))
                                else:
                                    fv = float(val)
                                    if abs(fv) <= 2 * 3.141592653589793 and (abs(fv) < 10 and (fv != int(fv))):
                                        offsets[k] = int(round(fv * 4096.0 / (2.0 * 3.141592653589793)))
                                    elif abs(fv) <= 360.0:
                                        offsets[k] = int(round(fv * 4096.0 / 360.0))
                                    else:
                                        offsets[k] = int(round(fv))
                            except Exception:
                                pass
        except Exception:
            pass

        robot_description = ParameterValue(
            Command(
                [
                    "xacro ",
                    so101_hw_urdf,
                    " usb_port:=",
                    usb_port_val,
                    " offset_j1:=",
                    str(offsets["1"]),
                    " offset_j2:=",
                    str(offsets["2"]),
                    " offset_j3:=",
                    str(offsets["3"]),
                    " offset_j4:=",
                    str(offsets["4"]),
                    " offset_j5:=",
                    str(offsets["5"]),
                    " offset_j6:=",
                    str(offsets["6"]),
                ]
            ),
            value_type=str,
        )

        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        )

        controller_manager = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description, "use_sim_time": False},
                os.path.join(
                    get_package_share_directory("lerobot_controller"),
                    "config",
                    "so101_controllers.yaml",
                ),
            ],
        )

        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager",
            ],
        )

        arm_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        )

        gripper_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        )

        return [
            robot_state_publisher_node,
            controller_manager,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ]

    return LaunchDescription(
        [
            is_sim_arg,
            usb_port_arg,
            calib_json_arg,
            OpaqueFunction(function=setup_nodes),
        ]
    )
