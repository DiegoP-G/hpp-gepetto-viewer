import yaml
from pyhpp import tools


def normalize_namespace(ns: str) -> str:
    """Ensure namespace starts with a single slash and has no trailing slashes."""
    ns = ns.strip("/")
    return f"/{ns}" if ns else ""


class RvizConfigGenerator:
    """Generates RViz configuration files for visualizing Pinocchio models."""

    def __init__(self):
        self.node_specs = []
        self.prefixUsed = []

        self.panels = [
            {"Class": "rviz_common/Displays", "Name": "Displays"},
            {"Class": "rviz_common/Views", "Name": "Views"},
        ]

        self.visualization_manager = {
            "Displays": [
                {
                    "Class": "rviz_default_plugins/Grid",
                    "Name": "Grid",
                    "Value": True,
                    "Alpha": 0.8,
                }
            ],
            "Global Options": {"Fixed Frame": "base"},
            "Tools": [{"Class": "rviz_default_plugins/MoveCamera"}],
            "Value": True,
            "Views": {
                "Current": {
                    "Class": "rviz_default_plugins/Orbit",
                    "Distance": 1.7,
                    "Name": "Current View",
                    "Pitch": 0.33,
                    "Value": "Orbit (rviz)",
                    "Yaw": 5.5,
                }
            },
        }

        self.window_geometry = {"Height": 800, "Width": 1200}

        self.config = {
            "Panels": self.panels,
            "Visualization Manager": self.visualization_manager,
            "Window Geometry": self.window_geometry,
        }

    def generate_config(self, config_path=None):
        """Write the RViz configuration to the specified file path."""
        if config_path is not None:
            with open(config_path, "w") as f:
                yaml.dump(self.config, f, default_flow_style=False, sort_keys=False)
        else:
            return yaml.dump(self.config, default_flow_style=False, sort_keys=False)

    def viewsOrbitOptions(
        self,
        name="Current View",
        distance=1.7,
        pitch=0.33,
        yaw=5.5,
        value="Orbit (rviz)",
    ):
        self.visualization_manager["Views"]["Current"] = {
            "Class": "rviz_default_plugins/Orbit",
            "Distance": distance,
            "Name": name,
            "Pitch": pitch,
            "Value": value,
            "Yaw": yaw,
        }

    def globalOptions(self, frame_name="base"):
        self.visualization_manager["Global Options"]["Fixed Frame"] = frame_name

    def addStaticTransform(
        self, parent_frame, child_frame, xyz=(0, 0, 0), rpy=(0, 0, 0), namespace=None
    ):
        """
        Add a static transform publisher.
        Note: child_frame doit déjà être préfixé si nécessaire.
        Ex: parent_frame="world", child_frame="panda/panda_link0"
        """
        args = [
            str(xyz[0]),
            str(xyz[1]),
            str(xyz[2]),
            str(rpy[0]),
            str(rpy[1]),
            str(rpy[2]),
            parent_frame,
            child_frame,
        ]
        self.node_specs.append(
            {
                "package": "tf2_ros",
                "executable": "static_transform_publisher",
                "namespace": namespace,
                "arguments": args,
                "output": "screen",
            }
        )

    def addTFPlugin(self, TFName=None, TFValue=True):
        if TFName is None:
            TFName = "TF"

        self.visualization_manager["Displays"].append(
            {"Class": "rviz_default_plugins/TF", "Name": TFName, "Value": TFValue}
        )

    def addObject(
        self, urdf_path, namespace=None, RobotModelName=None, DisplayValue=True
    ):
        """
        add a robot to the viewer.
        The robot_state_publisher is launched with tf_prefix=namespace,
        which prefixes ALL published TF frames.
        Ex: namespace="box" → frames "box/base_link", "box/..."

        For addStaticTransform, use the prefixed child_frame:
          cg.addStaticTransform("world", "box/base_link")
        """

        robot_desc = None
        with open(tools.xacro.retrieve_resource(urdf_path), "r") as f:
            robot_desc = f.read()
        if (robot_desc is None) or (robot_desc.strip() == ""):
            raise ValueError(f"URDF file at {urdf_path} is empty or could not be read.")

        suffix = f"_{namespace}" if namespace else ""
        if RobotModelName is None:
            RobotModelName = f"Robot Model{suffix}"

        prefix = normalize_namespace(namespace) if namespace else ""
        if prefix in self.prefixUsed:
            raise ValueError(
                f"Namespace '{prefix}' already used. Provide a unique namespace per robot."
            )
        self.prefixUsed.append(prefix)

        if prefix == "":
            raise ValueError("Namespace cannot be empty. Provide a valid namespace.")

        ns_value = prefix.strip("/")

        self.visualization_manager["Displays"].append(
            {
                "Class": "rviz_default_plugins/RobotModel",
                "Name": RobotModelName,
                "Value": DisplayValue,
                "Description Topic": {"Value": f"{prefix}/robot_description"},
                "TF Prefix": {"Value": prefix},
            }
        )

        parameters = [
            {"robot_description": robot_desc},
            {"frame_prefix": ns_value + "/"},
        ]

        remappings = [
            ["tf", "/tf"],
            ["tf_static", "/tf_static"],
        ]

        self.node_specs.append(
            {
                "package": "robot_state_publisher",
                "executable": "robot_state_publisher",
                "namespace": ns_value,
                "parameters": parameters,
                "remappings": remappings,
                "output": "screen",
            }
        )

    def getNodeSpecs(self):
        return self.node_specs

    def write_nodes_yaml(self, path: str):
        with open(path, "w") as f:
            yaml.dump(self.node_specs, f, default_flow_style=False, sort_keys=False)
