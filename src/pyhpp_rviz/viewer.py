import threading

import numpy as np
import pinocchio as pin
from pinocchio.visualize import BaseVisualizer
from attrs import field
from pyhpp_rviz import start_rviz2
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
        
from moveit_msgs.msg import DisplayTrajectory, RobotState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import pyhpp.core as core


from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import time

try:
    import hppfcl

    WITH_HPP_FCL_BINDINGS = True
except ImportError:
    WITH_HPP_FCL_BINDINGS = False


from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import rclpy


class OdometryPublisher(Node):
    """Publish odometry transforms on /odometry  in oder to show the path loaded. maybe will be replaces by simple circular markers in the future"""

    def __init__(self):
        super().__init__("pinnochio_odometry_publisher")
        self.publisher = self.create_publisher(TransformStamped, "odometry", 10)

    def publish(self, parent_frame, child_frame, xyz, quat_xyzw):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent_frame
        msg.child_frame_id = child_frame
        msg.transform.translation.x = xyz[0]
        msg.transform.translation.y = xyz[1]
        msg.transform.translation.z = xyz[2]
        msg.transform.rotation.x = quat_xyzw[0]
        msg.transform.rotation.y = quat_xyzw[1]
        msg.transform.rotation.z = quat_xyzw[2]
        msg.transform.rotation.w = quat_xyzw[3]
        self.publisher.publish(msg)


class JointStatePublisher(Node):
    """Publish posotions of all joints on /joint_states (or /<namespace>/joint_states)"""

    def __init__(self):
        super().__init__("pinnochio_joint_state_publisher")
        self.publishers_map = {}
        self.last_states = {}
        self.timer = self.create_timer(0.02, self._republish)  # 50 Hz

    def _republish(self):
        for topic, (publisher, names, positions) in self.last_states.items():
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = names
            msg.position = positions
            publisher.publish(msg)

    def _get_publisher(self, namespace):
        ns = namespace.strip("/")
        topic = f"{ns}/joint_states" if ns else "joint_states"
        if topic not in self.publishers_map:
            self.publishers_map[topic] = self.create_publisher(JointState, topic, 10)
        return self.publishers_map[topic]

    def publish(self, namespace, names, positions):
        ns = namespace.strip("/")
        topic = f"{ns}/joint_states" if ns else "joint_states"
        self._get_publisher(namespace)  # ensure created
        self.last_states[topic] = (
            self.publishers_map[topic],
            list(names),
            list(positions),
        )


class TFBroadcasterNode(Node):
    """
    Publish freeflyer transforms on /tf via TransformBroadcaster.
    For each freeflyer joint, broadcast a TF from parent (ex:"world") to  Child frame (ex:"<namespace>/base_link").
    """

    def __init__(self):
        super().__init__("pinnochio_tf_broadcaster")
        # TransformBroadcaster publie directement sur /tf (type TFMessage)
        self.broadcaster = TransformBroadcaster(self)
        # clé: child_frame_id → dict avec parent, xyz, quat
        self.last_transforms = {}
        self.timer = self.create_timer(0.02, self._republish)  # 50 Hz

    def _republish(self):
        if not self.last_transforms:
            return
        transforms = []
        now = self.get_clock().now().to_msg()
        for child_frame, data in self.last_transforms.items():
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = data["parent_frame"]  # ex: "world"
            t.child_frame_id = child_frame  # ex: "box/base_link"
            t.transform.translation.x = data["xyz"][0]
            t.transform.translation.y = data["xyz"][1]
            t.transform.translation.z = data["xyz"][2]
            # Ordre Pinocchio freeflyer: [x, y, z, qx, qy, qz, qw]
            t.transform.rotation.x = data["quat"][0]
            t.transform.rotation.y = data["quat"][1]
            t.transform.rotation.z = data["quat"][2]
            t.transform.rotation.w = data["quat"][3]
            transforms.append(t)
        self.broadcaster.sendTransform(transforms)

    def publish(self, parent_frame, child_frame, xyz, quat_xyzw):
        """
        Mettre à jour le transform pour child_frame.
        xyz    : [x, y, z]
        quat   : [qx, qy, qz, qw]  (ordre Pinocchio)
        """
        self.last_transforms[child_frame] = {
            "parent_frame": parent_frame,
            "xyz": list(xyz),
            "quat": list(quat_xyzw),
        }


class RVizVisualizer(BaseVisualizer):
    """Pinocchio RViz2 visualizer (ROS 2)"""

    def __init__(self):
        self.robot = None
        self.model: pin.Model = pin.Model()
        self.data: pin.Data = pin.Data()
        self.geom_model: pin.GeometryModel = None
        self.visual_model: pin.GeometryModel = None
        self.visual_data: pin.GeometryData = None
        self.publisher_frame_id = "world"

        self.joint_state_publisher = None
        self.joint_state_map = {}

        self.tf_broadcaster = None
        self.freeflyer_map = {}

        self._executor = None
        self._spin_thread = None

    def initViewer(self, config_generator=None, robot=None):
        self.display_pub = None   # ← Ajoute ça
        self.current_q = None
        self.model = robot.model()
        self.data = self.model.createData()
        if callable(robot.geomModel):
            self.geom_model = robot.geomModel()
        if callable(robot.visualModel):
            self.visual_model = robot.visualModel()
        if self.visual_model is not None:
            self.visual_data = self.visual_model.createData()

        self.robot = robot

        if config_generator is None:
            raise ValueError(
                "Config generator is required for initializing RVizVisualizer"
            )

        config_generator.generate_config(config_path="/tmp/test_config.rviz")
        config_generator.write_nodes_yaml("/tmp/nodes.yaml")
        start_rviz2(
            RVIZ_CONFIG_PATH="/tmp/test_config.rviz", NODES_YAML_PATH="/tmp/nodes.yaml"
        )

        if not rclpy.ok():
            rclpy.init()

        self.joint_state_publisher = JointStatePublisher()
        self.tf_broadcaster = TFBroadcasterNode()

        self._build_map_for_publisher()

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self.joint_state_publisher)
        self._executor.add_node(self.tf_broadcaster)

        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    # ====================== Main display function ======================

    def __call__(self, q):
        self.display(q)

    def display(self, q=None):
        if q is not None:
            pin.forwardKinematics(self.model, self.data, q)
        if q is not None and self.joint_state_publisher is not None:
            self._publish_scene(q)



    # def displayPath(self, path: core.bindings.Path, dt: float = 0.05, 
    #             topic_name: str = "display_planned_path"):
    #     """
    #     Affiche un path HPP dans RViz avec DisplayTrajectory (style MoveIt)
    #     """
    #     # Création du publisher la première fois
    #     if not hasattr(self, 'display_pub') or self.display_pub is None:
    #         self.display_pub = self.joint_state_publisher.create_publisher(  # ← Important
    #             DisplayTrajectory, 
    #             topic_name, 
    #             10
    #         )

    #     display = DisplayTrajectory()
        
    #     # État initial du robot
    #     if self.current_q is not None:
    #         display.trajectory_start = self._create_robot_state(self.current_q)

    #     traj = JointTrajectory()
        
    #     # Récupération des joint names (on prend le premier namespace)
    #     if self.joint_state_map:
    #         main_ns = next(iter(self.joint_state_map.keys()))
    #         traj.joint_names = [name for name, _ in self.joint_state_map[main_ns]]
    #     else:
    #         traj.joint_names = [self.model.names[i] for i in range(1, self.model.njoints)]

    #     t = 0.0
    #     step = 0
    #     while t <= path.length() + 1e-6:
    #         q_eval = path.eval(t)[0]
    #         q_vec = np.asarray(q_eval).reshape(-1)

    #         point = JointTrajectoryPoint()
    #         point.positions = [float(x) for x in q_vec]
    #         point.time_from_start = rclpy.duration.Duration(seconds=t).to_msg()
            
    #         traj.points.append(point)
    #         t += dt
    #         step += 1

    #     display.trajectory.append(traj)
    #     self.display_pub.publish(display)
        
    #     print(f"✅ Path publié ({len(traj.points)} points) sur /{topic_name}")


    def displayPath(self, path: core.bindings.Path, dt: float = 0.03, 
                topic_name: str = "hpp_path"):
        """
        Visualise le path HPP avec MarkerArray (beaucoup plus stable que moveit_msgs)
        """
        if not hasattr(self, 'marker_pub') or self.marker_pub is None:
            self.marker_pub = self.joint_state_publisher.create_publisher(
                MarkerArray, topic_name, 10
            )

        marker_array = MarkerArray()
        
        # Marker pour la ligne (le chemin)
        line_marker = Marker()
        line_marker.header.frame_id = "world"
        line_marker.header.stamp = self.joint_state_publisher.get_clock().now().to_msg()
        line_marker.ns = "hpp_path"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.015  # épaisseur
        line_marker.color.r = 0.0
        line_marker.color.g = 0.7
        line_marker.color.b = 1.0
        line_marker.color.a = 0.9

        # Marker pour les points (optionnel)
        points_marker = Marker()
        points_marker.header.frame_id = "world"
        points_marker.header.stamp = line_marker.header.stamp
        points_marker.ns = "hpp_path_points"
        points_marker.id = 1
        points_marker.type = Marker.SPHERE_LIST
        points_marker.action = Marker.ADD
        points_marker.scale.x = 0.025
        points_marker.scale.y = 0.025
        points_marker.scale.z = 0.025
        points_marker.color.r = 1.0
        points_marker.color.g = 0.5
        points_marker.color.b = 0.0
        points_marker.color.a = 0.8

        t = 0.0
        while t <= path.length() + 1e-6:
            q = path.eval(t)[0]
            q_vec = np.asarray(q).reshape(-1)

            # Récupérer la position d'un point de référence (ex: gripper ou base)
            pin.forwardKinematics(self.model, self.data, q_vec)
            
            # On prend la position du dernier link (à adapter selon ton robot)
            end_effector_pos = self.data.oMf[self.model.getFrameId("panda_hand")] if "panda" in str(self.model) else self.data.oMf[-1]
            
            p = Point()
            p.x = end_effector_pos.translation[0]
            p.y = end_effector_pos.translation[1]
            p.z = end_effector_pos.translation[2]
            
            line_marker.points.append(p)
            points_marker.points.append(p)
            
            t += dt

        marker_array.markers.append(line_marker)
        marker_array.markers.append(points_marker)
        
        self.marker_pub.publish(marker_array)
        print(f"✅ Path visualisé avec MarkerArray ({len(line_marker.points)} points)")

    # def displayPath(self, path: core.bindings.Path, dt=0.07):

    #     threading.Thread(
    #         target=self._display_path_thread, args=(path, dt), daemon=True
    #     ).start()
    #     pass

    def _display_path_thread(self, path: core.bindings.Path, dt):
        t = 0.0
        while t <= path.length():
            q: tuple = path.eval(t)
            q = q[0]
            self._publish_scene(q)
            t += dt

    # ====================== Méthodes abstraites ======================

    def captureImage(self, w=None, h=None):
        raise NotImplementedError

    def disableCameraControl(self):
        pass

    def enableCameraControl(self):
        pass

    def drawFrameVelocities(self, *args, **kwargs):
        pass

    def setBackgroundColor(self, *args, **kwargs):
        pass

    def setCameraPose(self, pose):
        pass

    def setCameraPosition(self, position):
        pass

    def setCameraTarget(self, target):
        pass

    def setCameraZoom(self, zoom):
        pass

    def displayCollisions(self, visibility: bool):
        pass

    def displayVisuals(self, visibility: bool):
        pass

    # ====================== Mapping joints ======================

    def _build_map_for_publisher(self):
        self.joint_state_map = {}
        self.freeflyer_map = {}

        for joint_id in range(1, self.model.njoints):
            name = self.model.names[joint_id]
            joint: pin.JointModel = self.model.joints[joint_id]
            shortname = joint.shortname()

            if "/" in name:
                namespace, joint_name = name.split("/", 1)
            else:
                namespace, joint_name = "", name

            if joint.nq == 1:
                self.joint_state_map.setdefault(namespace, []).append(
                    (joint_name, joint.idx_q)
                )

            elif joint.nq == 7:
                # Freeflyer → TF broadcast world → <namespace>/base_link
                # Le root link est le 1er body attaché à ce joint
                root_link = self._get_root_link_for_joint(joint_id)
                child_frame = f"{namespace}/{root_link}" if namespace else root_link
                self.freeflyer_map.setdefault(namespace, []).append(
                    (joint_name, joint.idx_q, child_frame)
                )
            else:
                raise ValueError(
                    f"Not Yet Supported joint type for joint '{name}' with nq={joint.nq} of type {shortname}"
                )

            print(
                f"  joint '{name}' | nq={joint.nq} | type={shortname} | idx_q={joint.idx_q}"
            )

        print("joint_state_map :", self.joint_state_map)
        print("freeflyer_map   :", self.freeflyer_map)

    def _get_root_link_for_joint(self, joint_id: int) -> str:
        """Retourne le nom du premier frame (body) attaché à ce joint."""
        for frame in self.model.frames:
            frame: pin.Frame
            if frame.parentJoint == joint_id and frame.type == pin.FrameType.BODY:
                # Enlever le namespace (ex: "box/base_link" → "base_link")
                fname = frame.name
                if "/" in fname:
                    fname = fname.split("/", 1)[1]
                return fname
        # Fallback: utiliser le nom du joint sans namespace
        name = self.model.names[joint_id]
        return name.split("/", 1)[1] if "/" in name else name

    # ====================== Publication ======================

    def _publish_scene(self, q):
        q_vec = np.asarray(q).reshape(-1)
        # 1. Joints classiques → /panda/joint_states, etc.
        for namespace, joints in self.joint_state_map.items():
            names = []
            positions = []
            for joint_name, idx_q in joints:
                names.append(joint_name)
                positions.append(float(q_vec[idx_q]))
            if names:
                print(
                    f"Publishing JointState for namespace '{namespace}': {list(zip(names, positions))}"
                )
                self.joint_state_publisher.publish(namespace, names, positions)

        # 2. Freeflyers → /tf via TransformBroadcaster
        for namespace, joints in self.freeflyer_map.items():
            for joint_name, idx_q, child_frame in joints:
                # Pinocchio freeflyer layout: [x, y, z, qx, qy, qz, qw]
                xyz = q_vec[idx_q : idx_q + 3]
                quat = q_vec[idx_q + 3 : idx_q + 7]  # qx qy qz qw

                print(
                    f"Publishing TF: world → '{child_frame}' | xyz={list(xyz)} quat={list(quat)}"
                )
                self.tf_broadcaster.publish(
                    parent_frame="world",
                    child_frame=child_frame,
                    xyz=xyz,
                    quat_xyzw=quat,
                )
