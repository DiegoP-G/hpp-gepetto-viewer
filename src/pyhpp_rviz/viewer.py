import os
import threading
import time
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from dataclasses import dataclass, field

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from tf2_ros import TransformBroadcaster

from pinocchio.visualize import BaseVisualizer
import pyhpp.core as core

try:
    import hppfcl
except ImportError:
    hppfcl = None


@dataclass
class _PathPlayerState:
    current: object = None
    paths: dict = field(default_factory=dict)
    counter: int = 0
    playing: bool = False
    thread: threading.Thread = None
    speed: float = 1.0
    fps: int = 60


class RVizVisualizer(BaseVisualizer):
    """
    Visualiseur Pinocchio pour RViz utilisant UNIQUEMENT MarkerArray
    (sans RobotModel / sans URDF)
    """

    def __init__(self):
        super().__init__()
        self.model = None
        self.data = None
        self.visual_model = None

        self._path_player = _PathPlayerState()
        self.current_q = None

        # ROS 2
        self.node = None
        self.marker_pub = None
        self.tf_broadcaster = None
        self.executor = None
        self.spin_thread = None

        self._robot_markers = None
        self._initialized = False

    def initViewer(self, robot=None, node_name="hpp_rviz_marker"):
        if robot is None:
            raise ValueError("robot is required")

        self._robot = robot
        self.model = robot.model()
        self.data = self.model.createData()

        if hasattr(robot, 'visualModel') and callable(robot.visualModel):
            self.visual_model = robot.visualModel()
            self.visual_data = self.visual_model.createData()

        # ROS 2
        if not rclpy.ok():
            rclpy.init()
        
        self.node = Node(node_name)
        self.PathMarker_pub = self.node.create_publisher(Path, "hpp_path", 10)
        self.ModelMarker_pub = self.node.create_publisher(MarkerArray, "hpp_robot", 10)
        self.tf_broadcaster = TransformBroadcaster(self.node)

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self._initialized = True
        print(f"✅ RVizVisualizer (Marker Only) initialized on node '{node_name}'")

    def __call__(self, q):
        self.display(q)

    def display(self, q=None):
        if q is None:
            return
        self.current_q = np.asarray(q).reshape(-1)
        pin.forwardKinematics(self.model, self.data, self.current_q)

        if self.visual_model is not None:
            pin.updateGeometryPlacements(
                self.model, self.data, self.visual_model, self.visual_data
            )
        
        self._publish_robot_as_markers()

    def _publish_robot_as_markers(self):
        """Affiche le robot complet avec les visuels Pinocchio si possible."""
        if self.ModelMarker_pub is None:
            return

        ma = MarkerArray()

        if self.visual_model is not None and self.visual_data is not None:
            for geom_id, geom_obj in enumerate(self.visual_model.geometryObjects):
                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = self.node.get_clock().now().to_msg()
                marker.ns = geom_obj.name
                marker.id = geom_id
                marker.action = Marker.ADD

                M = self.visual_data.oMg[geom_id]
                marker.pose.position.x = float(M.translation[0])
                marker.pose.position.y = float(M.translation[1])
                marker.pose.position.z = float(M.translation[2])
                quat = pin.Quaternion(M.rotation)
                marker.pose.orientation.x = quat.x
                marker.pose.orientation.y = quat.y
                marker.pose.orientation.z = quat.z
                marker.pose.orientation.w = quat.w

                if hppfcl is not None and hasattr(geom_obj, "geometry"):
                    geom = geom_obj.geometry
                    if isinstance(geom, hppfcl.Box):
                        marker.type = Marker.CUBE
                        marker.scale.x = float(geom.halfSide[0] * 2.0)
                        marker.scale.y = float(geom.halfSide[1] * 2.0)
                        marker.scale.z = float(geom.halfSide[2] * 2.0)
                    elif isinstance(geom, hppfcl.Sphere):
                        marker.type = Marker.SPHERE
                        d = float(geom.radius * 2.0)
                        marker.scale.x = d
                        marker.scale.y = d
                        marker.scale.z = d
                    elif isinstance(geom, hppfcl.Cylinder):
                        marker.type = Marker.CYLINDER
                        marker.scale.x = float(geom.radius * 2.0)
                        marker.scale.y = float(geom.radius * 2.0)
                        marker.scale.z = float(geom.halfLength * 2.0)
                    elif isinstance(geom, hppfcl.Capsule):
                        marker.type = Marker.CYLINDER
                        marker.scale.x = float(geom.radius * 2.0)
                        marker.scale.y = float(geom.radius * 2.0)
                        marker.scale.z = float(geom.halfLength * 2.0)
                    elif getattr(geom_obj, "meshPath", ""):
                        marker.type = Marker.MESH_RESOURCE
                        marker.mesh_resource = self._format_mesh_resource(
                            geom_obj.meshPath
                        )
                        marker.scale.x = float(geom_obj.meshScale[0])
                        marker.scale.y = float(geom_obj.meshScale[1])
                        marker.scale.z = float(geom_obj.meshScale[2])
                    else:
                        marker.type = Marker.CUBE
                        marker.scale.x = 0.05
                        marker.scale.y = 0.05
                        marker.scale.z = 0.05
                elif getattr(geom_obj, "meshPath", ""):
                    marker.type = Marker.MESH_RESOURCE
                    marker.mesh_resource = self._format_mesh_resource(
                        geom_obj.meshPath
                    )
                    marker.scale.x = float(geom_obj.meshScale[0])
                    marker.scale.y = float(geom_obj.meshScale[1])
                    marker.scale.z = float(geom_obj.meshScale[2])
                else:
                    marker.type = Marker.CUBE
                    marker.scale.x = 0.05
                    marker.scale.y = 0.05
                    marker.scale.z = 0.05

                if getattr(geom_obj, "overrideMaterial", False):
                    color = geom_obj.meshColor
                    marker.color.r = float(color[0])
                    marker.color.g = float(color[1])
                    marker.color.b = float(color[2])
                    marker.color.a = float(color[3])
                else:
                    marker.mesh_use_embedded_materials = True
                    marker.color.r = 0.8
                    marker.color.g = 0.8
                    marker.color.b = 0.8
                    marker.color.a = 1.0

                ma.markers.append(marker)
        else:
            for i in range(1, self.model.njoints):
                frame_id = self.model.getFrameId(self.model.names[i])
                oMf = self.data.oMf[frame_id]

                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = self.node.get_clock().now().to_msg()
                marker.ns = self.model.names[i]
                marker.id = i
                marker.action = Marker.ADD

                marker.pose.position.x = oMf.translation[0]
                marker.pose.position.y = oMf.translation[1]
                marker.pose.position.z = oMf.translation[2]
                quat = pin.Quaternion(oMf.rotation)
                marker.pose.orientation.x = quat.x
                marker.pose.orientation.y = quat.y
                marker.pose.orientation.z = quat.z
                marker.pose.orientation.w = quat.w

                marker.type = Marker.CUBE
                marker.scale.x = 0.08
                marker.scale.y = 0.08
                marker.scale.z = 0.08
 
                marker.color.r = 0.2
                marker.color.g = 0.5
                marker.color.b = 0.9
                marker.color.a = 0.85
                ma.markers.append(marker)

        self.ModelMarker_pub.publish(ma)

    def _format_mesh_resource(self, mesh_path: str) -> str:
        """Return a RViz-compatible mesh resource URI."""
        if not mesh_path:
            return mesh_path
        if mesh_path.startswith("package://"):
            return mesh_path
        if mesh_path.startswith("file://"):
            return mesh_path
        if os.path.isabs(mesh_path):
            return "file://" + mesh_path
        return mesh_path


    # ====================== Path ======================

    def loadPath(self, path, name=None):
        if name is None:
            name = f"Path {self._path_player.counter}"
            self._path_player.counter += 1

        self._path_player.paths[name] = path
        self._path_player.current = path
        print(f"✅ Path '{name}' loaded ({path.length():.2f} s)")

        q, _ = path.eval(0.0)
        self.display(q)

    def playPath(self, speed=1.0, fps=60):
        if self._path_player.current is None:
            print("Aucun path chargé")
            return

        self._path_player.playing = True
        self._path_player.speed = speed
        self._path_player.fps = fps

        def animate():
            path = self._path_player.current
            t = 0.0
            dt = 1.0 / fps

            while self._path_player.playing and t <= path.length():
                start = time.time()
                q, success = path.eval(t)
                if success:
                    self.display(q)
                t += dt * speed
                time.sleep(max(0, dt - (time.time() - start)))

            self._path_player.playing = False
            print("Playback terminé")

        self._path_player.thread = threading.Thread(target=animate, daemon=True)
        self._path_player.thread.start()

    def displayPath(self, path: core.bindings.Path, dt=0.04, color=(0.0, 0.8, 1.0, 0.9), origin_frame="panda_link0"):
        """Affiche uniquement le chemin en ligne (end-effector)"""
        path_msg = Path()
        path_msg.header.frame_id = "world"
        path_msg.header.stamp = self.node.get_clock().now().to_msg()

        frame_names = {frame.name for frame in self.model.frames}
        if origin_frame not in frame_names:
            raise ValueError(
                f"Frame '{origin_frame}' not found in model. "
                f"Available frames: {sorted(frame_names)}"
            )

        frame_id = self.model.getFrameId(origin_frame)
        t = 0.0
        while t <= path.length() + 1e-6:
            q = np.asarray(path.eval(t)[0]).reshape(-1)
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            # Position de l'end-effector (change selon ton robot)
            pos = self.data.oMf[frame_id].translation

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(pos[0])
            pose.pose.position.y = float(pos[1])
            pose.pose.position.z = float(pos[2])
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            t += dt

        if self.PathMarker_pub is not None:
            self.PathMarker_pub.publish(path_msg)
        print(f"Path affiché ({len(path_msg.poses)} points)")

    def close(self):
        if self.executor:
            self.executor.shutdown()
        print("RVizVisualizer fermé")

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

   