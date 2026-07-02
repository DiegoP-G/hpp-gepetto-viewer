// Copyright (c) 2024, LAAS-CNRS
// Authors: See package.xml
//
// This file is part of hpp-gepetto-viewer.
// hpp-gepetto-viewer is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-gepetto-viewer is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-gepetto-viewer. If not, see <http://www.gnu.org/licenses/>.

/// \page hpp_gepetto_viewer_rviz_viewer RViz2 Viewer
///
/// \section hpp_gepetto_viewer_rviz_overview Overview
///
/// The RViz2 viewer backend replaces the legacy Gepetto viewer with a
/// ROS 2 / RViz2-based visualization stack. It is split into two layers:
///
/// \li A \b Python layer (\c pyhpp_rviz) that bridges HPP / Pinocchio with
///     ROS 2. It publishes robot state, path and landmark information and
///     reacts to commands coming from RViz2.
/// \li A \b C++ layer (\c hpp_rviz) that provides native RViz2 plug-ins
///     (panel, displays, tools) compiled against \c rviz_common.
///
/// \section hpp_gepetto_viewer_rviz_python Python layer – \c pyhpp_rviz
///
/// The entry point is \c pyhpp_rviz.viewer.RVizVisualizer, a
/// \c pinocchio.visualize.BaseVisualizer subclass. Typical usage:
///
/// \code{.py}
/// from pyhpp_rviz import RVizVisualizer
/// v = RVizVisualizer()
/// v.initViewer(robot)   # starts ROS 2 nodes and publishes URDF
/// v.display(q)          # sends TF transforms for configuration q
/// v.loadPath(path)      # publishes path metadata to TrajectorySlider
/// v.displayPath(path, target_frame="world/ee_link")  # publishes nav_msgs/Path for 3D preview
/// v.addLandMark([x, y, z], [qx, qy, qz, qw], "my_landmark")  # creates a landmark
/// \endcode
///
/// On \c initViewer the following ROS 2 nodes are created:
///
/// \li \c hpp_pinocchio_joint_publisher – publishes joint states
///     (\c /hpp/scene_objects) and subscribes to joint commands from the
///     TrajectorySlider panel (\c /hpp/pinocchio_joints).
/// \li \c hpp_pinocchio_path_publisher – publishes path metadata
///     (\c /hpp/pathInfo) and subscribes to time/frame commands from the
///     TrajectorySlider panel.
/// \li \c hpp_waypoint_publisher – publishes \c Landmark pose requests to
///     the Landmark tool (\c /hpp_landmark_server/landmark).
/// \li \c pinnochio_robot_description_publisher – publishes URDF on
///     \c /<prefix>/robot_description with \c TRANSIENT_LOCAL QoS (latched)
///     for each robot in the scene.
/// \li \c hpp_static_tf_publisher – broadcasts static TF transforms for
///     root joints (published once per unique frame pair).
/// \li \c pinnochio_navigation_publisher – publishes \c nav_msgs/Path
///     messages for path preview. The topic name is configurable
///     (default \c /hpp_path).
/// \li \c hpp_tf_broadcaster – streams per-frame TF transforms at display
///     time via a \c TransformBroadcaster on \c /tf.
///
/// \section hpp_gepetto_viewer_rviz_api Python API reference
///
/// \subsection hpp_gepetto_viewer_rviz_api_init initViewer / display
///
/// \code{.py}
/// v.initViewer(robot)   # initialise model, ROS nodes, publishes URDF and static TF
/// v.display(q)          # forward kinematics + broadcast /tf for every frame
/// v(q)                  # alias for display(q)
/// \endcode
///
/// \subsection hpp_gepetto_viewer_rviz_api_path Path display
///
/// \code{.py}
/// v.loadPath(path)
/// # Stores the path and publishes PathInfo (length + frame list) on /hpp/pathInfo.
/// # Required before the TrajectorySlider can scrub the trajectory.
/// # Does NOT publish the 3D geometry.
///
/// v.displayPath(path, dt=0.05, topic_name="hpp_path",
///               origin="world", target_frame=None)
/// # Samples the path at interval dt, computes FK at each sample, and
/// # publishes a nav_msgs/Path on <topic_name> for 3D preview in RViz2.
/// # target_frame must be a valid Pinocchio frame name; if None or "",
/// # nothing is published.
/// \endcode
///
/// \subsection hpp_gepetto_viewer_rviz_api_landmarks Landmarks
///
/// \code{.py}
/// v.addLandMark(xyz, quat_xyzw, name=None)
/// # Publishes a Landmark message on /hpp_landmark_server/landmark.
/// # xyz        : [x, y, z] position in the fixed frame ("world").
/// # quat_xyzw  : [qx, qy, qz, qw] orientation.
/// # name       : optional string identifier; defaults to "" if None.
///
/// v.addLandMarkFromFrame(target_frame, name)
/// # Looks up target_frame in the current Pinocchio model, reads its
/// # pose from the last display() call, and calls addLandMark().
/// \endcode
///
/// \subsection hpp_gepetto_viewer_rviz_api_graph Constraint graph viewer
///
/// An optional integration with \c pyhpp_plot.GraphViewerThread provides
/// an interactive constraint-graph viewer served as a React web app.
///
/// \code{.py}
/// v.setGraph(graph)       # set the constraint graph (PyWGraph)
/// v.setProblem(problem)   # set the planning problem (PyWProblem)
/// v.launch_graph_viewer() # start the GraphViewerThread (graph + problem required)
/// \endcode
///
/// When a configuration is generated from the graph viewer it is forwarded
/// to \c display() automatically.
///
/// \section hpp_gepetto_viewer_rviz_ros_graph ROS 2 communication graph
///
/// The diagram below shows every topic exchanged between the Python
/// visualizer, the helper publisher nodes, and the RViz2 plug-ins.
/// Arrows point from publisher to subscriber; message types are shown in
/// italics.
///
/// \dot
/// digraph ros_nodes {
///   graph [rankdir=LR, fontname="Helvetica", fontsize=10,
///          bgcolor="transparent", nodesep=0.4, ranksep=3.0,
///          compound=true, splines=polyline];
///   node  [fontname="Helvetica", fontsize=9, shape=box,
///          style="filled,rounded"];
///   edge  [fontname="Helvetica", fontsize=8];
///
///   { rank=same;
///     joints_node; path_node; landmark_node;
///     tf_pub; rd_pub; static_tf; nav_pub; }
///   { rank=same;
///     panel; tdisp; ldisp; ltool; rviz_tf; rviz_rd; }
///
///   // ── Python nodes ──────────────────────────────────────────────────
///   joints_node   [label="hpp_pinocchio_joint_publisher",      fillcolor="#dbe9f4", color="#4a90d9"];
///   path_node     [label="hpp_pinocchio_path_publisher",       fillcolor="#dbe9f4", color="#4a90d9"];
///   landmark_node [label="hpp_waypoint_publisher",             fillcolor="#dbe9f4", color="#4a90d9"];
///   tf_pub        [label="hpp_tf_broadcaster",                 fillcolor="#dbe9f4", color="#4a90d9"];
///   rd_pub        [label="pinnochio_robot_description_publisher", fillcolor="#dbe9f4", color="#4a90d9"];
///   static_tf     [label="hpp_static_tf_publisher",            fillcolor="#dbe9f4", color="#4a90d9"];
///   nav_pub       [label="pinnochio_navigation_publisher",     fillcolor="#dbe9f4", color="#4a90d9"];
///
///   // ── RViz2 plug-ins ────────────────────────────────────────────────
///   panel [label="TrajectorySlider (Panel)",    fillcolor="#fde8d8", color="#e07b39"];
///   tdisp [label="TrajectoryDisplay (Display)", fillcolor="#fde8d8", color="#e07b39"];
///   ldisp [label="LandmarkDisplay (Display)",   fillcolor="#fde8d8", color="#e07b39"];
///   ltool [label="Landmark (Tool)",             fillcolor="#fde8d8", color="#e07b39"];
///
///   // ── RViz2 infra ───────────────────────────────────────────────────
///   rviz_tf [label="TF tree",            shape=ellipse, fillcolor="#e8f4e8", color="#4a9d4a"];
///   rviz_rd [label="RobotModel display", shape=ellipse, fillcolor="#e8f4e8", color="#4a9d4a"];
///
///   // ── Edges Python → RViz2 ─────────────────────────────────────────
///   tf_pub        -> rviz_tf [label="/tf"];
///   static_tf     -> rviz_tf [label="/tf_static"];
///   rd_pub        -> rviz_rd [label="/<prefix>/robot_description"];
///   path_node     -> panel   [label="/hpp/pathInfo (PathInfo)"];
///   nav_pub       -> tdisp   [label="/hpp_path (nav_msgs/Path)"];
///   joints_node   -> panel   [label="/hpp/scene_objects\n(HppVectorConfiguration)"];
///   landmark_node -> ltool   [label="/hpp_landmark_server/landmark\n(Landmark)"];
///
///   // ── Edges RViz2 → Python (panel commands, red) ───────────────────
///   panel -> path_node   [label="/hpp/trajectory_time (PathInfo)",       color="#c0392b", fontcolor="#c0392b"];
///   panel -> path_node   [label="/hpp/target_frame (PathInfo)",           color="#c0392b", fontcolor="#c0392b"];
///   panel -> joints_node [label="/hpp/pinocchio_joints (PinocchioJoint)\n[manual joint edit only]", color="#c0392b", fontcolor="#c0392b"];
///
///   // ── Edges within RViz2 (intra, no constraint) ────────────────────
///   ltool -> ldisp [xlabel="/hpp_landmark_server/update\n(InteractiveMarkerUpdate)",
///                   color="#888888", fontcolor="#888888", constraint=false];
///   ldisp -> ltool [xlabel="/hpp_landmark_server/landmark_visibility\n(Landmark)",
///                   color="#888888", fontcolor="#888888", style=dashed, constraint=false];
///
///   // ── Legend ───────────────────────────────────────────────────────
///   subgraph cluster_legend {
///    label = "Legende" ;
///    shape = rectangle ;
///    color = black ;
///
///    py1 -> py2 [style=invis] ;
///    rviz1 -> rviz2 [style=invis] ;
///    infra1 -> infra2 [style=invis] ;
///    e1 -> e2 [label="pub to sub", color="#888888", fontsize=10] ;
///    e3 -> e4 [label="panel cmd", color="#c0392b", fontsize=10] ;
///    e5 -> e6 [label="optional (intra RViz2)", color="#888888", style=dashed, fontsize=10] ;
///
///    py1   [label="Python node",   style=filled, fillcolor="#dbe9f4", color="#4a90d9"] ;
///    py2   [style=invis] ;
///    rviz1 [label="RViz2 plug-in",  style=filled, fillcolor="#fde8d8", color="#e07b39"] ;
///    rviz2 [style=invis] ;
///    infra1 [label="RViz2 infra",  shape=ellipse, style=filled, fillcolor="#e8f4e8", color="#4a9d4a"] ;
///    infra2 [style=invis] ;
///    e1 [style=invis] ;
///    e2 [style=invis] ;
///    e3 [style=invis] ;
///    e4 [style=invis] ;
///    e5 [style=invis] ;
///    e6 [style=invis] ;
///}
///}
/// \enddot
///
/// \section hpp_gepetto_viewer_rviz_topics Topic reference
///
/// | Topic | Direction | Message type | Description |
/// |-------|-----------|--------------|-------------|
/// | \c /hpp/pathInfo | Python → Panel | \c PathInfo | Announces a new path (total length in seconds, available frame names). |
/// | \c /hpp/trajectory_time | Panel → Python | \c PathInfo | Current slider time (field \c current_time, in seconds) sent during playback. |
/// | \c /hpp/target_frame | Panel → Python | \c PathInfo | Selected target TF frame (field \c target_frame) for path 3D preview. |
/// | \c /hpp/pinocchio_joints | Panel → Python | \c PinocchioJoint | Published when the user manually edits a joint slider in the panel (not during trajectory playback). |
/// | \c /hpp/scene_objects | Python → Panel | \c HppVectorConfiguration | Full configuration vector with per-joint details, used to populate the joint tree and the configuration text field. |
/// | \c /hpp_landmark_server/landmark | Python → Tool | \c Landmark | Requests creation of a new interactive landmark at the given pose. |
/// | \c /hpp_landmark_server/landmark_visibility | Display → Tool | \c Landmark | Published by LandmarkDisplay when a landmark's visibility toggle is changed; the Landmark tool applies the change on its InteractiveMarkerServer. |
/// | \c /hpp_landmark_server/update | Tool → Display | \c visualization_msgs/InteractiveMarkerUpdate | Published by the InteractiveMarkerServer (hosted in the Landmark tool) when markers are inserted or updated; consumed by LandmarkDisplay to populate the property panel. |
/// | \c /tf | Python → RViz2 | \c tf2_msgs/TFMessage | Per-frame dynamic TF transforms, broadcast on every \c display() call. |
/// | \c /tf_static | Python → RViz2 | \c tf2_msgs/TFMessage | Root-joint static transforms, broadcast once on \c initViewer() (deduplicated per frame pair). |
/// | \c /<prefix>/robot_description | Python → RViz2 | \c std_msgs/String | URDF string published with \c TRANSIENT_LOCAL QoS, consumed by the RViz2 RobotModel display. |
/// | \c /hpp_path | Python → TrajectoryDisplay | \c nav_msgs/Path | Path geometry for 3D preview in RViz2, published by \c displayPath(). |
///
/// \section hpp_gepetto_viewer_rviz_messages Custom message types
///
/// All custom messages are defined in \c msg/ and belong to
/// the \c hpp_rviz ROS 2 package.
///
/// \subsection hpp_gepetto_viewer_rviz_msg_pathinfo PathInfo
///
/// Carries timing and frame information for trajectory playback.
///
/// \code
/// std_msgs/Header header
/// float64 path_length      # total duration of the path (seconds)
/// float64 current_time     # current playback position (seconds)
/// string[] frame_names     # available Pinocchio frame names for target selection
/// string target_frame      # currently selected TF target frame
/// \endcode
///
/// \subsection hpp_gepetto_viewer_rviz_msg_pinocchiojoint PinocchioJoint
///
/// Carries the current value of one Pinocchio joint.
///
/// \code
/// std_msgs/Header header
/// string name              # joint name as in the Pinocchio model
/// string type              # joint type: "JOINT" (nq=1) or "FREE_FLYER" (nq=7)
/// float64[] values         # joint coordinates (1 value for JOINT, 7 for FREE_FLYER)
/// float64 min              # lower position bound (single-dof joints)
/// float64 max              # upper position bound (single-dof joints)
/// \endcode
///
/// \subsection hpp_gepetto_viewer_rviz_msg_hppvec HppVectorConfiguration
///
/// A full robot configuration packaged for scene-object display in the panel.
///
/// \code
/// float64[] hpp_vector     # flat configuration vector (HPP ordering)
/// PinocchioJoint[] joints  # per-joint details for tree population
/// \endcode
///
/// \subsection hpp_gepetto_viewer_rviz_msg_landmark Landmark
///
/// Pose and visibility information for a named landmark. Used both to
/// request landmark creation from Python (\c /hpp_landmark_server/landmark)
/// and to toggle visibility from the display
/// (\c /hpp_landmark_server/landmark_visibility).
///
/// \code
/// std_msgs/Header header
/// bool enable              # true = show, false = hide
/// string name              # landmark identifier
/// float64 tx               # translation x (metres)
/// float64 ty               # translation y (metres)
/// float64 tz               # translation z (metres)
/// float64 ow               # orientation quaternion w
/// float64 ox               # orientation quaternion x
/// float64 oy               # orientation quaternion y
/// float64 oz               # orientation quaternion z
/// \endcode
///
/// \section hpp_gepetto_viewer_rviz_plugins RViz2 plug-ins
///
/// \subsection hpp_gepetto_viewer_rviz_panel TrajectorySlider panel
///
/// Registered as \c hpp/TrajectorySlider.  The panel provides:
/// \li A time slider and spin-box to scrub through the current path.
/// \li Play / pause button with configurable speed multiplier.
/// \li A TF target-frame selector (combo-box populated from \c PathInfo.frame_names).
/// \li A joint-value tree that reflects the current configuration and allows
///     interactive editing of individual joints (JOINT and FREE_FLYER types).
///     Free-flyer quaternion components are automatically renormalised on edit.
/// \li A read-only text field showing the current HPP configuration vector,
///     with a one-click copy button.
///
/// \subsection hpp_gepetto_viewer_rviz_display_traj TrajectoryDisplay
///
/// Registered as \c hpp/TrajectoryDisplay. Inherits
/// \c rviz_default_plugins::displays::PathDisplay and automatically creates
/// a \c TrajectorySlider panel (pane title: \em "HPP Trajectory Control") on
/// initialization. Its path topic is fixed to \c /hpp_path.
///
/// \subsection hpp_gepetto_viewer_rviz_display_landmark LandmarkDisplay
///
/// Registered as \c hpp/LandmarkDisplay. Inherits
/// \c InteractiveMarkerDisplay and connects to the \c /hpp_landmark_server
/// interactive marker server. Each landmark discovered via
/// \c /hpp_landmark_server/update gets a \c LandmarkProperty entry in the
/// RViz2 property panel, exposing (read-only) position, orientation and a
/// visibility toggle. Toggling visibility publishes a \c Landmark message on
/// \c /hpp_landmark_server/landmark_visibility; the Landmark tool subscribes
/// to that topic and resizes the interactive marker accordingly (scale 0.001
/// to hide, nominal scale to show).
///
/// \subsection hpp_gepetto_viewer_rviz_tool Landmark tool
///
/// Registered as \c hpp/Landmark. A left-click in the 3D viewport
/// ray-casts against the scene and creates a new \c InteractiveLandmark
/// at the picked position with a default identity orientation. The
/// interactive marker supports 6-DOF manipulation (three move axes and
/// three rotate axes, all in \c FIXED orientation mode). The tool also
/// listens on \c /hpp_landmark_server/landmark so that landmarks can be
/// created programmatically from Python via \c addLandMark() or
/// \c addLandMarkFromFrame(). A right-click context menu on each marker
/// offers \em Edit (opens a pose dialog with spin-boxes) and \em Delete.
