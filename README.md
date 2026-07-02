# hpp-gepetto-viewer

[![Building Status](https://travis-ci.org/humanoid-path-planner/hpp-gepetto-viewer.svg?branch=master)](https://travis-ci.org/humanoid-path-planner/hpp-gepetto-viewer)
[![Pipeline status](https://gitlab.laas.fr/humanoid-path-planner/hpp-gepetto-viewer/badges/master/pipeline.svg)](https://gitlab.laas.fr/humanoid-path-planner/hpp-gepetto-viewer/commits/master)
[![Coverage report](https://gitlab.laas.fr/humanoid-path-planner/hpp-gepetto-viewer/badges/master/coverage.svg?job=doc-coverage)](https://gepettoweb.laas.fr/doc/humanoid-path-planner/hpp-gepetto-viewer/master/coverage/)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/humanoid-path-planner/hpp-gepetto-viewer/master.svg)](https://results.pre-commit.ci/latest/github/humanoid-path-planner/hpp-gepetto-viewer)

`hpp-gepetto-viewer` displays [HPP](https://github.com/humanoid-path-planner/hpp-doc) (Humanoid Path Planner) robots, obstacles and planned paths in a 3D viewer. It started as a thin Python layer that drives `hppcorbaserver` and `gepetto-viewer-server` simultaneously, and has since grown into a small family of visualization backends so that HPP scenes can be displayed with different viewers:

- **`pyhpp_viser`** — a [Pinocchio](https://github.com/stack-of-tasks/pinocchio) visualizer built on [`viser`](https://github.com/nerfstudio-project/viser), with a Gepetto-GUI-style scene hierarchy, that runs in a web browser without any CORBA/middleware dependency.
- **`pyhpp_rviz`** — a ROS 2 / RViz2-based visualization backend, made of a Python bridge (`pyhpp_rviz`) and native RViz2 plug-ins (panel, displays, tools) shipped in the `hpp_rviz` C++/ROS package.
- **`hpp.gepetto`** — the historical backend, controlling [`gepetto-viewer`](https://github.com/Gepetto/gepetto-viewer) through CORBA. **⚠️ Deprecated**: kept for backward compatibility only. New projects should use `pyhpp_viser` or `pyhpp_rviz` instead; see [Migrating away from the CORBA backend](#migrating-away-from-the-corba-backend).

## Table of contents

- [Package layout](#package-layout)
- [Dependencies](#dependencies)
- [Installation](#installation)
  - [From source with CMake](#from-source-with-cmake)
  - [With pip](#with-pip)
  - [With Nix](#with-nix)
- [Usage](#usage)
  - [Viser backend (`pyhpp_viser`)](#viser-backend-pyhpp_viser)
  - [RViz2 backend (`pyhpp_rviz`)](#rviz2-backend-pyhpp_rviz)
  - [Gepetto-viewer backend (`hpp.gepetto`) — deprecated](#gepetto-viewer-backend-hppgepetto--deprecated)
- [Migrating away from the CORBA backend](#migrating-away-from-the-corba-backend)
- [Documentation](#documentation)
- [Porting notes](#porting-notes)
- [License](#license)

## Package layout

```
hpp-gepetto-viewer/
├── CMakeLists.txt          # C++/CMake build (INTERFACE library + Python install)
├── package.xml             # ROS package manifest
├── pyproject.toml          # Python package metadata (pip-installable)
├── flake.nix               # Nix flake (build via github:gepetto/nix)
├── doc/                    # Doxygen pages (main page, RViz2 viewer, porting notes)
└── src/
    ├── pyhpp_viser/             # Viser-based Pinocchio visualizer (browser, no CORBA) — recommended
    │   └── viewer.py                # pyhpp_viser.Viewer(BaseVisualizer)
    ├── pyhpp_rviz/              # ROS 2 / RViz2 backend — recommended
    │   ├── viewer.py                # pyhpp_rviz.RVizVisualizer(BaseVisualizer)
    │   └── publisher/               # ROS 2 node helpers (TF, robot description, navigation, ...)
    ├── hpp/gepetto/             # gepetto-viewer / CORBA backend — DEPRECATED
    │   ├── viewer.py               # hpp.gepetto.Viewer
    │   ├── viewer_factory.py       # hpp.gepetto.ViewerFactory
    │   ├── path_player.py          # hpp.gepetto.PathPlayer
    │   ├── types.py                # Point3D / ColorRGBA type aliases
    │   ├── manipulation/           # hpp-manipulation-server flavored Viewer / ViewerFactory
    │   ├── gui/                    # path_graph GUI helper
    │   └── blender/                # exportmotion.py: export a path to Blender
    └── pyhpp_gepetto/           # Newer pyhpp-based Gepetto viewer client — DEPRECATED
        └── viewer.py
```

## Dependencies

Common to all backends:

- Python ≥ 3.9
- [`numpy`](https://numpy.org/)
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) (`pin`)

Backend-specific:

| Backend | Extra dependencies |
|---|---|
| `pyhpp_viser` (recommended) | [`viser`](https://github.com/nerfstudio-project/viser), `trimesh`, `pycollada`, `websockets`, `hppfcl` |
| `pyhpp_rviz` (recommended) | ROS 2 (`rclpy`), RViz2, the `hpp_rviz` C++ package (custom messages + plug-ins), `pyhpp`, `pyhpp_plot` |
| `hpp.gepetto` / `pyhpp_gepetto` ⚠️ deprecated | `hpp-corbaserver`, [`gepetto-viewer`](https://github.com/Gepetto/gepetto-viewer) (CORBA server), `omniORB` |

Build-time dependencies (C++/CMake path):

- CMake ≥ 3.22
- [`jrl-cmakemodules`](https://github.com/jrl-umi3218/jrl-cmakemodules) (fetched automatically via `FetchContent` if not already available as a git submodule or an installed package)
- `hpp-python`
- Optionally `hpp-corbaserver` and `gepetto-viewer` if the `USE_CORBA` CMake option is enabled (only needed for the deprecated CORBA backend)

## Installation

### From source with CMake

```bash
git clone --recursive https://github.com/humanoid-path-planner/hpp-gepetto-viewer.git
cd hpp-gepetto-viewer
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=<your_install_prefix>
make install
```

Relevant CMake options:

- `USE_HPP_PYTHON` (default `ON`): depend on `hpp-python`.
- `USE_CORBA` (default `OFF`): depend on `hpp-corbaserver` and `gepetto-viewer` to enable the **deprecated** CORBA-based Gepetto viewer backend. Leave this `OFF` unless you specifically need the legacy backend.

### With pip

The Python package can also be installed directly with pip, which pulls in the `pyhpp_viser` backend dependencies declared in `pyproject.toml`:

```bash
pip install .
```

This installs `numpy`, `pin`, `pycollada`, `trimesh`, `viser` and `websockets`. The `pyhpp_rviz` (ROS 2) backend and the deprecated `hpp.gepetto` (CORBA) backend have additional system-level dependencies (see the table above) that are not pulled in by pip and must be installed separately.

### With Nix

A flake is provided and builds against [`github:gepetto/nix`](https://github.com/gepetto/nix):

```bash
nix build github:humanoid-path-planner/hpp-gepetto-viewer
```

## Usage

### Viser backend (`pyhpp_viser`)

`pyhpp_viser.Viewer` is a `pinocchio.visualize.BaseVisualizer` subclass that serves an interactive 3D scene in the browser via [viser](https://github.com/nerfstudio-project/viser), organized with a Gepetto-GUI-style hierarchy (robots, visuals, collisions, frames, contact surfaces,landmarks). It requires no CORBA server and no ROS installation — only `viser`, `trimesh` and `hppfcl`. This is the recommended backend for local, dependency-light visualization.

```python
from pyhpp_viser import Viewer

v = Viewer(robot)
v.initViewer()
v.loadViewerModel()
v.display(q)
```

Install the optional dependencies with:

```bash
pip install --user viser trimesh
```

### RViz2 backend (`pyhpp_rviz`)

`pyhpp_rviz.RVizVisualizer` is a `pinocchio.visualize.BaseVisualizer` subclass that bridges HPP/Pinocchio with ROS 2, publishing robot state, path information and interactive landmarks, and reacting to commands from dedicated RViz2 plug-ins (a `TrajectorySlider` panel, a `TrajectoryDisplay`, a `LandmarkDisplay` and a `Landmark` interactive tool). This is the recommended backend for ROS 2-based setups.

```python
from pyhpp_rviz import RVizVisualizer

v = RVizVisualizer()
v.initViewer(robot)                                   # starts ROS 2 nodes, publishes URDF and static TF
v.display(q)                                          # broadcasts /tf for the current configuration
v.loadPath(path)                                      # publishes path metadata (for the TrajectorySlider)
v.displayPath(path, target_frame="world/ee_link")     # publishes nav_msgs/Path for 3D preview
v.addLandMark([x, y, z], [qx, qy, qz, qw], "my_landmark")
```

On `initViewer`, several ROS 2 nodes are started to publish joint states, path metadata, landmark requests, the robot description (URDF), static and dynamic TF transforms, and an optional `nav_msgs/Path` for 3D path preview. A full description of the ROS 2 nodes, topics, custom message types (`PathInfo`, `PinocchioJoint`, `HppVectorConfiguration`, `Landmark`) and RViz2 plug-ins is available in the [RViz2 viewer documentation page](doc/rviz-viewer.hh).

An optional constraint-graph viewer, served as a React web app, can be attached via `pyhpp_plot.GraphViewerThread`:

```python
v.setGraph(graph)        # PyWGraph
v.setProblem(problem)    # PyWProblem
v.launch_graph_viewer()
```

### Gepetto-viewer backend (`hpp.gepetto`) — deprecated

> **⚠️ Deprecated.** The CORBA-based Gepetto viewer backend (`hpp.gepetto` / `pyhpp_gepetto`) is kept only for backward compatibility with existing scripts. It requires `gepetto-viewer` and `hpp-corbaserver`, which are no longer actively developed. Use `pyhpp_viser` (browser, no middleware) or `pyhpp_rviz` (ROS 2) for new code — see [Migrating away from the CORBA backend](#migrating-away-from-the-corba-backend).

This was historically the way of visualizing an HPP robot. It keeps a `gepetto-viewer-server` client in sync with `hppcorbaserver`.

```python
from hpp.corbaserver import ProblemSolver
from hpp.gepetto import ViewerFactory

ps = ProblemSolver(robot)
vf = ViewerFactory(ps)
vf.loadObstacleModel("my_package", "obstacle_name", "obstacles")

v = vf.createViewer()
v(q)  # display configuration q
```

Key classes:

- **`hpp.gepetto.Viewer`** — simultaneous control of `hppcorbaserver` and `gepetto-viewer-server`; loads robot bodies and obstacles, displays configurations, computes trajectory colors, etc.
- **`hpp.gepetto.ViewerFactory`** — records viewer commands (`loadObstacleModel`, `buildRobotBodies`, ...) so that they can be replayed against a freshly (re)started `gepetto-viewer-server`, and creates `Viewer` instances on demand.
- **`hpp.gepetto.PathPlayer`** — samples a computed path at a fixed time step (`dt`) and configurable `speed`, and displays each sampled configuration.



## Migrating away from the CORBA backend

If you have existing code using `hpp.gepetto` / `pyhpp_gepetto`, consider switching to:

- **`pyhpp_viser`** if you want a lightweight, browser-based viewer with no external server process — closest to a drop-in replacement for local debugging and demos.
- **`pyhpp_rviz`** if you already work in a ROS 2 environment, or need RViz2-specific features.

Both replacement backends subclass `pinocchio.visualize.BaseVisualizer` and expose an `initViewer()` / `display(q)` pair analogous to the legacy `Viewer` API, so most call sites only need their imports and construction changed. 

## Documentation

- Doxygen-generated API documentation is published at <https://gepettoweb.laas.fr/doc/humanoid-path-planner/hpp-gepetto-viewer/master/doxygen-html/index.html>.
- The RViz2 backend has a dedicated documentation page: [`doc/rviz-viewer.md`](doc/rviz-viewer.hh) (overview, Python API, ROS 2 communication graph, topic reference, custom messages, and plug-in reference).
- Notable changes between releases are listed in [`NEWS`](NEWS).

## License

`hpp-gepetto-viewer` is released under the [BSD 2-Clause License](LICENSE), Copyright (c) 2014-2024, CNRS.