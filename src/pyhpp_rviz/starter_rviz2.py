import os

from ament_index_python import get_package_share_directory


def _launch_rviz2(RVIZ_CONFIG_PATH, NODES_YAML_PATH):

    os.system(
        f"ros2 launch hpp-gepetto-viewer viz.launch.py "
        f"rviz_config:={RVIZ_CONFIG_PATH} "
        f"nodes_yaml:={NODES_YAML_PATH} "
    )


def start_rviz2(
    RVIZ_CONFIG_PATH="/tmp/test_config.rviz", NODES_YAML_PATH="/tmp/nodes.yaml"
):
    from threading import Thread

    Thread(target=_launch_rviz2, args=(RVIZ_CONFIG_PATH, NODES_YAML_PATH)).start()
