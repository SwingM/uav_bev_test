from pathlib import Path


def test_sim_launch_uses_humble_safe_pose_bridge() -> None:
    """Guard against accidentally reintroducing unavailable Pose_V bridge mapping."""
    launch_file = Path(__file__).resolve().parents[1] / 'launch' / 'sim.launch.py'
    text = launch_file.read_text(encoding='utf-8')

    assert '/model/uav_platform/pose@geometry_msgs/msg/Pose@gz.msgs.Pose' in text
    assert '/world/mosaic_world/pose/info@gz.msgs.Pose_V@ros_gz_interfaces/msg/Pose_V' not in text
