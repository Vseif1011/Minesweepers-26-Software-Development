"""Competition geometry and evidence/freshness checks; no physical robot commands."""
import json
import math
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from std_msgs.msg import Bool

from minesweeper_laptop import mapping
from minesweeper_laptop.mapping import DetectionConfirmation, MineGrid
from minesweeper_laptop.odometry import quaternion


def test_all_400_competition_squares_use_row_major_coordinates():
    grid = MineGrid()
    for row in range(20):
        for col in range(20):
            assert grid.record(col+0.5, row+0.5, 0.0, 0.0, True)[:3] == (col, row, 1)
    assert len(grid.cells) == 400 and all(value == 1 for value in grid.cells)
    assert grid.tables()['surface'][-1] == {'column': 19, 'row': 19}


def test_edges_are_lower_inclusive_upper_exclusive_without_snapping():
    grid = MineGrid()
    assert grid.cell_at(0.0, 0.0) == (0, 0)
    assert grid.cell_at(math.nextafter(20.0, 0.0), math.nextafter(20.0, 0.0)) == (19, 19)
    assert grid.cell_at(math.nextafter(1.0, 0.0), 1.0) == (0, 1)
    assert grid.cell_at(1.0, 1.0) == (1, 1)
    for x, y in [(20., 0.), (0., 20.), (-1e-12, 0.), (0., -1e-12), (math.inf, 0.)]:
        assert grid.cell_at(x, y) is None


@pytest.mark.parametrize('heading,point', [
    (0.0, (10.6, 10.2)), (math.pi/2, (9.8, 10.6)),
    (math.pi, (9.4, 9.8)), (-math.pi/2, (10.2, 9.4)),
])
def test_forward_and_left_detector_offsets_rotate_with_robot(heading, point):
    grid = MineGrid()
    assert grid.record(10., 10., heading, 0.6, True, lateral=0.2)[3:] == pytest.approx(point)


@pytest.mark.parametrize('kwargs', [
    {'width': 0}, {'height': 1.5}, {'width': True}, {'resolution': math.nan},
    {'resolution': 0.0}, {'start_x': 20.0}, {'start_y': -0.1}, {'heading': math.inf},
])
def test_invalid_geometry_cannot_silently_make_bad_map(kwargs):
    with pytest.raises(ValueError):
        MineGrid(**kwargs)


def test_configurable_resolution_applies_to_both_transform_and_cell_bounds():
    grid = MineGrid(width=40, height=40, resolution=0.5, start_x=19.5, start_y=19.5,
                    heading=math.pi)
    x, y, theta = grid.transform(1.0, 0.0, 0.0)
    assert grid.record(x, y, theta, 0.55, False)[:3] == (35, 39, 2)


def test_confirmation_needs_distinct_samples_and_elapsed_time():
    grid = MineGrid()
    tracker = DetectionConfirmation(grid)
    assert tracker.observe(5.1, 6.2, True, 1.0) is None
    for _ in range(10):
        assert tracker.observe(5.1, 6.2, True, 1.0) is None
    assert tracker.observe(5.2, 6.3, True, 1.02) is None
    assert tracker.observe(5.3, 6.4, True, 1.04) is None
    record = tracker.observe(5.4, 6.5, True, 1.15)
    assert record[:3] == (5, 6, 1)
    assert record[3:] == pytest.approx((5.25, 6.35))
    assert tracker.observe(5.4, 6.5, True, 1.3) is None


def test_cell_boundary_jitter_cannot_confirm_two_neighboring_cells():
    grid = MineGrid()
    tracker = DetectionConfirmation(grid)
    for i in range(20):
        assert tracker.observe(0.99 if i % 2 else 1.01, 3.5, True, i*0.05) is None
    assert not any(grid.cells)
    for stamp in (2.0, 2.06, 2.12):
        result = tracker.observe(1.1, 3.5, True, stamp)
    assert result[:3] == (1, 3, 1)
    assert sum(value != 0 for value in grid.cells) == 1


def test_interrupted_invalid_and_reordered_samples_reset_confirmation():
    grid = MineGrid()
    tracker = DetectionConfirmation(grid)
    for x, stamp in [(3.5, 1.0), (3.5, 1.05), (3.5, 2.0), (3.5, 2.05),
                     (math.nan, 2.1), (3.5, 2.2), (3.5, 2.25), (3.5, 1.5)]:
        assert tracker.observe(x, 4.5, True, stamp) is None
    assert not any(grid.cells)
    tracker.reset()  # A negative metal sample or unavailable camera also breaks continuity.
    assert tracker.observe(3.5, 4.5, True, 3.0) is None


def test_classification_upgrade_requires_its_own_confirmation():
    grid = MineGrid()
    tracker = DetectionConfirmation(grid)
    for stamp in (1.0, 1.06, 1.12):
        buried = tracker.observe(19.5, 19.5, False, stamp)
    assert buried[2] == 2
    assert tracker.observe(19.5, 19.5, True, 1.2) is None
    assert grid.cells[-1] == 2
    assert tracker.observe(19.5, 19.5, True, 1.26) is None
    assert tracker.observe(19.5, 19.5, True, 1.32)[2] == 1
    for stamp in (1.4, 1.46, 1.52):
        assert tracker.observe(19.5, 19.5, False, stamp) is None
    assert grid.cells[-1] == 1


@pytest.fixture
def mapper(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_DOMAIN_ID', '89')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    original = mapping.param
    output = tmp_path/'mine_map.json'
    monkeypatch.setattr(mapping, 'param', lambda node, name, default:
                        str(output) if name == 'output_path' else original(node, name, default))
    clock = [100.0]
    monkeypatch.setattr(mapping, 'time', SimpleNamespace(monotonic=lambda: clock[0]))
    rclpy.init()
    instance = mapping.Mapping()
    yield instance, clock
    instance.destroy_node()
    rclpy.shutdown()


def pose(node, x=0.0, y=0.0, frame='odom', age=0.0):
    msg = PoseStamped()
    msg.header.frame_id = frame
    stamp = node.get_clock().now().nanoseconds-int(age*1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
    msg.pose.position.x, msg.pose.position.y = x, y
    msg.pose.orientation = quaternion(0.0)
    return msg


def feed(node, clock, *, visible=True, ready=True, positive=True, x=0.0):
    sample = pose(node, x=x)
    node.on_pose(sample)
    odometry = Odometry(header=sample.header)
    odometry.pose.pose = sample.pose
    node.on_odometry(odometry)
    node.on_camera(Bool(data=visible))
    node.on_camera_ready(Bool(data=ready))
    node.on_metal(Bool(data=positive))
    clock[0] += 0.06


def test_ros_map_and_autosave_keep_full_geometry_and_measured_position(mapper):
    node, clock = mapper
    maps, markers = [], []
    node.pub.publish = maps.append
    node.markers.publish = markers.append
    for _ in range(3):
        feed(node, clock, x=18.5)
    msg = maps[-1]
    assert (msg.info.width, msg.info.height, msg.info.resolution) == (20, 20, 1.0)
    assert len(msg.data) == 400 and msg.data[19] == 50
    saved = json.loads(node.output.read_text())
    assert saved['field_width_m'] == saved['field_height_m'] == 20.0
    assert saved['surface'] == [{'column': 19, 'row': 0}]
    assert saved['start_pose'] == {'x': 0.0, 'y': 0.0, 'heading_rad': 0.0}
    assert saved['detections'][0]['x_m'] == pytest.approx(19.05)
    assert markers[-1].markers[0].pose.position.x == pytest.approx(19.05)


def test_gui_timer_cannot_reuse_one_positive_sample_to_map_later_poses(mapper):
    node, clock = mapper
    feed(node, clock)
    for i in range(20):
        clock[0] += 0.06
        node.on_pose(pose(node, x=i*0.2))
        node.update()
    assert not any(node.grid.cells)
    assert not node.output.exists()


def test_missing_camera_is_unknown_and_does_not_mean_buried_mine(mapper):
    node, clock = mapper
    for _ in range(5):
        feed(node, clock, visible=False, ready=False)
    assert not any(node.grid.cells)
    for _ in range(3):
        feed(node, clock, visible=False)
    assert node.grid.tables()['buried'] == [{'column': 0, 'row': 0}]


@pytest.mark.parametrize('bad_pose', ['old', 'future', 'wrong_frame', 'zero_quaternion', 'nan', 'duplicate'])
def test_invalid_or_replayed_pose_does_not_refresh_mapping_state(mapper, bad_pose):
    node, clock = mapper
    first = pose(node)
    node.on_pose(first)
    clock[0] += 1.0
    msg = pose(node, age={'old': 2.0, 'future': -1.0}.get(bad_pose, 0.0))
    if bad_pose == 'wrong_frame':
        msg.header.frame_id = 'map'
    if bad_pose == 'zero_quaternion':
        msg.pose.orientation.w = 0.0
    if bad_pose == 'nan':
        msg.pose.orientation.x = math.nan
    if bad_pose == 'duplicate':
        msg = first
    node.on_pose(msg)
    node.on_camera(Bool(data=True))
    node.on_camera_ready(Bool(data=True))
    node.on_odometry(Odometry(header=pose(node).header))
    for _ in range(3):
        node.on_metal(Bool(data=True))
        clock[0] += 0.06
    assert not any(node.grid.cells)


def test_live_ekf_and_camera_cannot_mask_lost_encoder_odometry(mapper):
    node, clock = mapper
    feed(node, clock)
    clock[0] += 1.0
    for _ in range(4):
        node.on_pose(pose(node))
        node.on_camera(Bool(data=True))
        node.on_camera_ready(Bool(data=True))
        node.on_metal(Bool(data=True))
        clock[0] += 0.06
    assert not any(node.grid.cells)


def test_idle_shutdown_preserves_previous_export(mapper):
    node, _ = mapper
    node.output.write_text('previous competition result\n')
    # Fixture invokes destroy_node after this assertion; wrap it to inspect the result.
    destroy = node.destroy_node

    def destroy_and_check():
        result = destroy()
        assert node.output.read_text() == 'previous competition result\n'
        return result

    node.destroy_node = destroy_and_check
