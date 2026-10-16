"""Semantic mine grid, RViz markers, persistent tables, and optional Pygame GUI."""
import json
import math
import time
from pathlib import Path

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .odometry import normalize, quaternion, yaw
from .runtime import param, run


class MineGrid:
    def __init__(self, width=20, height=20, resolution=1.0, start_x=0.0, start_y=0.0, heading=0.0):
        if any(isinstance(v, bool) or not isinstance(v, int) or v <= 0 for v in (width, height)):
            raise ValueError('grid width and height must be positive integer cell counts')
        if not all(math.isfinite(v) for v in (resolution, start_x, start_y, heading)) or resolution <= 0:
            raise ValueError('resolution must be positive; geometry must be finite')
        if not (0 <= start_x < width*resolution and 0 <= start_y < height*resolution):
            raise ValueError('starting robot position must be inside the field')
        self.width, self.height, self.resolution = width, height, resolution
        self.start_x, self.start_y, self.heading = start_x, start_y, heading
        self.cells = [0] * (width*height)
        self.positions = {}

    def transform(self, x, y, theta):
        c, s = math.cos(self.heading), math.sin(self.heading)
        return self.start_x+c*x-s*y, self.start_y+s*x+c*y, normalize(theta+self.heading)

    def cell_at(self, x, y):
        if not all(math.isfinite(v) for v in (x, y)):
            return None
        col, row = math.floor(x/self.resolution), math.floor(y/self.resolution)
        if not (0 <= col < self.width and 0 <= row < self.height):
            return None
        return col, row

    @staticmethod
    def detector_position(x, y, theta, offset, lateral=0.0):
        # ROS body axes: +X forward, +Y left. Rotate both lever arms into map.
        c, s = math.cos(theta), math.sin(theta)
        return x+offset*c-lateral*s, y+offset*s+lateral*c

    def record(self, x, y, theta, offset, surface, lateral=0.0):
        if not all(math.isfinite(v) for v in (x, y, theta, offset, lateral)):
            return None
        return self.record_point(*self.detector_position(x, y, theta, offset, lateral), surface)

    def record_point(self, mx, my, surface):
        cell = self.cell_at(mx, my)
        if cell is None:
            return None
        col, row = cell
        index, kind = row*self.width+col, 1 if surface else 2
        # Vision can upgrade an earlier buried classification; duplicates are suppressed.
        if self.cells[index] == kind or self.cells[index] == 1:
            return None
        self.cells[index] = kind
        self.positions[index] = (mx, my)
        return col, row, kind, mx, my

    def tables(self):
        return {name: [{'column': i % self.width, 'row': i // self.width}
                       for i, value in enumerate(self.cells) if value == kind]
                for name, kind in [('surface', 1), ('buried', 2)]}

    def snapshot(self, frame, offset, lateral):
        return {
            'schema_version': 2, 'width': self.width, 'height': self.height,
            'resolution': self.resolution, 'frame_id': frame,
            'field_width_m': self.width*self.resolution,
            'field_height_m': self.height*self.resolution,
            'origin': {'x': 0.0, 'y': 0.0},
            'coordinates': 'zero-based column=floor(x/resolution), row=floor(y/resolution); +Y up',
            'cell_values': {'unmarked': 0, 'surface': 1, 'buried': 2},
            'start_pose': {'x': self.start_x, 'y': self.start_y, 'heading_rad': self.heading},
            'detector_offset': {'forward_m': offset, 'left_m': lateral},
            'cells': list(self.cells), **self.tables(),
            'detections': [
                {'column': i % self.width, 'row': i // self.width,
                 'x_m': point[0], 'y_m': point[1],
                 'classification': 'surface' if self.cells[i] == 1 else 'buried'}
                for i, point in sorted(self.positions.items())],
        }


class DetectionConfirmation:
    """Confirm consecutive metal samples in one cell; timers cannot add evidence."""

    def __init__(self, grid, samples=3, duration=0.1, max_gap=0.75):
        if isinstance(samples, bool) or not isinstance(samples, int) or samples < 1:
            raise ValueError('confirmation_samples must be a positive integer')
        if not all(math.isfinite(v) for v in (duration, max_gap)) or duration < 0 or max_gap <= 0:
            raise ValueError('confirmation duration must be nonnegative and timeout positive and finite')
        if duration >= max_gap:
            raise ValueError('confirmation duration must be shorter than sensor_timeout')
        self.grid, self.samples, self.duration, self.max_gap = grid, samples, duration, max_gap
        self.reset()

    def reset(self):
        self.key = None
        self.count = 0
        self.first = self.last = float('-inf')
        self.sum_x = self.sum_y = 0.0

    def observe(self, x, y, surface, stamp):
        cell = self.grid.cell_at(x, y)
        if cell is None or not math.isfinite(stamp):
            self.reset()
            return None
        key = (*cell, bool(surface))
        if key != self.key or stamp-self.last >= self.max_gap or stamp < self.last:
            self.reset()
            self.key, self.first = key, stamp
        elif stamp == self.last:
            return None
        self.last = stamp
        self.count += 1
        self.sum_x += x
        self.sum_y += y
        if self.count < self.samples or stamp-self.first < self.duration:
            return None
        return self.grid.record_point(self.sum_x/self.count, self.sum_y/self.count, surface)


class Mapping(Node):
    def __init__(self):
        super().__init__('mapping')
        self.grid = MineGrid(param(self, 'width', 20), param(self, 'height', 20),
                             param(self, 'resolution', 1.0), param(self, 'start_x', 0.0),
                             param(self, 'start_y', 0.0), param(self, 'start_heading', 0.0))
        self.offset = param(self, 'detector_offset', 0.55)
        self.lateral_offset = param(self, 'detector_lateral_offset', 0.0)
        self.timeout = param(self, 'sensor_timeout', 0.75)
        if not all(math.isfinite(v) for v in (self.offset, self.lateral_offset)):
            raise ValueError('detector offsets must be finite')
        self.confirmation = DetectionConfirmation(
            self.grid, param(self, 'confirmation_samples', 3),
            param(self, 'confirmation_duration', 0.1), self.timeout)
        self.frame = param(self, 'frame_id', 'map')
        self.odom_frame = param(self, 'odom_frame', 'odom')
        self.tf = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = self.grid.start_x
        transform.transform.translation.y = self.grid.start_y
        transform.transform.rotation = quaternion(self.grid.heading)
        self.tf.sendTransform(transform)
        self.output = Path(param(self, 'output_path', '~/.ros/minesweepers/mine_map.json')).expanduser()
        self.show_gui = param(self, 'show_gui', False)
        self.pose = None
        self.camera = self.camera_ready = False
        self.pose_time = self.odom_time = self.camera_time = self.camera_ready_time = float('-inf')
        self.pose_stamp = self.odom_stamp = None
        self.map_changed = False
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(OccupancyGrid, 'mine_map', durable)
        self.markers = self.create_publisher(MarkerArray, 'mine_markers', durable)
        self.mine_pub = self.create_publisher(PoseStamped, 'mine_pose', 10)
        self.create_subscription(PoseStamped, 'pose_combined', self.on_pose, 10)
        self.create_subscription(Odometry, 'odom', self.on_odometry, 10)
        self.create_subscription(Bool, 'detection', self.on_metal, 10)
        self.create_subscription(Bool, 'camera_detection', self.on_camera, 10)
        self.create_subscription(Bool, 'camera/ready', self.on_camera_ready, 10)
        self.create_service(Trigger, 'map/save', self.save)
        self.screen = None
        if self.show_gui:
            import pygame
            self.pygame = pygame
            pygame.init()
            display = pygame.display.Info()
            self.map_left, self.map_top, self.sidebar = 42, 50, 320
            self.tile = max(8, min(48, (display.current_w-100-self.sidebar-self.map_left)//self.grid.width,
                                   (display.current_h-150-self.map_top-35)//self.grid.height))
            self.map_pixels = self.grid.width*self.tile
            self.screen = pygame.display.set_mode((self.map_left+self.map_pixels+self.sidebar,
                                                   self.map_top+self.grid.height*self.tile+35))
            pygame.display.set_caption('Minesweepers A — ROS 2')
            self.font = pygame.font.Font(None, 23)
            self.small_font = pygame.font.Font(None, 16)
            self.scroll = 0
        self.create_timer(0.1, self.update)
        self.create_timer(1.0, self.publish_map)
        self.get_logger().info(
            f'Mapping ready: {self.grid.width} x {self.grid.height} cells, '
            f'{self.grid.width*self.grid.resolution:g} x {self.grid.height*self.grid.resolution:g} m; '
            'publishing mine_map and mine_markers. '
            'Waiting for fresh odom, pose_combined, detection, camera_detection, and camera/ready.')
        self.get_logger().info(
            f'Start pose: ({self.grid.start_x:g}, {self.grid.start_y:g}) m, '
            f'heading {self.grid.heading:g} rad. Confirm this and calibrate encoders/offsets before field use.')
        if self.show_gui:
            self.get_logger().info('Map window enabled. Press S to save the map and a screenshot.')
        else:
            self.get_logger().info(
                'Map window disabled. To open it, restart with: '
                'ros2 run minesweeper_laptop mapping --ros-args -p show_gui:=true')

    def on_pose(self, msg):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        q = msg.pose.orientation
        values = (msg.pose.position.x, msg.pose.position.y, q.x, q.y, q.z, q.w)
        valid = (msg.header.frame_id == self.odom_frame and -0.1 <= age < self.timeout
                 and all(math.isfinite(v) for v in values)
                 and abs(sum(v*v for v in (q.x, q.y, q.z, q.w))-1.0) < 0.01)
        if valid and (self.pose_stamp is None or stamp > self.pose_stamp):
            x, y, theta = values[0], values[1], yaw(q)
            self.pose = self.grid.transform(x, y, theta)
            self.pose_time = time.monotonic()-max(0.0, age)
            self.pose_stamp = stamp

    def on_odometry(self, msg):
        # The EKF republishes its held pose even if encoders disappear. Require
        # fresh encoder-backed odometry too, so that held pose is not new evidence.
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if (msg.header.frame_id == self.odom_frame and -0.1 <= age < self.timeout
                and (self.odom_stamp is None or stamp > self.odom_stamp)
                and all(math.isfinite(v) for v in (msg.pose.pose.position.x, msg.pose.pose.position.y))):
            self.odom_time = time.monotonic()-max(0.0, age)
            self.odom_stamp = stamp

    def on_metal(self, msg):
        now = time.monotonic()
        fresh = all(now-t < self.timeout for t in
                    (self.pose_time, self.odom_time, self.camera_time, self.camera_ready_time))
        if not msg.data or self.pose is None or not fresh or not self.camera_ready:
            self.confirmation.reset()
            return
        # Snapshot the pose when this distinct metal sample arrives, not on a later GUI timer.
        mx, my = self.grid.detector_position(*self.pose, self.offset, self.lateral_offset)
        record = self.confirmation.observe(mx, my, self.camera, now)
        if record:
            col, row, kind, mx, my = record
            mine = PoseStamped()
            mine.header.stamp, mine.header.frame_id = self.get_clock().now().to_msg(), self.frame
            mine.pose.position.x, mine.pose.position.y = mx, my
            mine.pose.orientation = quaternion(self.pose[2])
            self.mine_pub.publish(mine)
            self.get_logger().info(f'Confirmed mine at cell ({col}, {row}), class={kind}')
            self.map_changed = True
            self.publish_map()
            response = self.save(None, Trigger.Response())
            if not response.success:
                self.get_logger().error(f'Map save failed: {response.message}')

    def on_camera(self, msg):
        self.camera, self.camera_time = msg.data, time.monotonic()

    def on_camera_ready(self, msg):
        self.camera_ready, self.camera_ready_time = msg.data, time.monotonic()
        if not msg.data:
            self.confirmation.reset()

    def update(self):
        if self.screen:
            self.draw()

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), self.frame
        msg.info.width, msg.info.height, msg.info.resolution = self.grid.width, self.grid.height, self.grid.resolution
        msg.info.origin.orientation.w = 1.0
        msg.data = [0 if v == 0 else 50 if v == 1 else 100 for v in self.grid.cells]
        self.pub.publish(msg)
        markers = MarkerArray()
        for i, kind in enumerate(self.grid.cells):
            if kind == 0:
                continue
            marker = Marker()
            marker.header, marker.ns, marker.id = msg.header, 'mines', i
            marker.type, marker.action = Marker.CYLINDER, Marker.ADD
            marker.pose.position.x, marker.pose.position.y = self.grid.positions[i]
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = self.grid.resolution*0.5
            marker.scale.z = 0.1
            marker.color.g, marker.color.b, marker.color.a = (1.0 if kind == 1 else 0.5), 1.0, 1.0
            marker.text = 'surface' if kind == 1 else 'buried'
            markers.markers.append(marker)
        self.markers.publish(markers)

    def save(self, request, response):
        try:
            self.output.parent.mkdir(parents=True, exist_ok=True)
            data = self.grid.snapshot(self.frame, self.offset, self.lateral_offset)
            data['confirmation'] = {'samples': self.confirmation.samples, 'duration_s': self.confirmation.duration}
            temporary = self.output.with_suffix('.tmp')
            temporary.write_text(json.dumps(data, indent=2)+'\n')
            temporary.replace(self.output)
            response.success, response.message = True, str(self.output)
        except OSError as exc:
            response.success, response.message = False, str(exc)
        return response

    def draw(self):
        pg = self.pygame
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.display.quit()
                self.screen = None
                return
            if event.type == pg.KEYDOWN and event.key == pg.K_s:
                self.save(None, Trigger.Response())
                self.output.parent.mkdir(parents=True, exist_ok=True)
                pg.image.save(self.screen, str(self.output.with_suffix('.png')))
            if event.type == pg.MOUSEWHEEL:
                self.scroll = max(0, self.scroll-event.y*3)
        self.screen.fill((245, 245, 245))
        title = (f'{self.grid.width*self.grid.resolution:g} m x {self.grid.height*self.grid.resolution:g} m'
                 f' | {self.grid.resolution:g} m cells | origin lower left')
        self.screen.blit(self.font.render(title, True, (0, 0, 0)), (self.map_left, 12))
        colors = [(255, 255, 255), (0, 255, 255), (0, 128, 128)]
        for i, kind in enumerate(self.grid.cells):
            rect = pg.Rect(self.map_left+(i % self.grid.width)*self.tile,
                           self.map_top+(self.grid.height-1-i//self.grid.width)*self.tile, self.tile, self.tile)
            pg.draw.rect(self.screen, colors[kind], rect)
            pg.draw.rect(self.screen, (80, 80, 80), rect, 1)
        for col in range(self.grid.width):
            self.screen.blit(self.small_font.render(str(col), True, (0, 0, 0)),
                             (self.map_left+(col+0.35)*self.tile, self.map_top+self.grid.height*self.tile+6))
        for row in range(self.grid.height):
            self.screen.blit(self.small_font.render(str(row), True, (0, 0, 0)),
                             (12, self.map_top+(self.grid.height-row-0.6)*self.tile))
        field_rect = pg.Rect(self.map_left, self.map_top, self.map_pixels, self.grid.height*self.tile)
        self.screen.set_clip(field_rect)

        def pixels(x, y):
            return (int(self.map_left+x/self.grid.resolution*self.tile),
                    int(self.map_top+(self.grid.height-y/self.grid.resolution)*self.tile))

        for point in self.grid.positions.values():
            pg.draw.circle(self.screen, (0, 0, 0), pixels(*point), 3)
        if self.pose:
            x, y, theta = self.pose
            px, py = pixels(x, y)
            pg.draw.circle(self.screen, (220, 65, 65), (px, py), min(12, self.tile//3))
            pg.draw.line(self.screen, (0, 0, 0), (px, py), (px+25*math.cos(theta), py-25*math.sin(theta)), 3)
            detector = self.grid.detector_position(x, y, theta, self.offset, self.lateral_offset)
            pg.draw.circle(self.screen, (230, 160, 0), pixels(*detector), 4)
        self.screen.set_clip(None)
        side_x = self.map_left+self.map_pixels+15
        lines = ['Cells: (column, row), 0-based', 'White = unmarked, not cleared',
                 'Red = robot; gold = detector', 'S: save | Wheel: scroll mines']
        if self.pose:
            lines.append(f'Robot: {self.pose[0]:.2f}, {self.pose[1]:.2f} m')
        lines += ['']
        for kind, entries in self.grid.tables().items():
            lines += [f'{kind.upper()} ({len(entries)})']
            coordinates = [f"({e['column']},{e['row']})" for e in entries]
            lines += ['  '.join(coordinates[i:i+3]) for i in range(0, len(coordinates), 3)] + ['']
        visible_lines = (self.screen.get_height()-60)//22
        self.scroll = min(self.scroll, max(0, len(lines)-visible_lines))
        self.screen.set_clip(pg.Rect(side_x, 50, self.sidebar-20, self.screen.get_height()-60))
        for i, text in enumerate(lines[self.scroll:self.scroll+visible_lines]):
            self.screen.blit(self.font.render(text, True, (0, 0, 0)), (side_x, 50+i*22))
        self.screen.set_clip(None)
        pg.display.flip()

    def destroy_node(self):
        # An idle/failed run must not replace a previous competition result with an empty grid.
        if self.map_changed:
            response = self.save(None, Trigger.Response())
            if not response.success:
                self.get_logger().error(f'Map save failed: {response.message}')
        if self.show_gui:
            self.pygame.quit()
        return super().destroy_node()


def main(args=None):
    run(Mapping, args)
