import math
import os

import numpy as np
import yaml

# the map is in the odom frame, so (0, 0) is where odometry started
DEFAULT_MAP_FILE = os.path.expanduser('~/.ros/room_map.yaml')

PIXEL_FREE = 254
PIXEL_OCCUPIED = 0
PIXEL_UNKNOWN = 205


class RoomMap:
    HIT_UPDATE = 0.85
    MISS_UPDATE = -0.4
    LOG_ODDS_LIMIT = 5.0
    OCCUPIED_ABOVE = 0.4
    FREE_BELOW = -0.2

    def __init__(self, size_m=20.0, resolution=0.05,
                 origin_x=None, origin_y=None):
        self.resolution = resolution
        self.size = int(round(size_m / resolution))
        self.origin_x = -size_m / 2.0 if origin_x is None else origin_x
        self.origin_y = -size_m / 2.0 if origin_y is None else origin_y
        self.log_odds = np.zeros((self.size, self.size), dtype=np.float32)

    def world_to_cell(self, x, y):
        col = np.floor((x - self.origin_x) / self.resolution).astype(int)
        row = np.floor((y - self.origin_y) / self.resolution).astype(int)
        return col, row

    def add_scan(self, x, y, yaw, ranges, angle_increment,
                 range_min, range_max, lidar_offset_x=0.0):
        ranges = np.asarray(ranges, dtype=np.float64)
        # beam i points i * angle_increment ccw from the front, so ranges[0] is straight ahead
        angles = yaw + np.arange(len(ranges)) * angle_increment
        valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges < range_max)
        if not valid.any():
            return
        ranges = ranges[valid]
        angles = angles[valid]

        ox = x + lidar_offset_x * math.cos(yaw)
        oy = y + lidar_offset_x * math.sin(yaw)
        dx, dy = np.cos(angles), np.sin(angles)

        step = self.resolution / 2.0
        n_steps = int(np.max(ranges) / step) + 1
        t = np.arange(n_steps) * step
        mask = t[None, :] < (ranges[:, None] - self.resolution)
        free_x = (ox + dx[:, None] * t[None, :])[mask]
        free_y = (oy + dy[:, None] * t[None, :])[mask]
        self._update(free_x, free_y, self.MISS_UPDATE)

        self._update(ox + dx * ranges, oy + dy * ranges, self.HIT_UPDATE)

    def _update(self, xs, ys, delta):
        cols, rows = self.world_to_cell(xs, ys)
        inside = (cols >= 0) & (cols < self.size) & (rows >= 0) & (rows < self.size)
        flat = np.unique(rows[inside] * self.size + cols[inside])
        values = self.log_odds.reshape(-1)
        values[flat] = np.clip(values[flat] + delta,
                               -self.LOG_ODDS_LIMIT, self.LOG_ODDS_LIMIT)

    def to_occupancy_data(self):
        data = np.full(self.log_odds.shape, -1, dtype=np.int8)
        data[self.log_odds < self.FREE_BELOW] = 0
        data[self.log_odds > self.OCCUPIED_ABOVE] = 100
        return data.reshape(-1).tolist()

    def to_image(self):
        image = np.full(self.log_odds.shape, PIXEL_UNKNOWN, dtype=np.uint8)
        image[self.log_odds < self.FREE_BELOW] = PIXEL_FREE
        image[self.log_odds > self.OCCUPIED_ABOVE] = PIXEL_OCCUPIED
        return np.flipud(image)

    def save(self, yaml_path=DEFAULT_MAP_FILE):
        yaml_path = os.path.expanduser(yaml_path)
        os.makedirs(os.path.dirname(os.path.abspath(yaml_path)), exist_ok=True)
        pgm_path = os.path.splitext(yaml_path)[0] + '.pgm'
        image = self.to_image()
        with open(pgm_path, 'wb') as f:
            f.write(b'P5\n%d %d\n255\n' % (image.shape[1], image.shape[0]))
            f.write(image.tobytes())
        with open(yaml_path, 'w') as f:
            yaml.safe_dump({
                'image': os.path.basename(pgm_path),
                'resolution': self.resolution,
                'origin': [self.origin_x, self.origin_y, 0.0],
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.196,
            }, f)
        return yaml_path


class SavedMap:
    def __init__(self, image, resolution, origin_x, origin_y):
        self.image = image
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y

    @property
    def height(self):
        return self.image.shape[0]

    @property
    def width(self):
        return self.image.shape[1]

    @classmethod
    def load(cls, yaml_path=DEFAULT_MAP_FILE):
        yaml_path = os.path.expanduser(yaml_path)
        with open(yaml_path) as f:
            info = yaml.safe_load(f)
        pgm_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)),
                                info['image'])
        with open(pgm_path, 'rb') as f:
            raw = f.read()
        image = _parse_pgm(raw)
        origin = info['origin']
        return cls(image, float(info['resolution']), float(origin[0]), float(origin[1]))

    def pixel_to_world(self, col, row):
        x = self.origin_x + col * self.resolution
        y = self.origin_y + (self.height - row) * self.resolution
        return x, y

    def world_to_pixel(self, x, y):
        col = (x - self.origin_x) / self.resolution
        row = self.height - (y - self.origin_y) / self.resolution
        return col, row

    def explored_bounds(self, margin_cells=30):
        known_rows, known_cols = np.nonzero(self.image != PIXEL_UNKNOWN)
        if len(known_rows) == 0:
            return 0, 0, self.width, self.height
        return (max(0, known_cols.min() - margin_cells),
                max(0, known_rows.min() - margin_cells),
                min(self.width, known_cols.max() + 1 + margin_cells),
                min(self.height, known_rows.max() + 1 + margin_cells))

    def cell_state_at(self, x, y, radius=0.0):
        col, row = self.world_to_pixel(x, y)
        col, row = int(round(col)), int(round(row))
        # a cell tighter than the planner's inflation, so waypoints on its edge still pass
        r_cells = 0 if radius <= 0 else max(1, int(round(radius / self.resolution)) - 1)
        c0, c1 = col - r_cells, col + r_cells + 1
        r0, r1 = row - r_cells, row + r_cells + 1
        if c0 < 0 or r0 < 0 or c1 > self.width or r1 > self.height:
            return 'unknown'
        offsets = np.arange(-r_cells, r_cells + 1)
        inside = offsets[:, None] ** 2 + offsets[None, :] ** 2 <= r_cells ** 2
        window = self.image[r0:r1, c0:c1][inside]
        if (window == PIXEL_OCCUPIED).any():
            return 'occupied'
        if (window == PIXEL_UNKNOWN).any():
            return 'unknown'
        return 'free'


def _parse_pgm(raw):
    tokens = []
    pos = 0
    while len(tokens) < 4:
        while raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b'#':
            pos = raw.index(b'\n', pos) + 1
            continue
        end = pos
        while not raw[end:end + 1].isspace():
            end += 1
        tokens.append(raw[pos:end])
        pos = end
    pos += 1
    if tokens[0] != b'P5':
        raise ValueError('only binary PGM (P5) maps are supported')
    width, height = int(tokens[1]), int(tokens[2])
    return np.frombuffer(raw, dtype=np.uint8, count=width * height,
                         offset=pos).reshape(height, width).copy()
