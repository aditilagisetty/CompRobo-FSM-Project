import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from matplotlib.colors import ListedColormap
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = os.path.dirname(os.path.abspath(__file__))
BAGS = os.path.join(HERE, '..', 'bags')
OUT = os.path.join(HERE, 'figures')

BLUE, GREEN, RED, PURPLE, AMBER, GREY = '#2563EB', '#16A34A', '#DC2626', '#7C3AED', '#D97706', '#6B7280'
STATE_COLORS = {'idle': '#E5E7EB', 'following': '#C4B5FD', 'paused': '#FCD34D', 'done': '#86EFAC'}

plt.rcParams.update({
    'font.family': 'DejaVu Sans', 'font.size': 10, 'axes.titlesize': 11.5, 'axes.titleweight': 'bold',
    'axes.spines.top': False, 'axes.spines.right': False, 'axes.grid': True, 'grid.color': '#E5E7EB',
    'grid.linewidth': 0.8, 'axes.edgecolor': '#9CA3AF', 'figure.facecolor': 'white',
})


def read_bag(name, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=os.path.join(BAGS, name), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {t: [] for t in topics if t in types}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic in out:
            out[topic].append((stamp * 1e-9, deserialize_message(data, get_message(types[topic]))))
    return out


def odom_xy(msgs, t0):
    return np.array([(t - t0, m.pose.pose.position.x, m.pose.pose.position.y) for t, m in msgs])


def cmd_vel(msgs, t0):
    return np.array([(t - t0, m.linear.x, m.angular.z) for t, m in msgs])


def load_map(bag='teleop_scan_demo'):
    """Last /room_map in the mapping bag, as (grid, extent, view limits)."""
    msg = read_bag(bag, ['/room_map'])['/room_map'][-1][1]
    grid = np.array(msg.data).reshape(msg.info.height, msg.info.width)
    res, ox, oy = msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y
    extent = (ox, ox + msg.info.width * res, oy, oy + msg.info.height * res)
    rows, cols = np.nonzero(grid != -1)
    pad = 0.6
    view = (ox + cols.min() * res - pad, ox + cols.max() * res + pad,
            oy + rows.min() * res - pad, oy + rows.max() * res + pad)
    return grid, extent, view


def draw_map(ax, grid, extent, view):
    # unknown = light grey, free = white, occupied = near black
    image = np.zeros(grid.shape, dtype=int)
    image[grid == -1], image[grid == 0], image[grid == 100] = 0, 1, 2
    ax.imshow(image, origin='lower', extent=extent, interpolation='nearest',
              cmap=ListedColormap(['#F3F4F6', '#FFFFFF', '#1F2937']), vmin=0, vmax=2)
    ax.set_xlim(view[0], view[1])
    ax.set_ylim(view[2], view[3])
    ax.set_aspect('equal')
    ax.grid(False)
    ax.set_xlabel('x in odom frame (m)')
    ax.set_ylabel('y in odom frame (m)')


def style_time_axis(ax, label=None):
    ax.set_xlim(left=0)
    if label:
        ax.set_xlabel(label)


def figure_teleop_scan():
    d = read_bag('teleop_scan_demo', ['/odom', '/cmd_vel'])
    t0 = d['/odom'][0][0]
    od, cv = odom_xy(d['/odom'], t0), cmd_vel(d['/cmd_vel'], t0)
    grid, extent, view = load_map()

    fig = plt.figure(figsize=(12.5, 5.2))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.15, 1], hspace=0.12, wspace=0.22)
    ax_map = fig.add_subplot(gs[:, 0])
    ax_lin = fig.add_subplot(gs[0, 1])
    ax_ang = fig.add_subplot(gs[1, 1], sharex=ax_lin)

    draw_map(ax_map, grid, extent, view)
    sc = ax_map.scatter(od[:, 1], od[:, 2], c=od[:, 0], cmap='viridis', s=6, zorder=3)
    ax_map.plot(*od[0, 1:], 'o', color=GREEN, ms=9, mec='white', mew=1.5, zorder=4, label='start')
    ax_map.plot(*od[-1, 1:], 's', color=RED, ms=9, mec='white', mew=1.5, zorder=4, label='end')
    ax_map.legend(loc='upper left', frameon=True, framealpha=0.95)
    fig.colorbar(sc, ax=ax_map, orientation='horizontal', fraction=0.045, pad=0.14).set_label('time (s)')
    ax_map.set_title('Map built while driving with the keyboard')

    ax_lin.step(cv[:, 0], cv[:, 1], where='post', color=BLUE, lw=1.6)
    ax_lin.set_ylabel('linear (m/s)')
    ax_lin.set_title('Commands sent by teleop_scan.py')
    plt.setp(ax_lin.get_xticklabels(), visible=False)
    ax_ang.step(cv[:, 0], cv[:, 2], where='post', color=AMBER, lw=1.6)
    ax_ang.set_ylabel('angular (rad/s)')
    style_time_axis(ax_ang, 'time (s)')

    fig.suptitle('teleop_scan_demo: mapping run (40 s)', fontsize=13, fontweight='bold', y=0.98)
    fig.savefig(os.path.join(OUT, 'teleop_scan_demo.png'), dpi=200, bbox_inches='tight')
    plt.close(fig)


def status_spans(status_msgs, t0, t_end):
    """[(start, end, state)] from the periodic /path_following_status messages."""
    spans, current, start = [], None, 0.0
    for t, m in status_msgs:
        if m.data != current:
            if current is not None:
                spans.append((start, t - t0, current))
            current, start = m.data, t - t0
    spans.append((start, t_end, current))
    return spans


def figure_path_following():
    d = read_bag('path_following_demo', ['/odom', '/cmd_vel', '/path_following_status', '/drawn_path'])
    t0 = d['/odom'][0][0]
    od, cv = odom_xy(d['/odom'], t0), cmd_vel(d['/cmd_vel'], t0)
    spans = status_spans(d['/path_following_status'], t0, od[-1, 0])
    grid, extent, view = load_map()
    paths = [(t - t0, np.array([(p.pose.position.x, p.pose.position.y) for p in m.poses]))
             for t, m in d['/drawn_path']]

    fig = plt.figure(figsize=(12.5, 5.6))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.15, 1], height_ratios=[1, 1, 0.28], hspace=0.14, wspace=0.22)
    ax_map = fig.add_subplot(gs[:, 0])
    ax_lin = fig.add_subplot(gs[0, 1])
    ax_ang = fig.add_subplot(gs[1, 1], sharex=ax_lin)
    ax_st = fig.add_subplot(gs[2, 1], sharex=ax_lin)

    draw_map(ax_map, grid, extent, view)
    # widen the view so the whole driven path is visible
    x_lo, x_hi = min(view[0], od[:, 1].min() - 0.3), max(view[1], od[:, 1].max() + 0.3)
    y_lo, y_hi = min(view[2], od[:, 2].min() - 0.3), max(view[3], od[:, 2].max() + 0.3)
    ax_map.set_xlim(x_lo, x_hi)
    ax_map.set_ylim(y_lo, y_hi)

    path_colors = [PURPLE, AMBER]
    for i, (t, p) in enumerate(paths):
        ax_map.plot(p[:, 0], p[:, 1], '--', color=path_colors[i % 2], lw=2, zorder=3,
                    label=f'path {i + 1} sent at t = {t:.0f} s ({len(p)} waypoints)')
        ax_map.plot(p[:, 0], p[:, 1], 'o', color=path_colors[i % 2], ms=3.5, zorder=3)
    ax_map.plot(od[:, 1], od[:, 2], color=BLUE, lw=2.2, alpha=0.9, zorder=4, label='robot (odometry)')
    ax_map.plot(*od[0, 1:], 'o', color=GREEN, ms=10, mec='white', mew=1.5, zorder=5, label='start')
    ax_map.plot(*od[-1, 1:], 's', color=RED, ms=10, mec='white', mew=1.5, zorder=5, label='end')
    ax_map.legend(loc='upper left', frameon=True, framealpha=0.95, fontsize=8.5)
    ax_map.set_title('Two paths followed by path_following.py')

    for ax in (ax_lin, ax_ang):
        for s, e, state in spans:
            if state == 'following':
                ax.axvspan(s, e, color=STATE_COLORS['following'], alpha=0.35, lw=0)
    ax_lin.step(cv[:, 0], cv[:, 1], where='post', color=BLUE, lw=1.5)
    ax_lin.set_ylabel('linear (m/s)')
    ax_lin.set_title('Commands sent to /cmd_vel')
    plt.setp(ax_lin.get_xticklabels(), visible=False)
    ax_ang.step(cv[:, 0], cv[:, 2], where='post', color=AMBER, lw=1.5)
    ax_ang.set_ylabel('angular (rad/s)')
    plt.setp(ax_ang.get_xticklabels(), visible=False)

    for s, e, state in spans:
        ax_st.axvspan(s, e, ymin=0.1, ymax=0.9, color=STATE_COLORS[state], ec='#9CA3AF', lw=0.8)
        if e - s > 2.5:
            ax_st.text((s + e) / 2, 0.5, state, ha='center', va='center',
                       fontsize=8.5 if e - s > 6 else 7.5, color='#1F2937')
    ax_st.set_ylim(0, 1)
    ax_st.set_yticks([])
    ax_st.set_ylabel('status', rotation=0, ha='right', va='center')
    ax_st.grid(False)
    style_time_axis(ax_st, 'time (s)')

    fig.suptitle('path_following_demo: path following run (56 s)', fontsize=13, fontweight='bold', y=0.99)
    fig.savefig(os.path.join(OUT, 'path_following_demo.png'), dpi=200, bbox_inches='tight')
    plt.close(fig)


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    figure_teleop_scan()
    figure_path_following()
    print('wrote', sorted(f for f in os.listdir(OUT) if f.endswith('.png')))
