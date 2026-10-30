"""
WHCA* on small hand made instances, outside ROS.

    python3 whca_demo.py

Opens a window with the scenario on the left and its description,
parameters and result on the right. `plan` runs the planner; the step
buttons and the slider walk through the solution.

It needs no Gazebo, no colcon and no source. The planning layer only
imports math, heapq and itertools, and reads six attributes off the
OccupancyGrid, so the Grid class below is enough to run the whole
planner.
"""

import filecmp
import glob
import os
import sys
import textwrap

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from matplotlib.widgets import Button, Slider

PACKAGE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'ros2_ws', 'src', 'navigation', 'swarm_mapf')
sys.path.insert(0, PACKAGE)

from swarm_mapf.planners import whca          # noqa: E402

CELL = 1.0           # one character of the drawing is one lattice cell
WINDOW = 64          # steps of cooperation; large, to see the algorithm
SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']

WALL = '#b8b6b0'
FLOOR = '#ffffff'
GRIDLINE = '#e4e2dc'
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_2 = '#52514e'


def check_installed_copy():
    """
    Warn when the copy ROS runs differs from the source used here.

    colcon build copies the files into install/, so editing the source
    without rebuilding leaves two versions: this demo tests one and the
    robot runs the other.
    """
    target = glob.glob(os.path.join(
        PACKAGE, '..', '..', '..', 'install', 'swarm_mapf', 'lib',
        'python3*', 'site-packages', 'swarm_mapf'))
    if not target:
        return

    source = os.path.join(PACKAGE, 'swarm_mapf')
    differing = []
    for folder, _, files in os.walk(source):
        for name in files:
            if not name.endswith('.py'):
                continue
            path = os.path.join(folder, name)
            relative = os.path.relpath(path, source)
            copy = os.path.join(target[0], relative)
            if os.path.exists(copy) and not filecmp.cmp(path, copy,
                                                        shallow=False):
                differing.append(relative)

    if differing:
        print('Warning: the installed copy does not match the source.')
        print('This demo uses the source; the robot would use the copy.')
        for relative in sorted(differing):
            print('   ', relative)
        print('To match them: colcon build --packages-select swarm_mapf')


# ─── The scenarios ───────────────────────────────────────────────────
#
#   #  wall           A  where robot A starts
#   .  free           a  where robot A is going
#
# When a goal lands on another robot's start the drawing cannot hold
# both characters, so those scenarios give the goal in 'goals'.

SCENARIOS = [
    {
        'title': 'Single lane corridor',
        'tests': 'instance with no solution',
        'why':
            'Two robots swap places along a corridor one cell wide. '
            'There are no side cells.\n\n'
            'No sequence of moves solves the instance, so the planner '
            'has to report that no route exists. Failing is the '
            'expected result.',
        'map': """
#########
#A...B.a#
#########
""",
        'goals': {'B': (1, 1)},
    },
    {
        'title': 'Side cell, far',
        'tests': 'incompleteness with a solution available',
        'why':
            "The corridor has a side cell, placed on A's end. B needs "
            'four steps to reach it and A passes it on the first.\n\n'
            'A solution exists: if A stayed put while B entered the side '
            'cell, A could cross and B leave afterwards. WHCA* does not '
            'find it because it plans in priority order and the first '
            'robot plans without constraints, so it never waits.\n\n'
            'The limitation belongs to prioritised planning. CBS does '
            'not have it, because it can constrain the highest priority '
            'robot as well.',
        'map': """
#########
#A...B.a#
##.######
#########
""",
        'goals': {'B': (1, 2)},
    },
    {
        'title': 'Side cell, near',
        'tests': 'effect of where the side cell is',
        'why':
            'The same instance with the side cell next to B.\n\n'
            'B steps into it, stays while A crosses the corridor and '
            'leaves once A has freed its position. The only difference '
            'from the previous instance is where that cell sits.\n\n'
            'Waiting is an action of the search like moving: the state '
            'carries the time step, so staying on a cell is a valid '
            'transition.\n\n'
            'B takes one extra move before entering. Its arrival time '
            'depends on when A frees the corridor, so several routes '
            'cost exactly the same and the search returns any one of '
            'them.',
        'map': """
#########
#A...B.a#
#####.###
#########
""",
        'goals': {'B': (1, 2)},
    },
    {
        'title': 'Bottleneck',
        'tests': 'waiting caused by contention over a single cell',
        'why':
            'Two rooms joined by one cell. All three robots have to '
            'cross it.\n\n'
            'A and C are the same distance from that cell and need it on '
            'the same step. There is no alternative route, so the lower '
            'priority robot waits its turn.\n\n'
            'This does not happen in the simulation scenarios: the '
            'distances to the bottlenecks differ and the robots end up '
            'staggered without having to wait.',
        'map': """
#############
#A....#....a#
#B.........b#
#C....#....c#
#############
""",
    },
    {
        'title': 'Bottleneck, reversed priority',
        'tests': 'effect of the priority order',
        'why':
            'The same instance planned in the opposite order.\n\n'
            'A different robot waits and the routes change. The priority '
            'order is the only parameter WHCA* has, and it decides how '
            'the cost is shared between the robots.',
        'map': """
#############
#A....#....a#
#B.........b#
#C....#....c#
#############
""",
        'priority': ['C', 'B', 'A'],
    },
    {
        'title': 'Crossing',
        'tests': 'contention away from a corridor',
        'why':
            'A moves horizontally and B vertically. Both go through the '
            'centre cell and are the same distance from it.\n\n'
            'This is not a corridor: there is free space around. Even '
            'so the centre cell takes one robot per step, so the lower '
            'priority one waits until it is free.',
        'map': """
#########
####B####
####.####
##A....a#
####.####
####b####
#########
""",
    },
    {
        'title': 'A goal that blocks the way',
        'tests': 'the goal cell stays reserved',
        'why':
            "A's goal is the single crossing cell. A arrives and stays "
            'there.\n\n'
            'B has no other route, so the instance has no solution. The '
            'result checks that the table holds the cell of a robot that '
            'already arrived up to the search bound. If it released it, '
            'B would plan over a robot standing still.',
        'map': """
#########
#A..#...#
#.......#
#B..#..b#
#########
""",
        'goals': {'A': (4, 2)},
    },
]


# ─── The least a map has to be for the planner ───────────────────────

class Grid:
    """What the planner sees of a nav_msgs/OccupancyGrid."""

    class _Info:
        class _Origin:
            class _Point:
                x = 0.0
                y = 0.0
            position = _Point()
        resolution = CELL

    def __init__(self, width, height, data):
        self.info = Grid._Info()
        self.info.width = width
        self.info.height = height
        self.info.origin = Grid._Info._Origin()
        self.data = data


def parse(drawing):
    """Turn the drawing into a grid, the starts and the goals."""
    rows = [row for row in drawing.strip('\n').split('\n') if row.strip()]
    height, width = len(rows), max(len(row) for row in rows)
    data = [0] * (width * height)
    starts, goals = {}, {}

    for number, row in enumerate(rows):
        j = height - 1 - number        # the drawing reads down, the grid up
        for i, character in enumerate(row):
            if character == '#':
                data[j * width + i] = 100
            elif character.isupper():
                starts[character] = (i, j)
            elif character.islower():
                goals[character.upper()] = (i, j)

    return Grid(width, height, data), starts, goals


def build(scenario):
    """Assemble the scenario. It does not plan it."""
    grid, starts, goals = parse(scenario['map'])
    goals.update(scenario.get('goals', {}))

    # The order of this dictionary is the priority: whca reads it there
    order = scenario.get('priority') or sorted(starts)
    return grid, {name: starts[name] for name in order}, goals


# ─── The window ──────────────────────────────────────────────────────

def _wrap(text, width=62):
    """Break the text to a fixed width, keeping the paragraphs."""
    return '\n\n'.join(textwrap.fill(paragraph, width)
                       for paragraph in text.split('\n\n'))


class Demo:
    """One scenario at a time: plan it, then walk through it."""

    def __init__(self):
        self.fig = plt.figure(figsize=(15, 7.6), facecolor=SURFACE)
        self.fig.canvas.manager.set_window_title('WHCA* step by step')

        self.map_ax = self.fig.add_axes([0.03, 0.18, 0.56, 0.76])
        self.text_ax = self.fig.add_axes([0.62, 0.18, 0.36, 0.76])
        self.text_ax.axis('off')

        self.buttons = []
        self._button(0.03, 'scenario -', lambda: self.move(-1))
        self._button(0.14, 'scenario +', lambda: self.move(+1))
        self._button(0.27, 'plan', self.solve)
        self._button(0.39, 'step -', lambda: self.go_to(self.step - 1))
        self._button(0.50, 'step +', lambda: self.go_to(self.step + 1))
        self.b_play = self._button(0.61, 'play', self.play)

        bar = self.fig.add_axes([0.72, 0.055, 0.25, 0.03])
        self.slider = Slider(bar, 'step', 0, 1, valinit=0, valstep=1,
                             color=INK_2, initcolor='none')
        self.slider.on_changed(lambda value: self.go_to(int(value), False))

        self.timer = self.fig.canvas.new_timer(interval=600)
        self.timer.add_callback(self._tick)
        self.running = False

        self.index = 0
        self.load()

    def _button(self, x, text, action):
        axes = self.fig.add_axes([x, 0.04, 0.10, 0.06])
        button = Button(axes, text, color=SURFACE, hovercolor='#eeece6')
        button.label.set_fontsize(9)
        button.on_clicked(lambda _: action())
        self.buttons.append(button)
        return button

    # ── scenario ──

    def move(self, offset):
        self.index = (self.index + offset) % len(SCENARIOS)
        self.load()

    def load(self):
        """Set the scenario up without planning it yet."""
        self.pause()
        self.scenario = SCENARIOS[self.index]
        self.grid, self.starts, self.goals = build(self.scenario)
        self.paths = None
        self.planned = False
        self.last_step = 0
        self.step = 0
        self.colour = {name: SERIES[i % len(SERIES)]
                       for i, name in enumerate(sorted(self.starts))}
        self._fit_slider()
        self.draw()
        self.explain()

    def solve(self):
        """Run the planner and leave the solution ready to walk."""
        self.pause()
        self.paths = whca.plan(self.starts, self.goals, self.grid,
                               CELL, window=WINDOW)
        self.planned = True
        self.last_step = (0 if self.paths is None
                          else max(len(p) for p in self.paths.values()) - 1)
        self.step = 0

        if self.paths is None:
            print('%s: no solution' % self.scenario['title'])
        else:
            print('%s:' % self.scenario['title'])
            for name, path in self.paths.items():
                print('   %s  %s' % (name, ' '.join('(%d,%d)' % cell
                                                    for cell in path)))

        self._fit_slider()
        self.draw()
        self.explain()

    # ── walking through it ──

    def _fit_slider(self):
        self.slider.valmax = max(self.last_step, 1)
        self.slider.ax.set_xlim(0, max(self.last_step, 1))
        self.slider.set_val(self.step)

    def go_to(self, step, move_slider=True):
        if not self.planned or self.paths is None:
            return
        step = max(0, min(step, self.last_step))
        if step == self.step:
            return
        self.step = step
        if move_slider:
            self.slider.set_val(step)
        self.draw()
        self.explain()
        self.fig.canvas.draw_idle()

    def play(self):
        if self.paths is None:
            return
        if self.running:
            self.pause()
            return
        if self.step >= self.last_step:
            self.go_to(0)
        self.running = True
        self.b_play.label.set_text('pause')
        self.timer.start()

    def pause(self):
        if getattr(self, 'running', False):
            self.timer.stop()
        self.running = False
        if hasattr(self, 'b_play'):
            self.b_play.label.set_text('play')

    def _tick(self):
        if self.step >= self.last_step:
            self.pause()
            self.fig.canvas.draw_idle()
            return
        self.go_to(self.step + 1)

    # ── where a robot is ──

    def where(self, name, step):
        path = (self.paths or {}).get(name)
        if not path:
            return self.starts[name]
        return path[min(step, len(path) - 1)]

    def waiting(self, name, step):
        path = (self.paths or {}).get(name)
        return bool(path) and 0 < step < len(path) and path[step] == path[step - 1]

    # ── drawing ──

    def draw(self):
        ax = self.map_ax
        ax.clear()
        ax.set_facecolor(SURFACE)

        info = self.grid.info
        for i in range(info.width):
            for j in range(info.height):
                wall = self.grid.data[j * info.width + i] > 50
                ax.add_patch(Rectangle((i, j), 1, 1,
                                       facecolor=WALL if wall else FLOOR,
                                       edgecolor=GRIDLINE, linewidth=0.8))

        for name in sorted(self.starts):
            colour = self.colour[name]

            gi, gj = self.goals[name]
            ax.add_patch(Circle((gi + 0.5, gj + 0.5), 0.40, fill=False,
                                edgecolor=colour, linewidth=2.5,
                                linestyle=(0, (3, 2)), zorder=3))

            path = (self.paths or {}).get(name)
            if path:
                # The whole route faint, what has been walked solid
                ax.plot([c[0] + 0.5 for c in path], [c[1] + 0.5 for c in path],
                        color=colour, linewidth=1.5, alpha=0.15, zorder=2)
                walked = path[:self.step + 1]
                ax.plot([c[0] + 0.5 for c in walked],
                        [c[1] + 0.5 for c in walked],
                        color=colour, linewidth=2.0, alpha=0.40, zorder=4)

            i, j = self.where(name, self.step)
            ax.add_patch(Circle((i + 0.5, j + 0.5), 0.34, facecolor=colour,
                                edgecolor=SURFACE, linewidth=2.0, zorder=5))
            ax.text(i + 0.5, j + 0.5, name, ha='center', va='center',
                    color='white', fontsize=12, fontweight='bold', zorder=6)
            if self.waiting(name, self.step):
                ax.text(i + 0.5, j + 1.02, 'waits', ha='center', va='bottom',
                        color=colour, fontsize=9, fontweight='bold', zorder=6)

        if not self.planned:
            state = 'not planned'
        elif self.paths is None:
            state = 'no solution'
        else:
            state = 'step %d of %d' % (self.step, self.last_step)
        ax.set_title('%s  —  %s' % (self.scenario['title'], state),
                     color=INK, fontsize=13, loc='left', pad=12)

        ax.set_xlim(0, info.width)
        ax.set_ylim(0, info.height)
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        for side in ax.spines.values():
            side.set_visible(False)

    def explain(self):
        ax = self.text_ax
        ax.clear()
        ax.axis('off')

        info = self.grid.info
        ax.text(0, 1.0, 'Scenario %d of %d' % (self.index + 1,
                                               len(SCENARIOS)),
                va='top', color=INK_2, fontsize=9)
        ax.text(0, 0.95, self.scenario['title'], va='top', color=INK,
                fontsize=15, fontweight='bold')
        ax.text(0, 0.88, _wrap('Tests:  ' + self.scenario['tests']),
                va='top', color=INK, fontsize=10)
        ax.text(0, 0.80, _wrap(self.scenario['why']), va='top',
                color=INK_2, fontsize=10, linespacing=1.6)

        parameters = [
            ('planner', 'WHCA*'),
            ('window w', '%d steps' % WINDOW),
            ('cell size', '%.1f m' % CELL),
            ('grid', '%d x %d cells' % (info.width, info.height)),
            ('robots', '%d' % len(self.starts)),
            ('priority', ', '.join(self.starts)),
        ]
        ax.text(0, 0.30, 'Parameters', va='top', color=INK, fontsize=10,
                fontweight='bold')
        ax.text(0, 0.25, '\n'.join('%-12s %s' % pair for pair in parameters),
                va='top', color=INK_2, fontsize=9, linespacing=1.7,
                family='monospace')

        if not self.planned:
            result = 'press plan'
        elif self.paths is None:
            result = 'no solution'
        else:
            rows = []
            for name, path in self.paths.items():
                waits = sum(1 for a, b in zip(path, path[1:]) if a == b)
                rows.append('%-6s %2d steps, %d waits'
                            % (name, len(path) - 1, waits))
            rows.append('%-6s %2d steps'
                        % ('total', sum(len(p) - 1
                                        for p in self.paths.values())))
            rows.append('')
            rows.append('step %d' % self.step)
            for name in self.paths:
                i, j = self.where(name, self.step)
                mark = '  waits' if self.waiting(name, self.step) else ''
                rows.append('%-6s (%d, %d)%s' % (name, i, j, mark))
            result = '\n'.join(rows)

        ax.text(0.55, 0.30, 'Result', va='top', color=INK, fontsize=10,
                fontweight='bold')
        ax.text(0.55, 0.25, result, va='top', color=INK_2, fontsize=9,
                linespacing=1.7, family='monospace')


if __name__ == '__main__':
    check_installed_copy()
    Demo()
    plt.show()
