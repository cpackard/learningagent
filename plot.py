import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon, YAArrow, Circle
from matplotlib.text import Text
import ast
import sys

import environment_details

def update_plot(p1, p2, arrow, ax1, fig1, reset_points, line):
    """
    Given a line with an agent's move and the current plot, update
    the plot based on the agent's move.
    """
    l = line.strip()

    if 'Agent has reached the goal' in l:
        # TODO Change this when we add agent-reset logic
        return p1, p2, arrow, ax1, fig1, reset_points

    p = ast.literal_eval(l[l.find('('):])

    if 'currently at point' in l:
        p1 = Circle(p, radius=0.2, facecolor='yellow')
        ax1.add_patch(p1)
    elif 'attempting to reach point' in l:
        p2 = Circle(p, radius=0.2, facecolor='green')
        ax1.add_patch(p2)
    elif 'now at point' in l:
        arrow = YAArrow(fig1, p2.center, p1.center, width=0.1,
                        headwidth=0.5, facecolor='red')
        ax1.add_patch(arrow)
        reset_points = True
    elif 'Agent score' in l:
        ax1.text(2, 33, 'Agent Score: {0:.2f}'.format(float(l.split()[2])),
                 bbox=dict(facecolor='grey'))

    return p1, p2, arrow, ax1, fig1, reset_points


def process_remaining_lines(p1, p2, arrow, ax1, fig1, f):
    """
    Given a record of agent movements, update the plot based on the
    contents of the record.
    """
    count = 0
    leading_zeroes = '0000000'
    reset_points = False

    for line in f:
        file_number = (leading_zeroes [:len(leading_zeroes) - len(str(count))]
                       + str(count))

        fig1.savefig('{}{}.png'.format(name_prefix, file_number),
                    dpi=90, bbox_inches='tight')
        count += 1

        if reset_points:
            p1.remove()
            p2.remove()
            arrow.remove()
            reset_points = False

        p1, p2, arrow, ax1, fig1, reset_points = update_plot(
            p1, p2, arrow, ax1, fig1, reset_points, line)

    return


def generate_images(input_file, name_prefix, visible_obstacles):
    """
    Given an input file which logs an agent's movements,
    plot and save a sequence of images which correspond
    to those movements.
    """
    obstacle_plots = []

    for obstacle in visible_obstacles:
        obstacle_plots.append([ast.literal_eval(repr(line[0]))
                               for line in obstacle.lines])

    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111, aspect='equal', ylim=[0.0, 35], xlim=[0.0, 42])
    ax1.text(2, 33, 'Agent Score: {}'.format(0), bbox=dict(facecolor='grey'))

    for ob in obstacle_plots:
        ax1.add_patch(Polygon(ob))

    arrow = None
    p1 = None
    p2 = None

    with open(input_file) as f:
        # Initialize plot with start and goal points
        l = f.readline().strip()
        starting_point = ast.literal_eval(l[l.find('('):])
        l = f.readline().strip()
        goal_point = ast.literal_eval(l[l.find('('):])

        ax1.add_patch(Circle(starting_point, radius=0.2, facecolor='orange'))
        ax1.add_patch(Circle(goal_point, radius=0.2, facecolor='orange'))

        process_remaining_lines(p1, p2, arrow, ax1, fig1, f)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print('Usage: python plot.py agent_log_file any_naming_prefix')
    else:
        agent_movements = sys.argv[1]
        name_prefix = sys.argv[2]
        visible_obstacles = environment_details.visible_obstacles

        generate_images(agent_movements, name_prefix, visible_obstacles)
