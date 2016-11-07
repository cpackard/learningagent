import numpy as np

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from matplotlib.patches import Polygon, YAArrow, Circle
from matplotlib.text import Text

import pylab

import ast
import sys

from learningagent import environment_details

def update_plot(p1, p2, arrow, txt, ax, fig, reset_points, line):
    """
    Given a line with an agent's move and the current plot, update
    the plot based on the agent's move.
    """
    l = line.strip()

    if 'Agent score' in l:
        txt.remove()
        txt = plt.text(2, 33, 'Agent Score: {0:.2f}'.format(float(l.split()[2])),
                       fontsize=8)
        reset_points = True
    else:
        p = ast.literal_eval(l[l.find('('):])

        if 'actually at point' in l:
            p1 = Circle(p, radius=0.2, facecolor='yellow')
            ax.add_patch(p1)
        elif 'actually attempting to reach point' in l:
            p2 = Circle(p, radius=0.2, facecolor='green')
            ax.add_patch(p2)
        elif 'now at point' in l:
            arrow = YAArrow(fig, p2.center, p1.center, width=0.1,
                            headwidth=0.5, facecolor='red')
            ax.add_patch(arrow)
        elif 'Resetting agent to point' in l:
            p2 = Circle(p, radius=1, facecolor='green')
            ax.add_patch(p2)

            arrow = YAArrow(fig, p2.center, p1.center, width=0.25,
                            headwidth=1, facecolor='red')
            ax.add_patch(arrow)

    return p1, p2, arrow, txt, ax, fig, reset_points


def process_remaining_lines(p1, p2, arrow, txt, ax, fig, im, f):
    """
    Given a record of agent movements, update the plot based on the
    contents of the record.
    """
    reset_points = False

    for line in f:
        # Convert image from fig into a numpy array
        fig.canvas.draw()
        data = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
        data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))

        yield data

        if reset_points:
            p1.remove()
            p2.remove()
            arrow.remove()
            reset_points = False

        p1, p2, arrow, txt, ax, fig, reset_points = update_plot(
            p1, p2, arrow, txt, ax, fig, reset_points, line)


def generate_images(input_file, line_count, video_name, visible_obstacles):
    """
    Given an input file which logs an agent's movements,
    plot and save a sequence of images which correspond
    to those movements.
    """
    obstacle_plots = []

    # Convert obstacles into lists of tuples so matplotlib can
    # understand them.
    for obstacle in visible_obstacles:
        obstacle_plots.append([ast.literal_eval(repr(line[0]))
                               for line in obstacle.lines])

    # Initialize plot and obstacles for the maze
    fig = plt.figure(frameon=False)
    txt = plt.text(2, 33, 'Agent Score: {}'.format(0), fontsize=8)
    ax = fig.add_subplot(111, aspect='equal', ylim=[0.0, 35], xlim=[0.0, 42])

    for ob in obstacle_plots:
        ax.add_patch(Polygon(ob))

    arrow = None
    p1 = None
    p2 = None

    # Remove extra border around subplot
    frame = plt.gca()
    frame.axes.get_xaxis().set_visible(False)
    frame.axes.get_yaxis().set_visible(False)
    frame.set_frame_on(False)

    with open(input_file) as f:
        # Initialize plot with start and goal points
        l = f.readline().strip()
        starting_point = ast.literal_eval(l[l.find('('):])
        l = f.readline().strip()
        goal_point = ast.literal_eval(l[l.find('('):])

        ax.add_patch(Circle(goal_point, radius=0.2, facecolor='orange'))

        # Draw initial state of the maze
        fig.canvas.draw()
        data = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
        data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        im = ax.imshow(data)

        def animate(data, im):
            im.set_data(data)

        # Generator for the rest of the agent's movements, which
        # gets passed as the data parameter for each call to animate.
        remaining_lines = process_remaining_lines(p1, p2, arrow, txt,
                                                  ax, fig, im, f)

        ani = animation.FuncAnimation(
            fig=fig, func=animate,
            frames=remaining_lines,
            interval=10, fargs=(im, ),
            repeat=False, save_count=line_count-2)
        writer = animation.writers['ffmpeg'](fps=15, codec='libx264', bitrate=-1)

        ani.save('{}.mp4'.format(video_name), writer=writer, dpi=100)


if __name__ == "__main__":
    if len(sys.argv) < 4:
        print('Usage: python plot.py agent_log_file file_line_count video_name')
    else:
        agent_movements = sys.argv[1]
        line_count = int(sys.argv[2])
        video_name = sys.argv[3]
        visible_obstacles = environment_details.visible_obstacles

        generate_images(agent_movements, line_count,
                        video_name, visible_obstacles)
