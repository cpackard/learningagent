# Module for the agent perceiving and acting on the environment.

from random import randint

from learningagent import geometry_helpers as geom
from learningagent import environment_details as env

def line_is_unblocked(p, r, visible_obstacles):
    """
    Given a line pr and a set of obstacles, determine if any of the
    obstacles block line pr.
    """
    for obstacle in visible_obstacles:
        if (geom.point_inside_own_obstacle(p, obstacle) and
            geom.point_inside_own_obstacle(r, obstacle)):
            if (not geom.line_is_valid_for_own_obstacle(p, r, obstacle) or
                geom.obstacle_blocks_line(p, r, obstacle)):
                return False
        elif geom.obstacle_blocks_line(p, r, obstacle):
                return False

    return True


def visible_vertices(p, visible_obstacles):
    """
    Given a point p and a set of obstacles S, return a list of vertices
    visible from p.
    """
    V = []

    for obstacle in visible_obstacles:
        for vertex in [v[0] for v in obstacle.lines]:
            if line_is_unblocked(p, vertex, visible_obstacles):
                V.append(vertex)

    return V


def vertices_relative_to_agent(vertices, p):
    """
    Given a list of vertices and the agent's position p,
    return a list of vertices with distances relative to p.
    """
    relative_vertices = []

    for vertex in vertices:
        relative_vertex = geom.Point((vertex.x - p.x), (vertex.y - p.y))
        relative_vertices.append(relative_vertex)

    return relative_vertices


def heuristic(p, goal):
    """
    Heuristic function to return an (optimistic) cost estimate from
    point p to the goal.
    """
    return geom.distance(p, goal)


def actual_cost(s1, action, s2):
    """
    Given an action and two states s1, s2, return the cost of performing
    the action in state s1 to end in state s2.
    """
    # For now, we will model an action simply as the destination point of the
    # agent. Therefore, we will assume action == s2, as though the agent either
    # reached the intended destination, or recovered from an error and then
    # reached the destination.
    return geom.distance(s1, s2)


def actions(state):
    """
    Given a state, return a list of all possible actions from that state.
    """
    return visible_vertices(state, env.visible_obstacles)


def get_locations(percepts, obstacles):
    """
    Given a list of percepts (in our case, a list of visible vertices)
    relative to the agent and a list of obstacles (or map of the environment),
    return a list of possible locations where the agent might be.
    """
    possible_locations = []
    possible_starting_points = []
    unique_points = []

    # TODO We should add a check to make sure we don't consider
    # any points inside an obstacle
    for obstacle in obstacles:
        for line in obstacle.lines:
            possible_starting_points.append(geom.Point((line[0].x - percepts[0].x),
                                                       (line[0].y - percepts[0].y)))
            unique_points.append(line[0])

    for v in possible_starting_points:
        # Create a new point from v (possible agent start position) and percepts.
        # If all new points are actual vertice in on obstacle, v is valid
        valid_point = True

        for percept in percepts:
            vertex = geom.Point((v.x + percept.x), (v.y + percept.y))

            if not any([vertex == p for p in unique_points]):
                valid_point = False
                break

        if valid_point:
            possible_locations.append(v)

    return possible_locations


def perform_action(action):
    """
    Given an action, simulate the agent performing that action and return
    the new location of the agent as a result of that action.
    """
    # TODO Add logic to have the agent land at a nearby vertex
    # (instead of the intended destination) 30% of the time
    return action


def get_new_position(obstacles, x_bounds, y_bounds):
    """
    Assign a random location inside the maze that isn't inside
    any of the obstacles.
    """
    while True:
        x = randint(0, x_bounds)
        y = randint(0, y_bounds)

        if not geom.point_in_any_obstacle(geom.Point(x, y), obstacles):
            return geom.Point(x, y)
