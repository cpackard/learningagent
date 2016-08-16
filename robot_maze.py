import lines
import obstacles


def line_is_unblocked(p, r, obstacles):
    """
    Given a line pr and a set of obstacles, determine if any of the
    obstacles block line pr.
    """
    for obstacle in obstacles:
        if (lines.point_inside_own_obstacle(p, obstacle) and
            lines.point_inside_own_obstacle(r, obstacle)):
            if (not lines.line_is_valid_for_own_obstacle(p, r, obstacle) or
                lines.obstacle_blocks_line(p, r, obstacle)):
                return False
        elif lines.obstacle_blocks_line(p, r, obstacle):
                return False

    return True


def visible_vertices(p, obstacles):
    """
    Given a point p and a set of obstacles S, return a list of vertices
    visible from p.
    """
    V = []

    for obstacle in obstacles:
        for vertex in [v[0] for v in obstacle.lines]:
            if line_is_unblocked(p, vertex, obstacles):
                V.append(vertex)

    return V


def vertices_relative_to_agent(vertices, p):
    """
    Given a list of vertices and the agent's position p,
    return a list of vertices with distances relative to p.
    """
    relative_vertices = []

    for vertex in vertices:
        relative_vertex = lines.Point((vertex.x - p.x), (vertex.y - p.y))
        relative_vertices.append(relative_vertex)

    return relative_vertices


def goal_test(p, goal, obstacles):
    """
    Give a position p and a goal point, determine if there is an unblocked
    path from p to the goal.
    """
    return line_is_unblocked(p, goal, obstacles)


def heuristic(p, goal):
    """
    Heuristic function to return an (optimistic) cost estimate from
    point p to the goal.
    """
    return lines.distance(p, goal)


def actual_cost(s1, action, s2):
    """
    Given an action and two states s1, s2, return the cost of performing
    the action in state s1 to end in state s2.
    """
    # For now, we will model an action simply as the destination point of the
    # agent. Therefore, we will assume action == s2, as though the agent either
    # reached the intended destination, or recovered from an error and then
    # reached the destination.
    return lines.distance(s1, s2)


def actions(state):
    """
    Given a state, return a list of all possible actions from that state.
    """
    return visible_vertices(state, obstacles.obstacles)


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
            possible_starting_points.append(
                lines.Point((line[0].x - percepts[0].x),
                            (line[0].y - percepts[0].y)))
            unique_points.append(line[0])

    for v in possible_starting_points:
        # Create a new point from v (possible agent start position) and percepts.
        # If all new points are actual vertice in on obstacle, v is valid
        valid_point = True

        for percept in percepts:
            vertex = lines.Point((v.x + percept.x), (v.y + percept.y))

            if not any([vertex == p for p in unique_points]):
                valid_point = False
                break

        if valid_point:
            possible_locations.append(v)

    return possible_locations


# persistent for LRTA*
result = {} # a table indexed by state and action, initially empty
H = {} # a table of cost estimates indexed by state, initially empty
s = None # the previous state, initially null
a = None # the previous action, initially null


def LRTA_star_cost(prev_state, action, state, h_table):
    """
    Returns the f-value of a given state: g(s) + h(s)
    """
    if state is None:
        return heuristic(prev_state)
    else:
        return actual_cost(prev_state, action, state) + h_table[state]


# LRTA* algorithm from AIMA
# TODO Revisit this and make sure implementation is correct
def LRTA_star_agent(state):
    """
    Given a percept that identifies the current state, return the next
    action to take, or None if the goal is reached.
    """
    if goal_test(state):
        return None

    if state not in H:
        H[state] = heuristic(state)

    if s is not null:
        result[(s, a)] = state
        H[state] = min([LRTA_star_cost(s, b, result[(s, b)], H) for b in actions(s)])

    a = min([(b, LRTA_star_cost(state, b, result[(state, b)], H)) for b in actions(state)])
    s = state

    return a
