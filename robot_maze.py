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






# persistent for LRTA*
result = {} # a table indexed by state and action, initially empty
H = {} # a table of cost estimates indexed by state, initially empty
s = None # the previous state, initially null
a = None # the previous action, initially null

def goal_test():
    pass


def heuristic():
    pass


def actions(state):
    pass





def LRTA_star_cost(prev_state, action, state, h_table):
    """
    Returns the f-value of a given state: g(s) + h(s)
    """
    if state is None:
        return heuristic(prev_state)
    else:
        return actual_cost(prev_state, action, state) + h_table[state]


# LRTA* algorithm from AIMA
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

