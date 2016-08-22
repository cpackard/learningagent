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


def goal_test(p, goal):
    """
    Give a position p and a goal point, determine if there is an unblocked
    path from p to the goal.
    """
    return p == goal


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


def perform_action(action):
    """
    Given an action, simulate the agent performing that action and return
    the new location of the agent as a result of that action.
    """
    # TODO Add logic to have the agent land at a nearby vertex
    # (instead of the intended destination) 30% of the time
    return action


def LRTA_star_cost(prev_state, action, state, cost_estimates, goal):
    """
    Returns the f-value of a given state: g(s) + h(s)
    """
    if state is None:
        return heuristic(prev_state, goal)
    else:
        return actual_cost(prev_state, action, state) + cost_estimates[state]




# LRTA* algorithm from AIMA
def LRTA_star_agent(state, goal, obstacles, result,
                    cost_estimates, prev_state, prev_action):
    """
    Given a percept that identifies the current state, return the next
    action to take, or None if the goal is reached.

    LRTA_star_agent select an action according to the values of neighboring
    states, which are updated as the agent moves about the state space.
    """
    if goal_test(state, goal):
        return None, prev_state, result, cost_estimates

    if state not in cost_estimates:
        cost_estimates[state] = heuristic(state, goal)

    if prev_state:
        result[(prev_state, prev_action)] = state

        cost_estimates[prev_state] = min(
            [LRTA_star_cost(
                prev_state,
                action,
                result.get((prev_state, action)),
                cost_estimates,
                goal)
             for action in actions(prev_state)])

    # TODO Possibly refactor this to another location
    def lrta_action_cost(action):
        return LRTA_star_cost(state, action,
                              result.get((state, action)), cost_estimates, goal)

    if line_is_unblocked(state, goal, obstacles):
        prev_action = goal
    else:
        prev_action = min([action for action in actions(state)], key=lrta_action_cost)

    prev_state = state

    return prev_action, prev_state, result, cost_estimates


def run_simulation(number_of_turns, goal_point, initial_location, obstacles):
    """
    Given the number of turns allowed, run the simulation for the agent
    navigating a maze of polygons.
    """
    agent_location = initial_location
    remaining_turns = number_of_turns

    # persistent for LRTA*

    # a table indexed by state and action
    result = {}
    # a table of cost estimates indexed by state
    cost_estimates = {}
    prev_state = None
    prev_action = None

    print('Agent starting at point {}'.format(initial_location))
    print('Agent goal: {}'.format(goal_point))

    while remaining_turns > 0:
        print('Agent currently at point {}'.format(agent_location))

        prev_action, prev_state, result, cost_estimates = LRTA_star_agent(
            agent_location, goal_point, obstacles, result,
            cost_estimates, prev_state, prev_action)

        if not prev_action:
            # TODO Add logic to re-spawn the agent at a random location
            # after reaching the goal.
            print("Agent has reached the goal in {} turns!".format(
                number_of_turns - remaining_turns - 1))
            return True

        print('Agent attempting to reach point {}'.format(prev_action))

        agent_location = perform_action(prev_action)

        print('Agent now at point {}'.format(agent_location))

        remaining_turns -= 1

    print('Agent failed to find goal in alloted turns.')
    print('Final agent location: {}'.format(agent_location))

    return False


if __name__ == "__main__":
    number_of_turns = 2000
    goal_point = lines.Point(34, 22)
    initial_location = lines.Point(5, 5)

    run_simulation(number_of_turns, goal_point,
                   initial_location, obstacles.obstacles)
