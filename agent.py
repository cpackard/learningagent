import agent_percepts as percepts

def goal_test(p, goal):
    """
    Give a position p and a goal point, determine if there is an unblocked
    path from p to the goal.
    """
    return p == goal


def LRTA_star_cost(prev_state, action, state, cost_estimates, goal):
    """
    Returns the f-value of a given state: g(s) + h(s)
    """
    if state is None:
        return percepts.heuristic(prev_state, goal)
    else:
        return (percepts.actual_cost(prev_state, action, state)
                + cost_estimates[state])


# LRTA* algorithm from AIMA
def LRTA_star_agent(state, goal, visible_obstacles, result,
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
        cost_estimates[state] = percepts.heuristic(state, goal)

    if prev_state:
        result[(prev_state, prev_action)] = state

        cost_estimates[prev_state] = min(
            [LRTA_star_cost(
                prev_state,
                action,
                result.get((prev_state, action)),
                cost_estimates,
                goal)
             for action in percepts.actions(prev_state)])

    def lrta_action_cost(action):
        return LRTA_star_cost(state, action,
                              result.get((state, action)), cost_estimates, goal)

    if percepts.line_is_unblocked(state, goal, visible_obstacles):
        prev_action = goal
    else:
        prev_action = min([action for action in percepts.actions(state)],
                          key=lrta_action_cost)

    prev_state = state

    return prev_action, prev_state, result, cost_estimates
