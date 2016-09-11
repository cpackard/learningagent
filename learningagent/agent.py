from learningagent import agent_percepts as percepts

class State():
    def __init__(self, location, certainty):
        self.location = location
        self.certainty = certainty

    def __repr__(self):
        return '({}, {})'.format(self.location, self.certainty)

    def __eq__(self, other):
        return (self.location == other.location
                and self.certainty == other.certainty)


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


def prev_state_probability(a, prev_state, prev_result,
                           visible_obstacles,
                           prev_cost_estimates, goal):
    """
    Given that agent is currently at A, return the
    probability that the agent was previously at prev_state.
    """
    def prev_lrta_action_cost(action):
        return LRTA_star_cost(prev_state, action,
                              prev_result.get((prev_state, action)),
                              prev_cost_estimates, goal)

    def best_actions_from_state(s):
        actions_from_s = sorted([action for action in
                                percepts.actions(s)],
                                key=prev_lrta_action_cost)

        best_actions_from_s = [v for v in actions_from_s
                               if prev_lrta_action_cost(v)
                               == prev_lrta_action_cost(actions_from_s[0])]

        return best_actions_from_s

    visible_from_a = percepts.visible_vertices(a, visible_obstacles)

    # actions_from_b = sorted([action for action in
    #                          percepts.actions(prev_state)],
    #                         key=prev_lrta_action_cost)

    # best_actions_from_b = [v for v in actions_from_b
    #                        if prev_lrta_action_cost(v)
    #                        == prev_lrta_action_cost(actions_from_b[0])]
    best_actions_prev = best_actions_from_state(prev_state)

    if (prev_state in visible_from_a and a in best_actions_prev):
        # Only consider vertices visible from A which have
        # A as one of the lowest values for lrta_star_cost
        A_in_best_actions_from_v = [v for v in visible_from_a
                                    if a in best_actions_from_state(v)]

        if len(A_in_best_actions_from_v) == 0:
            return 0
        else:
            return (len(best_actions_prev) / len(A_in_best_actions_from_v))
    else:
        return 0


def current_state_probability(a, prev_state, prev_result,
                              obstacles, prev_cost_estimates,
                              goal):
    """
    Given that the agent was previously in state b, return the
    probability that the agent is currently in state a.
    """
    p_of_b_given_a = prev_state_probability(a, prev_state,
                                            prev_result, obstacles,
                                            prev_cost_estimates, goal)

    p_of_a = 1 / len(percepts.get_locations(
        percepts.visible_vertices(a, obstacles),
        obstacles))

    p_of_b = 1 / len(percepts.get_locations(
        percepts.visible_vertices(prev_state, obstacles),
        obstacles))

    if p_of_b == 0:
        # Unless the agent is somehow teleported inside of a circle,
        # this should never happen
        return 0
    else:
        # Bayes theorem
        return p_of_b_given_a * p_of_a / p_of_b


def refine_possible_locations(prev_state, visible_obstacles,
                              prev_result, prev_cost_estimates,
                              possible_locations, goal):
    """
    Given a list of possible locations from agent percepts, narrow
    down the possibile locations based on agent's previous location.
    """
    possible_states = []

    for s in possible_locations:
        p = current_state_probability(s, prev_state.location, prev_result,
                                      visible_obstacles, prev_cost_estimates,
                                      goal)
        if p > 0:
            possible_states.append(State(s, p))

    return possible_states


def update_certain(possible_agent_locations, prev_state,
                   prev_action, belief_history):
    """
    Given that the agent is certain of its current location,
    update its past beliefs (if applicable) and
    return its updated location and belief history.
    """
    # Agent is sure of its current location
    agent_belief_state = State(possible_agent_locations[0], 1)

    if prev_state and prev_state.certainty < 1:
        # now that agent is sure of its location, update past
        # beliefs to correct any false assumptions
        # TODO agent.update_previous_beliefs(belief_history, agent_belief_state)
        pass

    # TODO Assuming prev_action is relative, this should add
    # prev_state + prev_action == agent_belief_state.location
    if prev_action == agent_belief_state.location:
        # agent is certain of current and past position
        # and is at its intended destination, no further work to do
        pass
    else:
        # agent did not end at intended destination
        # TODO agent.recovery_scheme()
        pass

    belief_history = []

    return agent_belief_state, belief_history


def update_uncertain(possible_agent_locations, prev_state,
                     prev_action, belief_history,
                     obstacles, prev_result,
                     prev_cost_estimates, goal):
    """
    Given that the agent is uncertain of its current location,
    refine its estimates to maximize the certainty of its current location.
    """
    # See if we can narrow down the location
    refined_locations = refine_possible_locations(
        prev_state, obstacles, prev_result, prev_cost_estimates,
        possible_agent_locations, goal)

    if len(refined_locations) == 1:
        return update_agent_location(prev_state, [refined_locations[0].location],
                                     belief_history, prev_action, obstacles,
                                     prev_result, prev_cost_estimates,
                                     goal)
    elif len(refined_locations) == 0:
        # None of the current states are possible with the prev_state
        prev_possible_states = belief_history[-1]

        for l in sorted(prev_possible_states, key=lambda l: l.certainty):
            refined_locations = refine_possible_locations(
                l, obstacles, prev_result, prev_cost_estimates,
                possible_agent_locations, goal)

            if not refined_locations:
                # Given the agent's current percepts and its past belief state,
                # there is no possible location it could be in.
                # This means agent guessed the wrong state last time,
                # so check the next most likely state from the last location.
                continue
            else:
                return update_uncertain([s.location for s in refined_locations],
                                        l, prev_action, belief_history,
                                        obstacles, prev_result,
                                        prev_cost_estimates, goal)
    else:
        # agent is unsure of past or current location, make best guess
        belief_history.append(refined_locations)
        agent_belief_state = max(refined_locations, key=lambda s: s.certainty)

        return agent_belief_state, belief_history


def update_agent_location(prev_state, possible_agent_locations,
                          belief_history, prev_action,
                          obstacles, prev_result, prev_cost_estimates,
                          goal):
    """
    Given a previous state prev_state and a list of possible locations,
    determine the agent's current location.
    If agent is certain of current location, update belief history if
    applicable. Otherwise, return the state with the highest certainty and
    add this set of beliefs to the belief history.
    """
    if len(possible_agent_locations) == 1:
        agent_belief_state, belief_history = update_certain(
            possible_agent_locations, prev_state, prev_action, belief_history)
    else:
        if not prev_state:
            # no previous information, make best guess
            p = 1 / len(possible_agent_locations)
            current_beliefs = [State(location, p)
                               for location in possible_agent_locations]
            belief_history.append(current_beliefs)
            # Agent doesn't have any extra information, so just
            # pick the best guess
            # TODO pick one with min of lrta_action_cost
            agent_belief_state = State(possible_agent_locations[0], p)
        else:
            agent_belief_state, belief_history = update_uncertain(
                possible_agent_locations, prev_state, prev_action,
                belief_history, obstacles, prev_result,
                prev_cost_estimates, goal)

    return agent_belief_state, belief_history


# def update_agent_location(prev_state, visible_obstacles,
#                           prev_result, prev_cost_estimates,
#                           initial_locations, belief_history,
#                           prev_action):
#     """
#     Given a set of possible locations the agent might be,
#     refine the location and update belief history if applicable.
#     """

#     if len(initial_locations) == 1:
#         # Agent is certain of its current location

#     if not belief_history:
#         p = 1 / len(initial_locations)
#         current_beliefs = [State(location, p)
#                            for location in initial_locations]
#         belief_history.append(current_beliefs)
#         # Agent doesn't have any extra information, so just
#         # pick the best guess
#         # TODO pick one with min of lrta_action_cost
#         agent_belief_location = initial_locations[0]

#         return agent_belief_location, belief_history
#     else:
#         prev_possible_locations = belief_history[-1:]

#         for l in sorted(prev_possible_states, key=lambda l: l.certainty):
#             possible_agent_locations = refine_possible_locations(
#                 l, visible_obstacles, prev_result,
#                 prev_cost_estimates, initial_locations)


#             if not possible_agent_locations:
#                 # Given the agent's current percepts and its past belief state,
#                 # there is no possible location it could be in.
#                 # This means agent guessed the wrong state last time,
#                 # so check the next most likely state from the last location.
#                 continue
#             else:
#                 return find_current_location(l, possible_agent_locations,
#                                              belief_history, prev_action)

#         # None of the agent's past locations are compatible with
#         # the current percepts
#         return None, belief_history


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
