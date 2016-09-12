# Module to simulate an agent's movements in the environment

import sys
import ast

from learningagent import geometry_helpers
from learningagent import environment_details
from learningagent import agent_percepts as percepts
from learningagent import agent

def run_simulation(number_of_turns, goal_point, goal_reward,
                   initial_location, visible_obstacles,
                   x_bounds, y_bounds):
    """
    Given the number of turns allowed, run the simulation for the agent
    navigating a maze of polygons.
    """
    agent_belief_state = agent.State(initial_location, 1)
    actual_location = initial_location
    remaining_turns = number_of_turns

    # persistent for LRTA*

    # a table indexed by state and action
    result = {}
    # a table of cost estimates indexed by state
    cost_estimates = {}
    prev_state = agent.State(None, 1)
    prev_action = None
    agent_score = 0
    agent_reached_goal = False
    belief_history = []

    print('Agent starting at point {}'.format(initial_location))
    print('Agent goal: {}'.format(goal_point))

    while remaining_turns > 0:
        print('Agent currently believes it is at point {}'.format(
            agent_belief_state.location))
        print('Agent is actually at point {}'.format(actual_location))

        prev_result = result
        prev_cost_estimates = cost_estimates

        prev_action, prev_state, result, cost_estimates = agent.LRTA_star_agent(
            agent_belief_state.location, goal_point, visible_obstacles, result,
            cost_estimates, prev_state.location, prev_action)

        if not prev_action:
            # Agent reached the goal, reset to a new starting point
            agent_belief_state = agent.State(
                percepts.get_new_position(
                    visible_obstacles, x_bounds, y_bounds),
                1)
            actual_location = agent_belief_state.location
            belief_history = []
            prev_state = agent.State(None, 1)

            print('Resetting agent to point {}'.format(
                agent_belief_state.location))

            agent_score += goal_reward
            print('Agent score: {}'.format(agent_score))

            agent_reached_goal = True

            continue

        print('Agent believes it is attempting to reach point {}'.format(
            prev_action))

        relative_move = geometry_helpers.Point(
            (prev_action.x - agent_belief_state.location.x),
            (prev_action.y - agent_belief_state.location.y))
        actual_target = geometry_helpers.Point(
            (relative_move.x + actual_location.x),
            (relative_move.y + actual_location.y))
        print('Agent actually attempting to reach point {}'.format(
            actual_target))

        # Simulation knows actual agent location
        actual_location = percepts.perform_action(prev_action)
        # Agent only knows possible locations based on relative visible vertices
        initial_locations = percepts.get_locations(
            percepts.vertices_relative_to_agent(
                percepts.visible_vertices(actual_location,
                                          visible_obstacles),
                actual_location),
            visible_obstacles)

        prev_state = agent_belief_state

        agent_belief_state, belief_history = agent.update_agent_location(
            prev_state, initial_locations, belief_history,
            prev_action, visible_obstacles,
            prev_result, prev_cost_estimates, goal_point)

        
        print('Agent now at point {}'.format(actual_location))
        agent_score -= geometry_helpers.distance(prev_state.location,
                                                 actual_location)
        print('Agent score: {}'.format(agent_score))

        remaining_turns -= 1

    if not agent_reached_goal:
        print('Agent failed to find goal in alloted turns.')
        print('Final agent location: {}'.format(agent_location))

    return agent_reached_goal


if __name__ == "__main__":
    if len(sys.argv) < 5:
        print("Usage: python simulation.py number_of_turns goal_reward"
        " \"goal_point\" \"initial_point\"")
    else:
        number_of_turns = int(sys.argv[1])
        goal_reward = int(sys.argv[2])

        user_goal = ast.literal_eval(sys.argv[3])
        goal_point = geometry_helpers.Point(user_goal[0], user_goal[1])

        user_init = ast.literal_eval(sys.argv[4])
        initial_location = geometry_helpers.Point(user_init[0], user_init[1])

        visible_obstacles = environment_details.visible_obstacles
        x_bounds = environment_details.x_bounds
        y_bounds = environment_details.y_bounds

        run_simulation(number_of_turns, goal_point, goal_reward,
                       initial_location, visible_obstacles,
                       x_bounds, y_bounds)
