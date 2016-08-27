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
    agent_location = initial_location
    remaining_turns = number_of_turns

    # persistent for LRTA*

    # a table indexed by state and action
    result = {}
    # a table of cost estimates indexed by state
    cost_estimates = {}
    prev_state = None
    prev_action = None
    agent_score = 0
    agent_reached_goal = False

    print('Agent starting at point {}'.format(initial_location))
    print('Agent goal: {}'.format(goal_point))

    while remaining_turns > 0:
        # TODO change this to get_locations.
        # Agent has to figure out where it is on the map itself!
        # NOTE: changing this will also change the movement logic.
        # Agent will have to declare which point it moves to relative
        # to its own location, i.e.: move to vertex 2 up, 1 right.
        # From that, it will be trivial to translate that into an
        # absolute coordinate, from which we can make the actual move.
        # We should also log where the agent *thinks* it is, to see
        # the distinction.
        # When presented with multiple possible locations, agent will
        # pick the one that assumes it is closest to the goal.
        print('Agent currently at point {}'.format(agent_location))

        prev_action, prev_state, result, cost_estimates = agent.LRTA_star_agent(
            agent_location, goal_point, visible_obstacles, result,
            cost_estimates, prev_state, prev_action)

        if not prev_action:
            # TODO assigning None to prev_action may have unexpected
            # consequences - make sure this works.
            agent_location = percepts.get_new_position(
                visible_obstacles, x_bounds, y_bounds)
            print('Resetting agent to point {}'.format(agent_location))

            agent_score += goal_reward
            print('Agent score: {}'.format(agent_score))

            agent_reached_goal = True

            continue

        print('Agent attempting to reach point {}'.format(prev_action))

        agent_location = percepts.perform_action(prev_action)

        print('Agent now at point {}'.format(agent_location))
        agent_score -= geometry_helpers.distance(prev_state, agent_location)
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
