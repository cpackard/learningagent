# Module to simulate an agent's movements in the environment

import geometry_helpers
import environment_details
import agent_percepts as percepts
import agent

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
    number_of_turns = 250
    goal_point = geometry_helpers.Point(34, 22)
    goal_reward = 1000
    initial_location = geometry_helpers.Point(5, 5)
    visible_obstacles = environment_details.visible_obstacles
    x_bounds = environment_details.x_bounds
    y_bounds = environment_details.y_bounds

    run_simulation(number_of_turns, goal_point, goal_reward,
                   initial_location, visible_obstacles,
                   x_bounds, y_bounds)
