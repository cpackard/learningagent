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
    sim_agent = agent.Agent(goal_point, agent.State(initial_location, 1),
                            visible_obstacles)
    actual_location = initial_location
    remaining_turns = number_of_turns

    print('Agent starting at point {}'.format(initial_location))
    print('Agent goal: {}'.format(goal_point))

    while remaining_turns > 0:
        print('Agent currently believes it is at point {}'.format(
            sim_agent.belief_state.location))
        print('Agent is actually at point {}'.format(actual_location))

        sim_agent.LRTA_star_agent()

        if not sim_agent.prev_action:
            # Agent reached the goal, reset to a new starting point
            sim_agent.belief_state = agent.State(
                percepts.get_new_position(visible_obstacles, x_bounds, y_bounds),
                1)
            actual_location = sim_agent.belief_state.location
            sim_agent.belief_history = []
            sim_agent.prev_state = agent.State(None, 1)

            print('Resetting agent to point {}'.format(
                sim_agent.belief_state.location))

            sim_agent.score += goal_reward
            print('Agent score: {}'.format(sim_agent.score))

            sim_agent.reached_goal = True

            continue

        print('Agent believes it is attempting to reach point {}'.format(
            sim_agent.prev_action))

        relative_move = geometry_helpers.Point(
            (sim_agent.prev_action.x - sim_agent.belief_state.location.x),
            (sim_agent.prev_action.y - sim_agent.belief_state.location.y))

        actual_target = geometry_helpers.Point(
            (relative_move.x + actual_location.x),
            (relative_move.y + actual_location.y))

        print('Agent actually attempting to reach point {}'.format(actual_target))

        # Simulation knows actual agent location
        actual_location = percepts.perform_action(sim_agent.prev_action)

        # Agent only knows possible locations based on relative visible vertices
        relative_verts = percepts.vertices_relative_to_agent(
            percepts.visible_vertices(actual_location,
                                      visible_obstacles),
            actual_location)

        initial_locations = percepts.get_locations(relative_verts, visible_obstacles)

        sim_agent.update_agent_location(initial_locations)

        print('Agent now at point {}'.format(actual_location))
        sim_agent.score -= geometry_helpers.distance(
            sim_agent.prev_state.location, actual_location)
        print('Agent score: {}'.format(sim_agent.score))

        remaining_turns -= 1

    if not sim_agent.reached_goal:
        print('Agent failed to find goal in alloted turns.')
        print('Final agent location: {}'.format(agent_location))

    return sim_agent.reached_goal


if __name__ == "__main__":
    if len(sys.argv) < 5:
        number_of_turns = 250
        goal_reward = 1000
        goal_point = geometry_helpers.Point(34, 22)
        initial_location = geometry_helpers.Point(5, 5)
    else:
        try:
            number_of_turns = int(sys.argv[1])
            goal_reward = int(sys.argv[2])

            user_goal = ast.literal_eval(sys.argv[3])
            goal_point = geometry_helpers.Point(user_goal[0], user_goal[1])

            user_init = ast.literal_eval(sys.argv[4])
            initial_location = geometry_helpers.Point(user_init[0], user_init[1])
        except (ValueError, TypeError) as e:
            print('Usage: python -m learningagent.simulation number_of_turns goal_reward goal_point initial_point')
            print('Please enter the following for inputs:')
            print('number_of_turns:  integer > 0        (ex: 250)')
            print('goal_reward:      integer > 0        (ex: 1000)')
            print('goal_point:       Tuple of integers  (ex: "(34, 22)")')
            print('initial_location: Tuple of integers  (ex: "(5, 5)")')
            sys.exit(1)


    visible_obstacles = environment_details.visible_obstacles
    x_bounds = environment_details.x_bounds
    y_bounds = environment_details.y_bounds

    run_simulation(number_of_turns, goal_point, goal_reward, initial_location,
                   visible_obstacles, x_bounds, y_bounds)

