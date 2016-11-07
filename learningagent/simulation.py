# Module to simulate an agent's movements in the environment

import sys
import ast

from learningagent import geometry_helpers as geom
from learningagent import environment_details as env
from learningagent import agent_percepts as percepts
from learningagent import agent


def agent_reached_goal(sim_agent, goal_reward):
    """
    Award goal_points to the agent for reaching the goal
    and reset it to a new starting point.
    After resetting the agent, return the agent's true current location.
    """
    sim_agent.belief_state = agent.State(
        percepts.get_new_position(env.visible_obstacles, env.x_bounds, env.y_bounds), 1)
    sim_agent.belief_history = []
    sim_agent.prev_state = agent.State(None, 1)
    sim_agent.score += goal_reward
    sim_agent.reached_goal = True

    actual_location = sim_agent.belief_state.location

    print('Resetting agent to point {}'.format(sim_agent.belief_state.location))
    print('Agent score: {}'.format(sim_agent.score))

    return actual_location


def sim_perform_action(sim_agent, actual_location):
    """
    Given the agent's true location, let the agent perform its intended
    action and return the true results of the action.
    """
    print('Agent believes it is attempting to reach point {}'.format(
        sim_agent.prev_action))

    relative_move = geom.Point(
        (sim_agent.prev_action.x - sim_agent.belief_state.location.x),
        (sim_agent.prev_action.y - sim_agent.belief_state.location.y))

    actual_target = geom.Point(
        (relative_move.x + actual_location.x),
        (relative_move.y + actual_location.y))

    print('Agent actually attempting to reach point {}'.format(actual_target))
    actual_location = percepts.perform_action(sim_agent.prev_action)

    return actual_location


def sim_agent_action(sim_agent, actual_location):
    """
    Allow agent to update its location based on the visible vertices
    relative to its current position and update its score accordingly.
    """
    relative_verts = percepts.vertices_relative_to_agent(
        percepts.visible_vertices(actual_location, env.visible_obstacles),
        actual_location)
    initial_locations = percepts.get_locations(relative_verts, env.visible_obstacles)

    sim_agent.update_agent_location(initial_locations)

    sim_agent.score -= geom.distance(sim_agent.prev_state.location, actual_location)


def run_simulation(number_of_turns, goal_point, goal_reward, initial_location):
    """
    Given the number of turns allowed, run the simulation for the agent
    navigating a maze of polygons.
    """
    sim_agent = agent.Agent(goal_point, agent.State(initial_location, 1),
                            env.visible_obstacles)
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
            actual_location = agent_reached_goal(sim_agent, goal_reward)
            continue

        actual_location = sim_perform_action(sim_agent, actual_location)

        sim_agent_action(sim_agent, actual_location)

        print('Agent now at point {}'.format(actual_location))
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
        goal_point = geom.Point(34, 22)
        initial_location = geom.Point(5, 5)
    else:
        try:
            number_of_turns = int(sys.argv[1])
            goal_reward = int(sys.argv[2])

            user_goal = ast.literal_eval(sys.argv[3])
            goal_point = geom.Point(user_goal[0], user_goal[1])

            user_init = ast.literal_eval(sys.argv[4])
            initial_location = geom.Point(user_init[0], user_init[1])
        except (ValueError, TypeError) as e:
            print('Usage: python -m learningagent.simulation number_of_turns goal_reward goal_point initial_point')
            print('Please enter the following for inputs:')
            print('number_of_turns:  integer > 0        (ex: 250)')
            print('goal_reward:      integer > 0        (ex: 1000)')
            print('goal_point:       Tuple of integers  (ex: "(34, 22)")')
            print('initial_location: Tuple of integers  (ex: "(5, 5)")')
            sys.exit(1)


    run_simulation(number_of_turns, goal_point, goal_reward, initial_location)
