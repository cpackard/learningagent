import unittest

from learningagent import geometry_helpers
from learningagent import environment_details
from learningagent import agent
from learningagent import agent_percepts as percepts

class TestAgent(unittest.TestCase):

    def test_goal_test(self):
        goal_point = geometry_helpers.Point(29, 17)

        p = geometry_helpers.Point(30, 12)
        self.assertFalse(agent.goal_test(p, goal_point))

        q = geometry_helpers.Point(29, 17)
        self.assertTrue(agent.goal_test(q, goal_point))


    def test_LRTA_star_cost(self):
        # Test without state
        prev_state = geometry_helpers.Point(1, 1)
        action = geometry_helpers.Point(3, 1)
        state = None
        cost_estimates = {geometry_helpers.Point(3, 1): 2, geometry_helpers.Point(1, 1): 4}
        goal_point = geometry_helpers.Point(5, 1)

        self.assertEqual(4, agent.LRTA_star_cost(prev_state, action,
                                                      state, cost_estimates,
                                                      goal_point))

        # Test with state
        state = geometry_helpers.Point(3, 1)
        self.assertEqual(4, agent.LRTA_star_cost(prev_state, action,
                                                      state, cost_estimates,
                                                      goal_point))


    def test_prev_state_probability(self):
        # Test prev_state not visible from a
        a = geometry_helpers.Point(6, 2)
        prev_state = geometry_helpers.Point(32, 23)
        visible_obstacles = environment_details.visible_obstacles
        prev_result = {}
        prev_cost_estimates = {}
        goal = geometry_helpers.Point(34, 22)

        self.assertEqual(0, agent.prev_state_probability(
            a, prev_state, prev_result, visible_obstacles,
            prev_cost_estimates, goal))

        # Test location A not an optimal move from prev_state
        a = geometry_helpers.Point(5, 20)
        prev_state = geometry_helpers.Point(6, 2)
        prev_result[(prev_state, a)] = a
        prev_cost_estimates[a] = 50

        self.assertEqual(0, agent.prev_state_probability(
            a, prev_state, prev_result, visible_obstacles,
            prev_cost_estimates, goal))

        # Test location A is visible from prev_state, and is an
        # optimal move from prev_state
        prev_result = {}
        prev_cost_estimates = {}
        prev_state = geometry_helpers.Point(6, 2)
        a = geometry_helpers.Point(6, 10)

        self.assertEqual(0.375, agent.prev_state_probability(
            a, prev_state, prev_result, visible_obstacles,
            prev_cost_estimates, goal))


    def test_current_state_probability(self):
        a = geometry_helpers.Point(6, 10)
        prev_state = geometry_helpers.Point(6, 2)
        obstacles = environment_details.visible_obstacles
        prev_result = {}
        prev_cost_estimates = {}
        goal = geometry_helpers.Point(34, 22)

        # b = geometry_helpers.Point(5, 20)
        # prev_result[(prev_state, b)] = b
        # prev_cost_estimates[b] = 20

        self.assertEqual(0.375, agent.current_state_probability(
            a, prev_state, prev_result,
            obstacles, prev_cost_estimates, goal))


    def test_refine_possible_locations(self):
        # Sanity check, no possible locations
        prev_state = agent.State(geometry_helpers.Point(6, 2), 0.8)
        obstacles = environment_details.visible_obstacles
        prev_result = {}
        prev_cost_estimates = {}
        possible_locations = []
        goal = geometry_helpers.Point(34, 22)

        self.assertEqual([], agent.refine_possible_locations(
            prev_state, obstacles, prev_result,
            prev_cost_estimates, possible_locations, goal))

        # prev_state not visible from any of the possible locations
        possible_locations = [geometry_helpers.Point(35, 21)]

        self.assertEqual([], agent.refine_possible_locations(
            prev_state, obstacles, prev_result,
            prev_cost_estimates, possible_locations, goal))

        # only point (6, 10) from possible_locations is visible
        possible_locations = [geometry_helpers.Point(6, 10),
                              geometry_helpers.Point(35, 21)]

        self.assertEqual([agent.State(geometry_helpers.Point(6, 10), 0.375)],
                         agent.refine_possible_locations(
                             prev_state, obstacles, prev_result,
                             prev_cost_estimates, possible_locations, goal))

        # Sanity check, a refined list of locations shouldn't be
        # updated again.
        possible_locations = [geometry_helpers.Point(6, 10)]

        self.assertEqual([agent.State(geometry_helpers.Point(6, 10), 0.375)],
                         agent.refine_possible_locations(
                             prev_state, obstacles, prev_result,
                             prev_cost_estimates, possible_locations, goal))



    def test_update_certain(self):
        possible_locations = [geometry_helpers.Point(6, 10)]
        prev_state = agent.State(geometry_helpers.Point(6, 2), 0.7)
        prev_action = geometry_helpers.Point(6, 10)
        belief_history = [[agent.State(geometry_helpers.Point(5, 3), 0.8)]]

        # Agent is uncertain of past location, update previous belief states
        agent_belief_state, belief_history = agent.update_certain(
            possible_locations, prev_state, prev_action, belief_history)

        self.assertEqual(geometry_helpers.Point(6, 10),
                         agent_belief_state.location)
        # TODO This will change when we add update_previous_belief logic
        # Add logic to check that agent.results were updated
        self.assertEqual([], belief_history)

        # Agent is certain of past location, and ended at
        # intended destination.
        prev_state = agent.State(geometry_helpers.Point(6, 2), 1)

        agent_belief_state, belief_history = agent.update_certain(
            possible_locations, prev_state, prev_action, belief_history)

        self.assertEqual(geometry_helpers.Point(6, 10),
                         agent_belief_state.location)
        self.assertEqual([], belief_history)

        # Agent is certain of past location, but did not end up at
        # intended destination.
        # TODO Add logic to check that recovery scheme worked
        prev_action = geometry_helpers.Point(18, 2)

        agent_belief_state, belief_history = agent.update_certain(
            possible_locations, prev_state, prev_action, belief_history)

        self.assertEqual(geometry_helpers.Point(6, 10),
                         agent_belief_state.location)
        # TODO This will change when we add update_previous_belief logic
        # Add logic to check that agent.results were updated
        self.assertEqual([], belief_history)


    def test_update_uncertain(self):
        possible_locations = [geometry_helpers.Point(6, 10),
                              geometry_helpers.Point(35, 21)]
        prev_state = agent.State(geometry_helpers.Point(6, 2), 0.7)
        prev_action = geometry_helpers.Point(6, 10)
        obstacles = environment_details.visible_obstacles
        prev_result = {}
        prev_cost_estimates = {}
        goal = geometry_helpers.Point(34, 22)
        belief_history = [[agent.State(geometry_helpers.Point(5, 3), 0.8)]]

        # Agent is initially uncertain of current location
        # but is able to refine it
        agent_belief_state, belief_history = agent.update_uncertain(
            possible_locations, prev_state, prev_action, belief_history,
            obstacles, prev_result, prev_cost_estimates, goal)

        self.assertEqual(geometry_helpers.Point(6, 10),
                         agent_belief_state.location)
        self.assertEqual([], belief_history)

        # After refining the current possible locations there are no
        # possible results, so agent searches back in its history to
        # find a previous state that fits with the current possibilities.
        belief_history = [[agent.State(geometry_helpers.Point(35, 21), 0.8),
                           agent.State(geometry_helpers.Point(6, 2), 0.2)]]
        prev_state = agent.State(geometry_helpers.Point(35, 21), 0.8)

        agent_belief_state, belief_history = agent.update_uncertain(
            possible_locations, prev_state, prev_action, belief_history,
            obstacles, prev_result, prev_cost_estimates, goal)

        self.assertEqual(geometry_helpers.Point(6, 10),
                         agent_belief_state.location)
        self.assertEqual([], belief_history)

        # Agent once again has to search back in its history to find a
        # compatible previous state, but this time is still not
        # certain of its current location afterwards
        belief_history = [[agent.State(geometry_helpers.Point(35, 21), 0.8),
                           agent.State(geometry_helpers.Point(6, 2), 0.2)]]
        prev_state = agent.State(geometry_helpers.Point(35, 21), 0.8)
        possible_locations.append(geometry_helpers.Point(18, 2))

        agent_belief_state, belief_history = agent.update_uncertain(
            possible_locations, prev_state, prev_action, belief_history,
            obstacles, prev_result, prev_cost_estimates, goal)

        self.assertEqual(geometry_helpers.Point(6, 10),
                         agent_belief_state.location)
        self.assertEqual([[agent.State(geometry_helpers.Point(35, 21), 0.8),
                           agent.State(geometry_helpers.Point(6, 2), 0.2)],
                          [agent.State(geometry_helpers.Point(6, 10), 0.375),
                           agent.State(geometry_helpers.Point(18, 2), 0.3)]],
                         belief_history)


    def test_update_agent_location(self):
        possible_locations = [geometry_helpers.Point(6, 10)]
        prev_state = None
        prev_action = geometry_helpers.Point(6, 10)
        obstacles = environment_details.visible_obstacles
        prev_result = {}
        prev_cost_estimates = {}
        goal = geometry_helpers.Point(34, 22)
        belief_history = []

        # Agent is certain of its current location
        agent_belief_state, belief_history = agent.update_agent_location(
            prev_state, possible_locations, belief_history, prev_action,
            obstacles, prev_result, prev_cost_estimates, goal)

        self.assertEqual(geometry_helpers.Point(6, 10),
                         agent_belief_state.location)
        self.assertEqual([], belief_history)

        # Agent is uncertain of current location and does not
        # have a previous location for context
        possible_locations.append(geometry_helpers.Point(18, 2))

        agent_belief_state, belief_history = agent.update_agent_location(
            prev_state, possible_locations, belief_history, prev_action,
            obstacles, prev_result, prev_cost_estimates, goal)

        self.assertEqual(geometry_helpers.Point(6, 10),
                         agent_belief_state.location)
        self.assertEqual([[agent.State(geometry_helpers.Point(6, 10), 0.5),
                           agent.State(geometry_helpers.Point(18, 2), 0.5)]],
                         belief_history)

        # Agent is uncertain of current location but
        # has previous state for context
        prev_state = agent.State(geometry_helpers.Point(6, 2), 0.7)
        belief_history = []

        agent_belief_state, belief_history = agent.update_agent_location(
            prev_state, possible_locations, belief_history, prev_action,
            obstacles, prev_result, prev_cost_estimates, goal)

        self.assertEqual(geometry_helpers.Point(6, 10),
                         agent_belief_state.location)
        self.assertEqual([[agent.State(geometry_helpers.Point(6, 10), 0.375),
                           agent.State(geometry_helpers.Point(18, 2), 0.3)]],
                         belief_history)


    def test_LRTA_star_agent(self):
        visible_obstacles = environment_details.visible_obstacles

        result = {}
        cost_estimates = {}
        prev_state = None
        prev_action = None
        goal_point = geometry_helpers.Point(4, 5)
        agent_location = geometry_helpers.Point(4, 5)

        self.assertEqual((None, None, {}, {}), agent.LRTA_star_agent(
            agent_location, goal_point, visible_obstacles, result,
            cost_estimates, prev_state, prev_action))


        # Update goal point so agent has to plan an action
        goal_point = geometry_helpers.Point(34, 22)

        prev_action, prev_state, result, cost_estimates = agent.LRTA_star_agent(
            agent_location, goal_point, visible_obstacles, result,
            cost_estimates, prev_state, prev_action)

        self.assertEqual(cost_estimates[prev_state],
                         percepts.heuristic(prev_state, goal_point))

        self.assertFalse((None, None) in result)

        self.assertFalse(None in cost_estimates)

        self.assertEqual(geometry_helpers.Point(6, 2), prev_action)

        self.assertEqual(geometry_helpers.Point(4, 5), prev_state)

        agent_location = percepts.perform_action(prev_action)

        prev_action, prev_state, result, cost_estimates = agent.LRTA_star_agent(
            agent_location, goal_point, visible_obstacles, result,
            cost_estimates, prev_state, prev_action)

        self.assertEqual(prev_state,
                         result[(geometry_helpers.Point(4, 5), geometry_helpers.Point(6, 2))])

        self.assertEqual(34.48187929913333, cost_estimates[geometry_helpers.Point(4, 5)])

        self.assertEqual(geometry_helpers.Point(18, 2), prev_action)

        self.assertEqual(geometry_helpers.Point(6, 2), prev_state)

if __name__ == "__main__":
    unittest.main()
