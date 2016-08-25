import unittest

import geometry_helpers
import environment_details
import agent
import agent_percepts as percepts

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
