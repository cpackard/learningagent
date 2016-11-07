import unittest

from learningagent import geometry_helpers
from learningagent import environment_details
from learningagent import agent
from learningagent import agent_percepts as percepts

class TestAgent(unittest.TestCase):

    def test_goal_test(self):
        goal_point = geometry_helpers.Point(29, 17)
        sim_agent = agent.Agent(goal_point, None, None)

        p = geometry_helpers.Point(30, 12)
        self.assertFalse(sim_agent.goal_test(p))

        q = geometry_helpers.Point(29, 17)
        self.assertTrue(sim_agent.goal_test(q))


    def test_LRTA_star_cost(self):
        # Test without state
        prev_state = geometry_helpers.Point(1, 1)
        action = geometry_helpers.Point(3, 1)
        state = None
        cost_estimates = {geometry_helpers.Point(3, 1): 2,
                          geometry_helpers.Point(1, 1): 4}
        goal_point = geometry_helpers.Point(5, 1)
        a = agent.Agent(goal_point, None, None)

        self.assertEqual(4, a.LRTA_star_cost(prev_state, action,
                                                 state, cost_estimates,
                                                 goal_point))

        # Test with state
        state = geometry_helpers.Point(3, 1)
        self.assertEqual(4, a.LRTA_star_cost(prev_state, action,
                                                 state, cost_estimates,
                                                 goal_point))


    def test_prev_state_probability(self):
        # Test prev_state not visible from a
        a = geometry_helpers.Point(6, 2)
        goal = geometry_helpers.Point(34, 22)
        visible_obstacles = environment_details.visible_obstacles

        sim_agent = agent.Agent(goal, None, visible_obstacles)
        sim_agent.prev_result = {}
        sim_agent.prev_cost_estimates = {}
        sim_agent.prev_state = agent.State(geometry_helpers.Point(32, 23), 1)

        self.assertEqual(0, sim_agent._prev_state_probability(a))

        # Test location A not an optimal move from prev_state
        a = geometry_helpers.Point(5, 20)
        sim_agent.prev_state = agent.State(geometry_helpers.Point(6, 2), 1)
        sim_agent.prev_result[(sim_agent.prev_state.location, a)] = a
        sim_agent.prev_cost_estimates[a] = 50

        self.assertEqual(0, sim_agent._prev_state_probability(a))

        # Test location A is visible from prev_state, and is an
        # optimal move from prev_state
        sim_agent.prev_result = {}
        sim_agent.prev_cost_estimates = {}
        sim_agent.prev_state = agent.State(geometry_helpers.Point(6, 2), 1)
        a = geometry_helpers.Point(6, 10)

        self.assertEqual(0.375, sim_agent._prev_state_probability(a))


    def test_current_state_probability(self):
        a = geometry_helpers.Point(6, 10)

        obstacles = environment_details.visible_obstacles
        goal = geometry_helpers.Point(34, 22)

        sim_agent = agent.Agent(goal, None, obstacles)
        sim_agent.prev_state = agent.State(geometry_helpers.Point(6, 2), 1)
        sim_agent.prev_cost_estimates = {}
        sim_agent.prev_result = {}

        self.assertEqual(0.375, sim_agent._current_state_probability(a))


    def test_refine_possible_locations(self):
        # Sanity check, no possible locations
        obstacles = environment_details.visible_obstacles
        possible_locations = []
        goal = geometry_helpers.Point(34, 22)

        sim_agent = agent.Agent(goal, None, obstacles)
        sim_agent.prev_state = agent.State(geometry_helpers.Point(6, 2), 1)
        sim_agent.prev_result = {}
        sim_agent.prev_cost_estimates = {}

        self.assertEqual([], sim_agent._refine_possible_locations(
            possible_locations))

        # prev_state not visible from any of the possible locations
        possible_locations = [geometry_helpers.Point(35, 21)]

        self.assertEqual([], sim_agent._refine_possible_locations(
            possible_locations))

        # only point (6, 10) from possible_locations is visible
        possible_locations = [geometry_helpers.Point(6, 10),
                              geometry_helpers.Point(35, 21)]

        self.assertEqual([agent.State(geometry_helpers.Point(6, 10), 0.375)],
                         sim_agent._refine_possible_locations(possible_locations))

        # Sanity check, a refined list of locations shouldn't be
        # updated again.
        possible_locations = [geometry_helpers.Point(6, 10)]

        self.assertEqual([agent.State(geometry_helpers.Point(6, 10), 0.375)],
                         sim_agent._refine_possible_locations(possible_locations))


    def test_update_certain(self):
        possible_locations = [geometry_helpers.Point(6, 10)]

        sim_agent = agent.Agent(None, None, None)
        sim_agent.prev_state = agent.State(geometry_helpers.Point(6, 2), 0.7)
        sim_agent.prev_action = geometry_helpers.Point(6, 10)
        sim_agent.belief_history = [[agent.State(geometry_helpers.Point(5, 3),
                                                 0.8)]]


        # Agent is uncertain of past location, update previous belief states
        sim_agent._update_certain(possible_locations)

        self.assertEqual(geometry_helpers.Point(6, 10),
                         sim_agent.belief_state.location)
        # TODO This will change when we add update_previous_belief logic
        # Add logic to check that agent.results were updated
        self.assertEqual([], sim_agent.belief_history)

        # Agent is certain of past location, and ended at
        # intended destination.
        sim_agent.prev_state = agent.State(geometry_helpers.Point(6, 2), 1)

        sim_agent._update_certain(possible_locations)

        self.assertEqual(geometry_helpers.Point(6, 10),
                         sim_agent.belief_state.location)
        self.assertEqual([], sim_agent.belief_history)

        # Agent is certain of past location, but did not end up at
        # intended destination.
        # TODO Add logic to check that recovery scheme worked
        sim_agent.prev_action = geometry_helpers.Point(18, 2)

        sim_agent._update_certain(possible_locations)

        self.assertEqual(geometry_helpers.Point(6, 10),
                         sim_agent.belief_state.location)
        # TODO This will change when we add update_previous_belief logic
        # Add logic to check that agent.results were updated
        self.assertEqual([], sim_agent.belief_history)


    def test_update_uncertain(self):
        possible_locations = [geometry_helpers.Point(6, 10),
                              geometry_helpers.Point(35, 21)]
        obstacles = environment_details.visible_obstacles
        goal = geometry_helpers.Point(34, 22)

        sim_agent = agent.Agent(goal, None, obstacles)
        sim_agent.prev_state = agent.State(geometry_helpers.Point(6, 2), 1)
        sim_agent.prev_action = geometry_helpers.Point(6, 10)
        sim_agent.prev_result = {}
        sim_agent.prev_cost_estimates = {}
        sim_agent.belief_history = [[agent.State(geometry_helpers.Point(5, 3),
                                                 0.8)]]

        # Agent is initially uncertain of current location
        # but is able to refine it
        sim_agent._update_uncertain(possible_locations)

        self.assertEqual(geometry_helpers.Point(6, 10),
                         sim_agent.belief_state.location)
        self.assertEqual([], sim_agent.belief_history)

        # After refining the current possible locations there are no
        # possible results, so agent searches back in its history to
        # find a previous state that fits with the current possibilities.
        sim_agent.belief_history = [[agent.State(geometry_helpers.Point(35, 21), 0.8),
                                     agent.State(geometry_helpers.Point(6, 2), 0.2)]]
        sim_agent.prev_state = agent.State(geometry_helpers.Point(35, 21), 1)

        sim_agent._update_uncertain(possible_locations)

        self.assertEqual(geometry_helpers.Point(6, 10),
                         sim_agent.belief_state.location)
        self.assertEqual([], sim_agent.belief_history)

        # Agent once again has to search back in its history to find a
        # compatible previous state, but this time is still not
        # certain of its current location afterwards
        sim_agent.belief_history = [[agent.State(geometry_helpers.Point(35, 21), 0.8),
                                     agent.State(geometry_helpers.Point(6, 2), 0.2)]]
        sim_agent.prev_state = agent.State(geometry_helpers.Point(35, 21), 1)
        possible_locations.append(geometry_helpers.Point(18, 2))

        sim_agent._update_uncertain(possible_locations)

        self.assertEqual(geometry_helpers.Point(6, 10),
                         sim_agent.belief_state.location)
        self.assertEqual([[agent.State(geometry_helpers.Point(35, 21), 0.8),
                           agent.State(geometry_helpers.Point(6, 2), 0.2)],
                          [agent.State(geometry_helpers.Point(6, 10), 0.375),
                           agent.State(geometry_helpers.Point(18, 2), 0.3)]],
                         sim_agent.belief_history)


    def test_update_agent_location(self):
        possible_locations = [geometry_helpers.Point(6, 10)]
        obstacles = environment_details.visible_obstacles
        goal = geometry_helpers.Point(34, 22)

        sim_agent = agent.Agent(goal, None, obstacles)
        sim_agent.prev_state = None
        sim_agent.prev_action = geometry_helpers.Point(6, 10)
        sim_agent.prev_result = {}
        sim_agent.prev_cost_estimates = {}
        sim_agent.belief_history = []

        # Agent is certain of its current location
        sim_agent.update_agent_location(possible_locations)

        self.assertEqual(geometry_helpers.Point(6, 10),
                         sim_agent.belief_state.location)
        self.assertEqual([], sim_agent.belief_history)

        # Agent is uncertain of current location and does not
        # have a previous location for context
        possible_locations.append(geometry_helpers.Point(18, 2))

        sim_agent.update_agent_location(possible_locations)

        self.assertEqual(geometry_helpers.Point(6, 10),
                         sim_agent.belief_state.location)
        self.assertEqual([[agent.State(geometry_helpers.Point(6, 10), 0.5),
                           agent.State(geometry_helpers.Point(18, 2), 0.5)]],
                         sim_agent.belief_history)

        # Agent is uncertain of current location but
        # has previous state for context
        sim_agent.prev_state = agent.State(geometry_helpers.Point(6, 2), 0.7)
        sim_agent.belief_history = []

        sim_agent.update_agent_location(possible_locations)

        self.assertEqual(geometry_helpers.Point(6, 10),
                         sim_agent.belief_state.location)
        self.assertEqual([[agent.State(geometry_helpers.Point(6, 10), 0.5),
                           agent.State(geometry_helpers.Point(18, 2), 0.5)]],
                         sim_agent.belief_history)


    def test_LRTA_star_agent(self):
        visible_obstacles = environment_details.visible_obstacles
        goal_point = geometry_helpers.Point(4, 5)
        agent_location = geometry_helpers.Point(4, 5)

        sim_agent = agent.Agent(goal_point, agent.State(agent_location, 1),
                                visible_obstacles)
        sim_agent.result = {}
        sim_agent.cost_estimates = {}
        sim_agent.prev_state = agent.State(None, 1)
        sim_agent.prev_action = None

        sim_agent.LRTA_star_agent()

        self.assertEqual(None, sim_agent.prev_action)
        self.assertEqual(None, sim_agent.prev_state.location)
        self.assertEqual({}, sim_agent.result)
        self.assertEqual({}, sim_agent.cost_estimates)

        # Update goal point so agent has to plan an action
        sim_agent.goal = geometry_helpers.Point(34, 22)

        sim_agent.LRTA_star_agent()

        self.assertEqual(sim_agent.cost_estimates[sim_agent.prev_state.location],
                         percepts.heuristic(sim_agent.prev_state.location, sim_agent.goal))

        self.assertFalse((None, None) in sim_agent.result)

        self.assertFalse(None in sim_agent.cost_estimates)

        self.assertEqual(geometry_helpers.Point(6, 2), sim_agent.prev_action)

        self.assertEqual(geometry_helpers.Point(4, 5), sim_agent.prev_state.location)

        sim_agent.belief_state = agent.State(
            percepts.perform_action(sim_agent.prev_action), 1)

        sim_agent.LRTA_star_agent()

        self.assertEqual(sim_agent.prev_state.location,
                         sim_agent.result[(geometry_helpers.Point(4, 5),
                                           geometry_helpers.Point(6, 2))])

        self.assertEqual(34.48187929913333,
                         sim_agent.cost_estimates[geometry_helpers.Point(4, 5)])

        self.assertEqual(geometry_helpers.Point(18, 2), sim_agent.prev_action)

        self.assertEqual(geometry_helpers.Point(6, 2), sim_agent.prev_state.location)

if __name__ == "__main__":
    unittest.main()
