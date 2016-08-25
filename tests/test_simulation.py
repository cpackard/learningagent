import unittest

from learningagent import geometry_helpers
from learningagent import environment_details
from learningagent import simulation

class TestSimulation(unittest.TestCase):

    def test_run_simulation(self):
        number_of_turns = 1000
        goal_point = geometry_helpers.Point(34, 22)
        goal_reward = 1000
        initial_location = geometry_helpers.Point(5, 5)
        visible_obstacles = environment_details.visible_obstacles
        x_bounds = environment_details.x_bounds
        y_bounds = environment_details.y_bounds

        self.assertTrue(simulation.run_simulation(
            number_of_turns, goal_point, goal_reward,
            initial_location, visible_obstacles,
            x_bounds, y_bounds))


if __name__ == "__main__":
    unittest.main()
