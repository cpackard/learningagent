import unittest
import geometry_helpers
import environment_details
import robot_maze

class TestRobotMaze(unittest.TestCase):

    def test_line_is_unblocked(self):
        visible_obstacles = environment_details.visible_obstacles

        # Test line from open space to polygon
        p = geometry_helpers.Point(33, 8)
        r = geometry_helpers.Point(32, 11)

        self.assertTrue(robot_maze.line_is_unblocked(p, r, visible_obstacles))

        # Test line from polygon to polygon
        p = geometry_helpers.Point(29, 12)
        r = geometry_helpers.Point(32, 11)

        self.assertTrue(robot_maze.line_is_unblocked(p, r, visible_obstacles))

        # Test line along the side of a polygon
        p = geometry_helpers.Point(32, 11)
        r = geometry_helpers.Point(29, 21)

        self.assertTrue(robot_maze.line_is_unblocked(p, r, visible_obstacles))

        # Test line going through obstacle
        p = geometry_helpers.Point(35, 21)
        r = geometry_helpers.Point(29, 21)

        self.assertFalse(robot_maze.line_is_unblocked(p, r, visible_obstacles))

        # Test line going through obstacle, along edge of obstacle
        p = geometry_helpers.Point(10, 14)
        r = geometry_helpers.Point(16, 14)

        self.assertFalse(robot_maze.line_is_unblocked(p, r, visible_obstacles))


    def test_visible_vertices(self):
        visible_obstacles = environment_details.visible_obstacles

        # Test upper-right corner (open space)
        p = geometry_helpers.Point(34, 22)

        self.assertEqual(set([geometry_helpers.Point(32, 23), geometry_helpers.Point(35, 21)]),
                         set(robot_maze.visible_vertices(p, visible_obstacles)))

        # Test lower-left corner (open space)
        p = geometry_helpers.Point(5, 1)

        self.assertEqual(set([geometry_helpers.Point(5, 20), geometry_helpers.Point(6, 2),
                              geometry_helpers.Point(6, 10), geometry_helpers.Point(18, 2)]),
                         set(robot_maze.visible_vertices(p, visible_obstacles)))


        # Test bottom-middle (open space)
        p = geometry_helpers.Point(23, 8)

        self.assertEqual(set([geometry_helpers.Point(18, 2), geometry_helpers.Point(19, 7),
                              geometry_helpers.Point(23, 9), geometry_helpers.Point(27, 11),
                              geometry_helpers.Point(29, 12), geometry_helpers.Point(29, 3),
                              geometry_helpers.Point(26, 9), geometry_helpers.Point(26, 6)]),
                         set(robot_maze.visible_vertices(p, visible_obstacles)))

        # Test bottom-middle (on triangle)
        p = geometry_helpers.Point(23, 9)

        self.assertEqual(set([geometry_helpers.Point(18, 2), geometry_helpers.Point(19, 7),
                              geometry_helpers.Point(19, 12), geometry_helpers.Point(26, 6),
                              geometry_helpers.Point(16, 17), geometry_helpers.Point(22, 11),
                              geometry_helpers.Point(26, 9), geometry_helpers.Point(27, 11)]),
                         set(robot_maze.visible_vertices(p, visible_obstacles)))


        # Test far-right vertex of pentagon
        p = geometry_helpers.Point(12, 19)

        self.assertEqual(set([geometry_helpers.Point(8.5, 23), geometry_helpers.Point(14.5, 21),
                              geometry_helpers.Point(10, 14), geometry_helpers.Point(13, 14)]),
                         set(robot_maze.visible_vertices(p, visible_obstacles)))

        # Test bottom of quadrilateral (on vertex)
        p = geometry_helpers.Point(32, 11)

        self.assertEqual(set([geometry_helpers.Point(29, 12), geometry_helpers.Point(32, 9),
                              geometry_helpers.Point(27, 23), geometry_helpers.Point(29, 21),
                              geometry_helpers.Point(35, 21)]),
                         set(robot_maze.visible_vertices(p, visible_obstacles)))

        # Test upper-left corner of rectangle2
        p = geometry_helpers.Point(22, 23)

        self.assertEqual(set([geometry_helpers.Point(27, 23), geometry_helpers.Point(22, 11),
                              geometry_helpers.Point(18.5, 23), geometry_helpers.Point(21, 20),
                              geometry_helpers.Point(18, 10), geometry_helpers.Point(19, 12)]),
                         set(robot_maze.visible_vertices(p, visible_obstacles)))

        # Test top of triangle1 (on triangle)
        p = geometry_helpers.Point(14.5, 21)

        self.assertEqual(set([geometry_helpers.Point(13, 14), geometry_helpers.Point(16, 14),
                              geometry_helpers.Point(18.5, 23), geometry_helpers.Point(16.5, 21.5),
                              geometry_helpers.Point(10, 14), geometry_helpers.Point(16, 17),
                              geometry_helpers.Point(12, 19), geometry_helpers.Point(8.5, 23),
                              geometry_helpers.Point(19, 7), geometry_helpers.Point(18, 10)]),
                         set(robot_maze.visible_vertices(p, visible_obstacles)))


    def test_vertices_relative_to_agent(self):
        visible_obstacles = environment_details.visible_obstacles

        # Test arbitrary points
        p = geometry_helpers.Point(5, 5)
        vertices = [geometry_helpers.Point(7, 7), geometry_helpers.Point(4, 6),
                    geometry_helpers.Point(5, 3), geometry_helpers.Point(1, 5),
                    geometry_helpers.Point(7, 0)]


        self.assertEqual(set([geometry_helpers.Point(2, 2), geometry_helpers.Point(-1, 1),
                              geometry_helpers.Point(0, -2), geometry_helpers.Point(-4, 0),
                              geometry_helpers.Point(2, -5)]),
                         set(robot_maze.vertices_relative_to_agent(vertices, p)))

        # Test open space in maze near top-left corner
        p = geometry_helpers.Point(34, 22)
        vertices = robot_maze.visible_vertices(p, visible_obstacles)

        self.assertEqual(set([geometry_helpers.Point(-2, 1), geometry_helpers.Point(1, -1)]),
                         set(robot_maze.vertices_relative_to_agent(vertices, p)))

        # Test on hexagon vertex in maze
        p = geometry_helpers.Point(32, 6)
        vertices = robot_maze.visible_vertices(p, visible_obstacles)

        self.assertEqual(set([geometry_helpers.Point(-3, -3), geometry_helpers.Point(0, 3),
                              geometry_helpers.Point(3, 15)]),
                         set(robot_maze.vertices_relative_to_agent(vertices, p)))

        # Test open space in maze in between pentagon and triangle
        p = geometry_helpers.Point(13, 19)
        vertices = robot_maze.visible_vertices(p, visible_obstacles)

        self.assertEqual(set([geometry_helpers.Point(-1, 0), geometry_helpers.Point(-3, -5),
                              geometry_helpers.Point(-4.5, 4), geometry_helpers.Point(0, -5),
                              geometry_helpers.Point(1.5, 2)]),
                         set(robot_maze.vertices_relative_to_agent(vertices, p)))



    def test_get_locations(self):
        visible_obstacles = environment_details.visible_obstacles

        # Test upper-right corner (open space)
        p = geometry_helpers.Point(34, 22)
        vertex_list = robot_maze.visible_vertices(p, visible_obstacles)
        agent_vertex_list = robot_maze.vertices_relative_to_agent(vertex_list, p)

        self.assertEqual(set([geometry_helpers.Point(34, 22), geometry_helpers.Point(18, 13)]),
                         set(robot_maze.get_locations(agent_vertex_list, visible_obstacles)))

        # Test upper-left corner above pentagon (open space)
        p = geometry_helpers.Point(12, 21)
        vertex_list = robot_maze.visible_vertices(p, visible_obstacles)
        agent_vertex_list = robot_maze.vertices_relative_to_agent(vertex_list, p)

        self.assertEqual(set([geometry_helpers.Point(12, 21)]),
                         set(robot_maze.get_locations(agent_vertex_list, visible_obstacles)))

        # Test lower-right corner of hexagon (on polygon)
        p = geometry_helpers.Point(32, 6)
        vertex_list = robot_maze.visible_vertices(p, visible_obstacles)
        agent_vertex_list = robot_maze.vertices_relative_to_agent(vertex_list, p)

        self.assertEqual(set([geometry_helpers.Point(32, 6)]),
                         set(robot_maze.get_locations(agent_vertex_list, visible_obstacles)))

        # Test open space in middle
        p = geometry_helpers.Point(5, 20)
        vertex_list = robot_maze.visible_vertices(p, visible_obstacles)
        agent_vertex_list = robot_maze.vertices_relative_to_agent(vertex_list, p)

        self.assertEqual(set([geometry_helpers.Point(5, 20)]),
                         set(robot_maze.get_locations(agent_vertex_list, visible_obstacles)))

        # Test far-left side of pentagon (on polygon)
        p = geometry_helpers.Point(20, 16)
        vertex_list = robot_maze.visible_vertices(p, visible_obstacles)
        agent_vertex_list = robot_maze.vertices_relative_to_agent(vertex_list, p)

        self.assertEqual(set([geometry_helpers.Point(20, 16)]),
                         set(robot_maze.get_locations(agent_vertex_list, visible_obstacles)))


    def test_goal_test(self):
        goal_point = geometry_helpers.Point(29, 17)

        p = geometry_helpers.Point(30, 12)
        self.assertFalse(robot_maze.goal_test(p, goal_point))

        q = geometry_helpers.Point(29, 17)
        self.assertTrue(robot_maze.goal_test(q, goal_point))


    def test_heuristic(self):
        # For now these tests are slightly redundant since heuristic is just
        # a straight line, but since the heuristic could change in the
        # future it's better to have them.

        # Test straight horizontal line
        p1 = geometry_helpers.Point(1, 1)
        goal = geometry_helpers.Point(4, 1)

        self.assertEqual(3, robot_maze.heuristic(p1, goal))

        # Test straight vertical line
        p1 = geometry_helpers.Point(1, 1)
        goal = geometry_helpers.Point(1, 4)

        self.assertEqual(3, robot_maze.heuristic(p1, goal))

        # Test positive horizontal line
        p1 = geometry_helpers.Point(1, 1)
        goal = geometry_helpers.Point(4, 4)

        self.assertEqual(4.242640687119285, robot_maze.heuristic(p1, goal))

        # Test negative horizontal line
        p1 = geometry_helpers.Point(1, 1)
        goal = geometry_helpers.Point(-2, -2)

        self.assertEqual(4.242640687119285, robot_maze.heuristic(p1, goal))


    def test_heuristic(self):
        # For now these tests are also redundant since actual_cost is just
        # a straight line, but this could also change in the future, so
        # we keep them.

        # Test straight horizontal line
        s1 = geometry_helpers.Point(1, 1)
        s2 = geometry_helpers.Point(4, 1)

        self.assertEqual(3, robot_maze.actual_cost(s1, s2, s2))

        # Test straight vertical line
        s1 = geometry_helpers.Point(1, 1)
        s2 = geometry_helpers.Point(1, 4)

        self.assertEqual(3, robot_maze.actual_cost(s1, s2, s2))

        # Test positive horizontal line
        s1 = geometry_helpers.Point(1, 1)
        s2 = geometry_helpers.Point(4, 4)

        self.assertEqual(4.242640687119285, robot_maze.actual_cost(s1, s2, s2))

        # Test negative horizontal line
        s1 = geometry_helpers.Point(1, 1)
        s2 = geometry_helpers.Point(-2, -2)

        self.assertEqual(4.242640687119285, robot_maze.actual_cost(s1, s2, s2))


    def test_actions(self):
        # Redundant since this only makes a call to visible_vertices, so only
        # test a subset of visible_vertices tests for a sanity check.

        # Test upper-right corner (open space)
        state = geometry_helpers.Point(34, 22)

        self.assertEqual(set([geometry_helpers.Point(32, 23), geometry_helpers.Point(35, 21)]),
                         set(robot_maze.actions(state)))

        # Test lower-left corner (open space)
        state = geometry_helpers.Point(5, 1)

        self.assertEqual(set([geometry_helpers.Point(5, 20), geometry_helpers.Point(6, 2),
                              geometry_helpers.Point(6, 10), geometry_helpers.Point(18, 2)]),
                         set(robot_maze.actions(state)))

        # Test far-right vertex of pentagon
        state = geometry_helpers.Point(12, 19)

        self.assertEqual(set([geometry_helpers.Point(8.5, 23), geometry_helpers.Point(14.5, 21),
                              geometry_helpers.Point(10, 14), geometry_helpers.Point(13, 14)]),
                         set(robot_maze.actions(state)))

        # Test top of triangle1 (on triangle)
        state = geometry_helpers.Point(14.5, 21)

        self.assertEqual(set([geometry_helpers.Point(13, 14), geometry_helpers.Point(16, 14),
                              geometry_helpers.Point(18.5, 23), geometry_helpers.Point(16.5, 21.5),
                              geometry_helpers.Point(10, 14), geometry_helpers.Point(16, 17),
                              geometry_helpers.Point(12, 19), geometry_helpers.Point(8.5, 23),
                              geometry_helpers.Point(19, 7), geometry_helpers.Point(18, 10)]),
                         set(robot_maze.actions(state)))


    def test_perform_action(self):
        # TODO Beef up these tests after we add random failure logic
        action = geometry_helpers.Point(1, 1)
        self.assertEqual(action, robot_maze.perform_action(action))


    def test_get_new_position(self):
        obstacles = environment_details.visible_obstacles
        x_bounds = environment_details.x_bounds
        y_bounds = environment_details.y_bounds

        # Since positions are randomly tested, get a good sample
        # to make sure method works as expected.
        for i in range(0, 100):
            new_position = robot_maze.get_new_position(obstacles,
                                                    x_bounds, y_bounds)

            self.assertTrue(0 <= new_position.x <= x_bounds)

            self.assertTrue(0 <= new_position.y <= y_bounds)

            self.assertFalse(geometry_helpers.point_in_any_obstacle(new_position, obstacles))


    def test_LRTA_star_cost(self):
        # Test without state
        prev_state = geometry_helpers.Point(1, 1)
        action = geometry_helpers.Point(3, 1)
        state = None
        cost_estimates = {geometry_helpers.Point(3, 1): 2, geometry_helpers.Point(1, 1): 4}
        goal_point = geometry_helpers.Point(5, 1)

        self.assertEqual(4, robot_maze.LRTA_star_cost(prev_state, action,
                                                      state, cost_estimates,
                                                      goal_point))

        # Test with state
        state = geometry_helpers.Point(3, 1)
        self.assertEqual(4, robot_maze.LRTA_star_cost(prev_state, action,
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

        self.assertEqual((None, None, {}, {}), robot_maze.LRTA_star_agent(
            agent_location, goal_point, visible_obstacles, result,
            cost_estimates, prev_state, prev_action))


        # Update goal point so agent has to plan an action
        goal_point = geometry_helpers.Point(34, 22)

        prev_action, prev_state, result, cost_estimates = robot_maze.LRTA_star_agent(
            agent_location, goal_point, visible_obstacles, result,
            cost_estimates, prev_state, prev_action)

        self.assertEqual(cost_estimates[prev_state],
                         robot_maze.heuristic(prev_state, goal_point))

        self.assertFalse((None, None) in result)

        self.assertFalse(None in cost_estimates)

        self.assertEqual(geometry_helpers.Point(6, 2), prev_action)

        self.assertEqual(geometry_helpers.Point(4, 5), prev_state)

        agent_location = robot_maze.perform_action(prev_action)

        prev_action, prev_state, result, cost_estimates = robot_maze.LRTA_star_agent(
            agent_location, goal_point, visible_obstacles, result,
            cost_estimates, prev_state, prev_action)

        self.assertEqual(prev_state,
                         result[(geometry_helpers.Point(4, 5), geometry_helpers.Point(6, 2))])

        self.assertEqual(34.48187929913333, cost_estimates[geometry_helpers.Point(4, 5)])

        self.assertEqual(geometry_helpers.Point(18, 2), prev_action)

        self.assertEqual(geometry_helpers.Point(6, 2), prev_state)


    def test_run_simulation(self):
        number_of_turns = 1000
        goal_point = geometry_helpers.Point(34, 22)
        goal_reward = 1000
        initial_location = geometry_helpers.Point(5, 5)
        visible_obstacles = environment_details.visible_obstacles
        x_bounds = environment_details.x_bounds
        y_bounds = environment_details.y_bounds

        self.assertTrue(robot_maze.run_simulation(
            number_of_turns, goal_point, goal_reward,
            initial_location, visible_obstacles,
            x_bounds, y_bounds))


if __name__ == "__main__":
    unittest.main()
