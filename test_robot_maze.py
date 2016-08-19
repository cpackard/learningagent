import unittest
import lines
import obstacles
import robot_maze

class TestRobotMaze(unittest.TestCase):

    def test_line_is_unblocked(self):
        # Test line from open space to polygon
        p = lines.Point(33, 8)
        r = lines.Point(32, 11)

        self.assertTrue(robot_maze.line_is_unblocked(p, r, obstacles.obstacles))

        # Test line from polygon to polygon
        p = lines.Point(29, 12)
        r = lines.Point(32, 11)

        self.assertTrue(robot_maze.line_is_unblocked(p, r, obstacles.obstacles))

        # Test line along the side of a polygon
        p = lines.Point(32, 11)
        r = lines.Point(29, 21)

        self.assertTrue(robot_maze.line_is_unblocked(p, r, obstacles.obstacles))

        # Test line going through obstacle
        p = lines.Point(35, 21)
        r = lines.Point(29, 21)

        self.assertFalse(robot_maze.line_is_unblocked(p, r, obstacles.obstacles))

        # Test line going through obstacle, along edge of obstacle
        p = lines.Point(10, 14)
        r = lines.Point(16, 14)

        self.assertFalse(robot_maze.line_is_unblocked(p, r, obstacles.obstacles))


    def test_visible_vertices(self):
        # Test upper-right corner (open space)
        p = lines.Point(34, 22)

        self.assertEqual(set([lines.Point(32, 23), lines.Point(35, 21)]),
                         set(robot_maze.visible_vertices(p, obstacles.obstacles)))

        # Test lower-left corner (open space)
        p = lines.Point(5, 1)

        self.assertEqual(set([lines.Point(5, 20), lines.Point(6, 2),
                              lines.Point(6, 10), lines.Point(18, 2)]),
                         set(robot_maze.visible_vertices(p, obstacles.obstacles)))


        # Test bottom-middle (open space)
        p = lines.Point(23, 8)

        self.assertEqual(set([lines.Point(18, 2), lines.Point(19, 7),
                              lines.Point(23, 9), lines.Point(27, 11),
                              lines.Point(29, 12), lines.Point(29, 3),
                              lines.Point(26, 9), lines.Point(26, 6)]),
                         set(robot_maze.visible_vertices(p, obstacles.obstacles)))

        # Test bottom-middle (on triangle)
        p = lines.Point(23, 9)

        self.assertEqual(set([lines.Point(18, 2), lines.Point(19, 7),
                              lines.Point(19, 12), lines.Point(26, 6),
                              lines.Point(16, 17), lines.Point(22, 11),
                              lines.Point(26, 9), lines.Point(27, 11)]),
                         set(robot_maze.visible_vertices(p, obstacles.obstacles)))


        # Test far-right vertex of pentagon
        p = lines.Point(12, 19)

        self.assertEqual(set([lines.Point(8.5, 23), lines.Point(14.5, 21),
                              lines.Point(10, 14), lines.Point(13, 14)]),
                         set(robot_maze.visible_vertices(p, obstacles.obstacles)))

        # Test bottom of quadrilateral (on vertex)
        p = lines.Point(32, 11)

        self.assertEqual(set([lines.Point(29, 12), lines.Point(32, 9),
                              lines.Point(27, 23), lines.Point(29, 21),
                              lines.Point(35, 21)]),
                         set(robot_maze.visible_vertices(p, obstacles.obstacles)))

        # Test upper-left corner of rectangle2
        p = lines.Point(22, 23)

        self.assertEqual(set([lines.Point(27, 23), lines.Point(22, 11),
                              lines.Point(18.5, 23), lines.Point(21, 20),
                              lines.Point(18, 10), lines.Point(19, 12)]),
                         set(robot_maze.visible_vertices(p, obstacles.obstacles)))

        # Test top of triangle1 (on triangle)
        p = lines.Point(14.5, 21)

        self.assertEqual(set([lines.Point(13, 14), lines.Point(16, 14),
                              lines.Point(18.5, 23), lines.Point(16.5, 21.5),
                              lines.Point(10, 14), lines.Point(16, 17),
                              lines.Point(12, 19), lines.Point(8.5, 23),
                              lines.Point(19, 7), lines.Point(18, 10)]),
                         set(robot_maze.visible_vertices(p, obstacles.obstacles)))


    def test_vertices_relative_to_agent(self):
        # Test arbitrary points
        p = lines.Point(5, 5)
        vertices = [lines.Point(7, 7), lines.Point(4, 6),
                    lines.Point(5, 3), lines.Point(1, 5),
                    lines.Point(7, 0)]


        self.assertEqual(set([lines.Point(2, 2), lines.Point(-1, 1),
                              lines.Point(0, -2), lines.Point(-4, 0),
                              lines.Point(2, -5)]),
                         set(robot_maze.vertices_relative_to_agent(vertices, p)))

        # Test open space in maze near top-left corner
        p = lines.Point(34, 22)
        vertices = robot_maze.visible_vertices(p, obstacles.obstacles)

        self.assertEqual(set([lines.Point(-2, 1), lines.Point(1, -1)]),
                         set(robot_maze.vertices_relative_to_agent(vertices, p)))

        # Test on hexagon vertex in maze
        p = lines.Point(32, 6)
        vertices = robot_maze.visible_vertices(p, obstacles.obstacles)

        self.assertEqual(set([lines.Point(-3, -3), lines.Point(0, 3),
                              lines.Point(3, 15)]),
                         set(robot_maze.vertices_relative_to_agent(vertices, p)))

        # Test open space in maze in between pentagon and triangle
        p = lines.Point(13, 19)
        vertices = robot_maze.visible_vertices(p, obstacles.obstacles)

        self.assertEqual(set([lines.Point(-1, 0), lines.Point(-3, -5),
                              lines.Point(-4.5, 4), lines.Point(0, -5),
                              lines.Point(1.5, 2)]),
                         set(robot_maze.vertices_relative_to_agent(vertices, p)))



    def test_get_locations(self):
        # Test upper-right corner (open space)
        p = lines.Point(34, 22)
        vertex_list = robot_maze.visible_vertices(p, obstacles.obstacles)
        agent_vertex_list = robot_maze.vertices_relative_to_agent(vertex_list, p)

        self.assertEqual(set([lines.Point(34, 22), lines.Point(18, 13)]),
                         set(robot_maze.get_locations(agent_vertex_list, obstacles.obstacles)))

        # Test upper-left corner above pentagon (open space)
        p = lines.Point(12, 21)
        vertex_list = robot_maze.visible_vertices(p, obstacles.obstacles)
        agent_vertex_list = robot_maze.vertices_relative_to_agent(vertex_list, p)

        self.assertEqual(set([lines.Point(12, 21)]),
                         set(robot_maze.get_locations(agent_vertex_list, obstacles.obstacles)))

        # Test lower-right corner of hexagon (on polygon)
        p = lines.Point(32, 6)
        vertex_list = robot_maze.visible_vertices(p, obstacles.obstacles)
        agent_vertex_list = robot_maze.vertices_relative_to_agent(vertex_list, p)

        self.assertEqual(set([lines.Point(32, 6)]),
                         set(robot_maze.get_locations(agent_vertex_list, obstacles.obstacles)))

        # Test open space in middle
        p = lines.Point(5, 20)
        vertex_list = robot_maze.visible_vertices(p, obstacles.obstacles)
        agent_vertex_list = robot_maze.vertices_relative_to_agent(vertex_list, p)

        self.assertEqual(set([lines.Point(5, 20)]),
                         set(robot_maze.get_locations(agent_vertex_list, obstacles.obstacles)))

        # Test far-left side of pentagon (on polygon)
        p = lines.Point(20, 16)
        vertex_list = robot_maze.visible_vertices(p, obstacles.obstacles)
        agent_vertex_list = robot_maze.vertices_relative_to_agent(vertex_list, p)

        self.assertEqual(set([lines.Point(20, 16)]),
                         set(robot_maze.get_locations(agent_vertex_list, obstacles.obstacles)))


    def test_goal_test(self):
        goal_point = lines.Point(29, 17)

        # Test reachable goal from open space
        p = lines.Point(30, 12)

        self.assertTrue(robot_maze.goal_test(p, goal_point, obstacles.obstacles))

        # Test reachable goal from polygon
        p = lines.Point(29, 21)

        self.assertTrue(robot_maze.goal_test(p, goal_point, obstacles.obstacles))

        # Test unreachable goal from open space
        p = lines.Point(21, 18)

        self.assertFalse(robot_maze.goal_test(p, goal_point, obstacles.obstacles))

        # Test unreachable goal from polygon
        p = lines.Point(21, 20)

        self.assertFalse(robot_maze.goal_test(p, goal_point, obstacles.obstacles))


    def test_heuristic(self):
        # For now these tests are slightly redundant since heuristic is just
        # a straight line, but since the heuristic could change in the
        # future it's better to have them.

        # Test straight horizontal line
        p1 = lines.Point(1, 1)
        goal = lines.Point(4, 1)

        self.assertEqual(3, robot_maze.heuristic(p1, goal))

        # Test straight vertical line
        p1 = lines.Point(1, 1)
        goal = lines.Point(1, 4)

        self.assertEqual(3, robot_maze.heuristic(p1, goal))

        # Test positive horizontal line
        p1 = lines.Point(1, 1)
        goal = lines.Point(4, 4)

        self.assertEqual(4.242640687119285, robot_maze.heuristic(p1, goal))

        # Test negative horizontal line
        p1 = lines.Point(1, 1)
        goal = lines.Point(-2, -2)

        self.assertEqual(4.242640687119285, robot_maze.heuristic(p1, goal))


    def test_heuristic(self):
        # For now these tests are also redundant since actual_cost is just
        # a straight line, but this could also change in the future, so
        # we keep them.

        # Test straight horizontal line
        s1 = lines.Point(1, 1)
        s2 = lines.Point(4, 1)

        self.assertEqual(3, robot_maze.actual_cost(s1, s2, s2))

        # Test straight vertical line
        s1 = lines.Point(1, 1)
        s2 = lines.Point(1, 4)

        self.assertEqual(3, robot_maze.actual_cost(s1, s2, s2))

        # Test positive horizontal line
        s1 = lines.Point(1, 1)
        s2 = lines.Point(4, 4)

        self.assertEqual(4.242640687119285, robot_maze.actual_cost(s1, s2, s2))

        # Test negative horizontal line
        s1 = lines.Point(1, 1)
        s2 = lines.Point(-2, -2)

        self.assertEqual(4.242640687119285, robot_maze.actual_cost(s1, s2, s2))


    def test_actions(self):
        # Redundant since this only makes a call to visible_vertices, so only
        # test a subset of visible_vertices tests for a sanity check.

        # Test upper-right corner (open space)
        state = lines.Point(34, 22)

        self.assertEqual(set([lines.Point(32, 23), lines.Point(35, 21)]),
                         set(robot_maze.actions(state)))

        # Test lower-left corner (open space)
        state = lines.Point(5, 1)

        self.assertEqual(set([lines.Point(5, 20), lines.Point(6, 2),
                              lines.Point(6, 10), lines.Point(18, 2)]),
                         set(robot_maze.actions(state)))

        # Test far-right vertex of pentagon
        state = lines.Point(12, 19)

        self.assertEqual(set([lines.Point(8.5, 23), lines.Point(14.5, 21),
                              lines.Point(10, 14), lines.Point(13, 14)]),
                         set(robot_maze.actions(state)))

        # Test top of triangle1 (on triangle)
        state = lines.Point(14.5, 21)

        self.assertEqual(set([lines.Point(13, 14), lines.Point(16, 14),
                              lines.Point(18.5, 23), lines.Point(16.5, 21.5),
                              lines.Point(10, 14), lines.Point(16, 17),
                              lines.Point(12, 19), lines.Point(8.5, 23),
                              lines.Point(19, 7), lines.Point(18, 10)]),
                         set(robot_maze.actions(state)))


    def test_perform_action(self):
        # TODO Beef up these tests after we add random failure logic
        action = lines.Point(1, 1)
        self.assertEqual(action, robot_maze.perform_action(action))


    def test_LRTA_star_cost(self):
        # Test without state
        prev_state = lines.Point(1, 1)
        action = lines.Point(3, 1)
        state = None
        cost_estimates = {lines.Point(3, 1): 2, lines.Point(1, 1): 4}
        goal_point = lines.Point(5, 1)

        self.assertEqual(4, robot_maze.LRTA_star_cost(prev_state, action,
                                                      state, cost_estimates,
                                                      goal_point))

        # Test with state
        state = lines.Point(3, 1)
        self.assertEqual(4, robot_maze.LRTA_star_cost(prev_state, action,
                                                      state, cost_estimates,
                                                      goal_point))


    def test_LRTA_star_agent(self):
        result = {}
        cost_estimates = {}
        prev_state = None
        prev_action = None
        goal_point = lines.Point(5, 5)
        agent_location = lines.Point(4, 5)

        self.assertEqual(None, robot_maze.LRTA_star_agent(
            agent_location, goal_point, obstacles.obstacles, result,
            cost_estimates, prev_state, prev_action))


        # Update goal point so agent has to plan an action
        goal_point = lines.Point(34, 22)

        prev_action, prev_state, result, cost_estimates = robot_maze.LRTA_star_agent(
            agent_location, goal_point, obstacles.obstacles, result,
            cost_estimates, prev_state, prev_action)

        self.assertEqual(cost_estimates[prev_state],
                         robot_maze.heuristic(prev_state, goal_point))

        self.assertFalse((None, None) in result)

        self.assertFalse(None in cost_estimates)

        self.assertEqual(lines.Point(6, 2), prev_action)

        self.assertEqual(lines.Point(4, 5), prev_state)

        agent_location = robot_maze.perform_action(prev_action)

        prev_action, prev_state, result, cost_estimates = robot_maze.LRTA_star_agent(
            agent_location, goal_point, obstacles.obstacles, result,
            cost_estimates, prev_state, prev_action)

        self.assertEqual(prev_state,
                         result[(lines.Point(4, 5), lines.Point(6, 2))])

        self.assertEqual(34.48187929913333, cost_estimates[lines.Point(4, 5)])

        self.assertEqual(lines.Point(18, 2), prev_action)

        self.assertEqual(lines.Point(6, 2), prev_state)


    def test_run_simulation(self):
        number_of_turns = 1000
        goal_point = lines.Point(34, 22)
        initial_location = lines.Point(5, 5)

        self.assertTrue(robot_maze.run_simulation(
            number_of_turns, goal_point, initial_location, obstacles.obstacles))


if __name__ == "__main__":
    unittest.main()
