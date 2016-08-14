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

        self.assertEqual(sorted([lines.Point(32, 23), lines.Point(35, 21)],
                                key=lambda p: p.x + p.y),
                         sorted(robot_maze.visible_vertices(p, obstacles.obstacles),
                                key=lambda p: p.x + p.y))

        # Test lower-left corner (open space)
        p = lines.Point(5, 1)

        self.assertEqual(sorted([lines.Point(5, 20), lines.Point(6, 2),
                                 lines.Point(6, 10), lines.Point(18, 2)],
                                key=lambda p: p.x + p.y),
                         sorted(robot_maze.visible_vertices(p, obstacles.obstacles),
                                key=lambda p: p.x + p.y))


        # Test bottom-middle (open space)
        p = lines.Point(23, 8)

        self.assertEqual(sorted([lines.Point(18, 2), lines.Point(19, 7),
                                 lines.Point(23, 9), lines.Point(27, 11),
                                 lines.Point(29, 12), lines.Point(29, 3),
                                 lines.Point(26, 9), lines.Point(26, 6)],
                                key=lambda p: p.x + p.y),
                         sorted(robot_maze.visible_vertices(p, obstacles.obstacles),
                                key=lambda p: p.x + p.y))

        # Test bottom-middle (on triangle)
        p = lines.Point(23, 9)

        self.assertEqual(sorted([lines.Point(18, 2), lines.Point(19, 7),
                                 lines.Point(19, 12), lines.Point(26, 6),
                                 lines.Point(16, 17), lines.Point(22, 11),
                                 lines.Point(26, 9), lines.Point(27, 11)],
                                key=lambda p: p.x + p.y),
                         sorted(robot_maze.visible_vertices(p, obstacles.obstacles),
                                key=lambda p: p.x + p.y))


        # Test far-right vertex of pentagon
        p = lines.Point(12, 19)

        self.assertEqual(sorted([lines.Point(8.5, 23), lines.Point(14.5, 21),
                                 lines.Point(10, 14), lines.Point(13, 14)],
                                key=lambda p: p.x + p.y),
                         sorted(robot_maze.visible_vertices(p, obstacles.obstacles),
                                key=lambda p: p.x + p.y))

        # Test bottom of quadrilateral (on vertex)
        p = lines.Point(32, 11)

        self.assertEqual(sorted([lines.Point(29, 12), lines.Point(32, 9),
                                 lines.Point(27, 23), lines.Point(29, 21),
                                 lines.Point(35, 21)],
                                key=lambda p: p.x + p.y),
                         sorted(robot_maze.visible_vertices(p, obstacles.obstacles),
                                key=lambda p: p.x + p.y))

        # Test upper-left corner of rectangle2
        p = lines.Point(22, 23)

        self.assertEqual(sorted([lines.Point(27, 23), lines.Point(22, 11),
                                 lines.Point(18.5, 23), lines.Point(21, 20),
                                 lines.Point(18, 10), lines.Point(19, 12)],
                                key=lambda p: p.x + p.y),
                         sorted(robot_maze.visible_vertices(p, obstacles.obstacles),
                                key=lambda p: p.x + p.y))

        # Test top of triangle1 (on triangle)
        p = lines.Point(14.5, 21)

        self.assertEqual(sorted([lines.Point(13, 14), lines.Point(16, 14),
                                 lines.Point(18.5, 23), lines.Point(16.5, 21.5),
                                 lines.Point(10, 14), lines.Point(16, 17),
                                 lines.Point(12, 19), lines.Point(8.5, 23),
                                 lines.Point(19, 7), lines.Point(18, 10)],
                                key=lambda p: p.x + p.y),
                         sorted(robot_maze.visible_vertices(p, obstacles.obstacles),
                                key=lambda p: p.x + p.y))


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



if __name__ == "__main__":
    unittest.main()
