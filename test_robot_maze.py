import unittest
import lines
import robot_maze

class TestRobotMaze(unittest.TestCase):

    def test_visible_vertices(self):
        p = lines.Point(34, 22)

        self.assertEqual([lines.Point(35, 21), lines.Point(32, 23)],
                         robot_maze.point_visibility_polygons(p, robot_maze.obstacles))


if __name__ == "__main__":
    unittest.main()
