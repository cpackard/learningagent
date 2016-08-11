import unittest
import lines
import robot_maze

class TestLines(unittest.TestCase):

    def test_on_segment(self):
        # q clearly on line pr
        p = lines.Point(1, 1)
        q = lines.Point(2, 1)
        r = lines.Point(3, 1)
        epsilon = 0.01

        self.assertTrue(lines.on_segment(p, r, q, epsilon))

        p = lines.Point(1, 1)
        q = lines.Point(2, 2)
        r = lines.Point(3, 3)
        epsilon = 0.01

        self.assertTrue(lines.on_segment(p, r, q, epsilon))

        p = lines.Point(1, 1)
        q = lines.Point(1, 2)
        r = lines.Point(1, 3)
        epsilon = 0.01

        self.assertTrue(lines.on_segment(p, r, q, epsilon))


        # q clearly outside of line pr
        p = lines.Point(19, 12)
        q = lines.Point(22, 11)
        r = lines.Point(23, 9)
        epsilon = 0.01

        self.assertFalse(lines.on_segment(p, r, q, epsilon))

        # q on edge of line pr
        p = lines.Point(1, 1)
        q = lines.Point(3, 3)
        r = lines.Point(3, 3)
        epsilon = 0.01

        self.assertTrue(lines.on_segment(p, r, q, epsilon))


    def test_orientation(self):
        # Test colinear
        p = lines.Point(1, 1)
        q = lines.Point(2, 2)
        r = lines.Point(3, 3)

        self.assertEquals(0, lines.orientation(p, r, q))

        # Test clockwise
        p = lines.Point(1, 1)
        q = lines.Point(4, 2)
        r = lines.Point(3, 3)

        self.assertEquals(1, lines.orientation(p, r, q))

        # Test counter-clockwise
        p = lines.Point(1, 1)
        q = lines.Point(4, 5)
        r = lines.Point(3, 3)

        self.assertEquals(2, lines.orientation(p, r, q))


    def test_inside_area(self):
        # q clearly inside area of line pr
        p = lines.Point(1, 1)
        r = lines.Point(3, 3)
        q = lines.Point(2, 2)

        self.assertTrue(lines.inside_area(p, q, r))

        # q clearly outside area of line pr
        p = lines.Point(1, 1)
        r = lines.Point(3, 3)
        q = lines.Point(5, 5)

        self.assertFalse(lines.inside_area(p, q, r))

        # q on edge of area of line pr
        p = lines.Point(1, 1)
        r = lines.Point(3, 3)
        q = lines.Point(1, 3)

        self.assertTrue(lines.inside_area(p, q, r))



    def test_do_instersect(self):
        # sanity check - bottom-left rectangle
        p1 = lines.Point(18, 10)
        q1 = lines.Point(18, 2)
        p2 = lines.Point(6,10)
        q2 = lines.Point(18, 10)

        self.assertTrue(lines.do_intersect(p1, q1, p2, q2))

        # sanity check - bottom-right hexagon
        p1 = lines.Point(29, 12)
        q1 = lines.Point(32, 9)
        p2 = lines.Point(32, 6)
        q2 = lines.Point(32, 9)

        self.assertTrue(lines.do_intersect(p1, q1, p2, q2))

        # test bottom-middle triangle to upper-left pentagon
        p1 = lines.Point(19, 7)
        q1 = lines.Point(10, 14)
        p2 = lines.Point(18, 10)
        q2 = lines.Point(18, 2)

        self.assertTrue(lines.do_intersect(p1, q1, p2, q2))

        # test bottom-right hexagon to top-middle rectangle
        p1 = lines.Point(26, 9)
        q1 = lines.Point(29, 12)
        p2 = lines.Point(27, 23)
        q2 = lines.Point(27, 11)

        self.assertFalse(lines.do_intersect(p1, q1, p2, q2))


    def test_obstacle_inside_area(self):
        # Test obstacle clearly inside area
        p = lines.Point(23, 9)
        r = lines.Point(6, 10)
        O = robot_maze.rectangle1

        self.assertTrue(lines.obstacle_inside_area(p, r, O))

        # Test obstacle clearly outside area
        p = lines.Point(26, 9)
        r = lines.Point(23, 9)
        O = robot_maze.rectangle2

        self.assertFalse(lines.obstacle_inside_area(p, r, O))

        # Test obstacle on edge of area
        p = lines.Point(19, 7)
        r = lines.Point(18, 10)
        O = robot_maze.rectangle1

        self.assertTrue(lines.obstacle_inside_area(p, r, O))


    def test_obstacle_blocks_line(self):
        # Test line clearly crossing through obstacle
        p = lines.Point(23, 9)
        r = lines.Point(6, 10)
        O = robot_maze.rectangle1

        self.assertTrue(lines.obstacle_blocks_line(p, r, O),
                        "Line intersection not detected.")

        # Test one line segment end touching obstacle,
        # but otherwise not intersecting
        p = lines.Point(26, 9)
        r = lines.Point(23, 9)
        O = robot_maze.hexagon

        self.assertFalse(lines.obstacle_blocks_line(p, r, O),
                         "Only ends touch, shouldn't count as intersection.")

        # Test line clearly not intersecting obstacle
        p = lines.Point(32, 23)
        r = lines.Point(35, 21)
        O = robot_maze.hexagon

        self.assertFalse(lines.obstacle_blocks_line(p, r, O),
                         "Lines are nowhere near each other, something's wrong.")

        # Test line intersecting obstacle by passing along
        # one of the obstacle's edges
        p = lines.Point(32, 6)
        r = lines.Point(32, 11)
        O = robot_maze.hexagon

        self.assertTrue(lines.obstacle_blocks_line(p, r, O),
                        "Line should intersect along obstacle's edge.")


if __name__ == '__main__':
    unittest.main()