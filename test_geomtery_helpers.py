import unittest
import geometry_helpers
import environment_details

class TestGeometryHelpers(unittest.TestCase):

    def test_on_segment(self):
        # q clearly on line pr
        p = geometry_helpers.Point(1, 1)
        q = geometry_helpers.Point(2, 1)
        r = geometry_helpers.Point(3, 1)
        epsilon = 0.01

        self.assertTrue(geometry_helpers.on_segment(p, r, q, epsilon))

        p = geometry_helpers.Point(1, 1)
        q = geometry_helpers.Point(2, 2)
        r = geometry_helpers.Point(3, 3)
        epsilon = 0.01

        self.assertTrue(geometry_helpers.on_segment(p, r, q, epsilon))

        p = geometry_helpers.Point(1, 1)
        q = geometry_helpers.Point(1, 2)
        r = geometry_helpers.Point(1, 3)
        epsilon = 0.01

        self.assertTrue(geometry_helpers.on_segment(p, r, q, epsilon))


        # q clearly outside of line pr
        p = geometry_helpers.Point(19, 12)
        q = geometry_helpers.Point(22, 11)
        r = geometry_helpers.Point(23, 9)
        epsilon = 0.01

        self.assertFalse(geometry_helpers.on_segment(p, r, q, epsilon))

        # q on edge of line pr
        p = geometry_helpers.Point(1, 1)
        q = geometry_helpers.Point(3, 3)
        r = geometry_helpers.Point(3, 3)
        epsilon = 0.01

        self.assertTrue(geometry_helpers.on_segment(p, r, q, epsilon))


    def test_orientation(self):
        # Test colinear
        p = geometry_helpers.Point(1, 1)
        q = geometry_helpers.Point(2, 2)
        r = geometry_helpers.Point(3, 3)

        self.assertEqual(geometry_helpers.orientation(p, r, q), 0)

        # Test clockwise
        p = geometry_helpers.Point(1, 1)
        q = geometry_helpers.Point(4, 2)
        r = geometry_helpers.Point(3, 3)

        self.assertEqual(geometry_helpers.orientation(p, r, q), 1)

        # Test counter-clockwise
        p = geometry_helpers.Point(1, 1)
        q = geometry_helpers.Point(4, 5)
        r = geometry_helpers.Point(3, 3)

        self.assertEqual(geometry_helpers.orientation(p, r, q), 2)


    def test_inside_area(self):
        # q clearly inside area of line pr
        p = geometry_helpers.Point(1, 1)
        r = geometry_helpers.Point(3, 3)
        q = geometry_helpers.Point(2, 2)

        self.assertTrue(geometry_helpers.inside_area(p, q, r))

        # q clearly outside area of line pr
        p = geometry_helpers.Point(1, 1)
        r = geometry_helpers.Point(3, 3)
        q = geometry_helpers.Point(5, 5)

        self.assertFalse(geometry_helpers.inside_area(p, q, r))

        # q on edge of area of line pr
        p = geometry_helpers.Point(1, 1)
        r = geometry_helpers.Point(3, 3)
        q = geometry_helpers.Point(1, 3)

        self.assertTrue(geometry_helpers.inside_area(p, q, r))



    def test_do_instersect(self):
        # sanity check - bottom-left rectangle
        p1 = geometry_helpers.Point(18, 10)
        q1 = geometry_helpers.Point(18, 2)
        p2 = geometry_helpers.Point(6,10)
        q2 = geometry_helpers.Point(18, 10)

        self.assertTrue(geometry_helpers.do_intersect(p1, q1, p2, q2))

        # sanity check - bottom-right hexagon
        p1 = geometry_helpers.Point(29, 12)
        q1 = geometry_helpers.Point(32, 9)
        p2 = geometry_helpers.Point(32, 6)
        q2 = geometry_helpers.Point(32, 9)

        self.assertTrue(geometry_helpers.do_intersect(p1, q1, p2, q2))

        # test bottom-middle triangle to upper-left pentagon
        p1 = geometry_helpers.Point(19, 7)
        q1 = geometry_helpers.Point(10, 14)
        p2 = geometry_helpers.Point(18, 10)
        q2 = geometry_helpers.Point(18, 2)

        self.assertTrue(geometry_helpers.do_intersect(p1, q1, p2, q2))

        # test bottom-right hexagon to top-middle rectangle
        p1 = geometry_helpers.Point(26, 9)
        q1 = geometry_helpers.Point(29, 12)
        p2 = geometry_helpers.Point(27, 23)
        q2 = geometry_helpers.Point(27, 11)

        self.assertFalse(geometry_helpers.do_intersect(p1, q1, p2, q2))


    def test_distance(self):
        # Test straight horizontal line
        p1 = geometry_helpers.Point(1, 1)
        p2 = geometry_helpers.Point(4, 1)

        self.assertEqual(3, geometry_helpers.distance(p1, p2))

        # Test straight vertical line
        p1 = geometry_helpers.Point(1, 1)
        p2 = geometry_helpers.Point(1, 4)

        self.assertEqual(3, geometry_helpers.distance(p1, p2))

        # Test positive horizontal line
        p1 = geometry_helpers.Point(1, 1)
        p2 = geometry_helpers.Point(4, 4)

        self.assertEqual(4.242640687119285, geometry_helpers.distance(p1, p2))

        # Test negative horizontal line
        p1 = geometry_helpers.Point(1, 1)
        p2 = geometry_helpers.Point(-2, -2)

        self.assertEqual(4.242640687119285, geometry_helpers.distance(p1, p2))


    def test_point_inside_own_obstacle(self):
        # Test point is clearly vertex
        p = geometry_helpers.Point(6, 2)
        obstacle = environment_details.rectangle1

        self.assertTrue(geometry_helpers.point_inside_own_obstacle(p, obstacle))

        # Test point on edge of obstacle
        p = geometry_helpers.Point(10, 2)
        obstacle = environment_details.rectangle1

        self.assertFalse(geometry_helpers.point_inside_own_obstacle(p, obstacle))

        # Test point clearly outside obstacle
        p = geometry_helpers.Point(6, 2)
        obstacle = environment_details.hexagon

        self.assertFalse(geometry_helpers.point_inside_own_obstacle(p, obstacle))


    def test_line_is_valid_for_own_obstacle(self):
        # Test line is part of obstacle
        p = geometry_helpers.Point(6, 2)
        r = geometry_helpers.Point(6, 10)
        obstacle = environment_details.rectangle1

        self.assertTrue(geometry_helpers.line_is_valid_for_own_obstacle(p, r, obstacle))

        # Test line crosses through obstacle
        p = geometry_helpers.Point(6, 2)
        r = geometry_helpers.Point(18, 10)
        obstacle = environment_details.rectangle1

        self.assertFalse(geometry_helpers.line_is_valid_for_own_obstacle(p, r, obstacle))


    def test_point_in_any_obstacle(self):
        # Test point clearly inside obstacle
        p = geometry_helpers.Point(8, 8)
        obstacles = environment_details.visible_obstacles

        self.assertTrue(geometry_helpers.point_in_any_obstacle(p, obstacles))

        # Test point clearly outside obstacle
        p = geometry_helpers.Point(5, 5)
        obstacles = environment_details.visible_obstacles

        self.assertFalse(geometry_helpers.point_in_any_obstacle(p, obstacles))

        # Test point on edge of obstacle
        p = geometry_helpers.Point(10, 2)
        obstacles = environment_details.visible_obstacles

        self.assertFalse(geometry_helpers.point_in_any_obstacle(p, obstacles))

    def test_obstacle_inside_area(self):
        # Test obstacle clearly inside area
        p = geometry_helpers.Point(23, 9)
        r = geometry_helpers.Point(6, 10)
        O = environment_details.rectangle1

        self.assertTrue(geometry_helpers.obstacle_inside_area(p, r, O))

        # Test obstacle clearly outside area
        p = geometry_helpers.Point(26, 9)
        r = geometry_helpers.Point(23, 9)
        O = environment_details.rectangle2

        self.assertFalse(geometry_helpers.obstacle_inside_area(p, r, O))

        # Test obstacle on edge of area
        p = geometry_helpers.Point(19, 7)
        r = geometry_helpers.Point(18, 10)
        O = environment_details.rectangle1

        self.assertTrue(geometry_helpers.obstacle_inside_area(p, r, O))


    def test_obstacle_blocks_line(self):
        # Test line clearly crossing through obstacle
        p = geometry_helpers.Point(23, 9)
        r = geometry_helpers.Point(6, 10)
        O = environment_details.rectangle1

        self.assertTrue(geometry_helpers.obstacle_blocks_line(p, r, O),
                        "Line intersection not detected.")

        # Test one line segment end touching obstacle,
        # but otherwise not intersecting
        p = geometry_helpers.Point(26, 9)
        r = geometry_helpers.Point(23, 9)
        O = environment_details.hexagon

        self.assertFalse(geometry_helpers.obstacle_blocks_line(p, r, O),
                         "Only ends touch, shouldn't count as intersection.")

        # Test line clearly not intersecting obstacle
        p = geometry_helpers.Point(32, 23)
        r = geometry_helpers.Point(35, 21)
        O = environment_details.hexagon

        self.assertFalse(geometry_helpers.obstacle_blocks_line(p, r, O),
                         "Lines are nowhere near each other, something's wrong.")

        # Test line intersecting obstacle by passing along
        # one of the obstacle's edges
        p = geometry_helpers.Point(32, 6)
        r = geometry_helpers.Point(32, 11)
        O = environment_details.hexagon

        self.assertTrue(geometry_helpers.obstacle_blocks_line(p, r, O),
                        "Line should intersect along obstacle's edge.")


if __name__ == '__main__':
    unittest.main()
