import math
from matplotlib import path

class Point():

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return '({}, {})'.format(self.x, self.y)

    def __eq__(self, other):
        return (self.x, self.y) == (other.x, other.y)

    def __hash__(self):
        return hash((self.x, self.y))


class Obstacle():

    def __init__(self, lines):
        # Lines is a list of tuples of points, where the first element
        # of each tuple is the unique point
        self.lines = lines


    def __repr__(self):
        return '{}'.format(self.lines)


def on_segment(p, r, q, epsilon):
    """
    Given three colinear points p, q, r, and a threshold epsilone, determine if
    determine if point q lies on line segment pr
    """
    # Taken from http://stackoverflow.com/questions/328107/how-can-you-determine-a-point-is-between-two-other-points-on-a-line-segment
    crossproduct = (q.y - p.y) * (r.x - p.x) - (q.x - p.x) * (r.y - p.y)
    if abs(crossproduct) > epsilon:
        return False   # (or != 0 if using integers)

    dotproduct = (q.x - p.x) * (r.x - p.x) + (q.y - p.y)*(r.y - p.y)
    if dotproduct < 0:
        return False

    squaredlengthba = (r.x - p.x)*(r.x - p.x) + (r.y - p.y)*(r.y - p.y)
    if dotproduct > squaredlengthba:
        return False

    return True


def orientation(p, q, r):
    """
    Returns the orientation of the ordered triplet (p, q, r).
    The function returns the following values:
    0 --> p, q, r, are colinear
    1 --> Clockwise
    2 --> Counterclockwise
    """
    # Taken from:
    # http://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
    val = ((q.y - p.y) * (r.x - q.x)) - ((q.x - p.x) * (r.y - q.y))

    if val == 0:
        # (p, q, r) is colinear
        return 0
    elif val > 0:
        # (p, q, r) is clockwise
        return 1
    else:
        # (p, q, r) is counterclockwise
        return 2


def inside_area(p, q, r):
    """
    Given a line segment pr and a point q, determine if the point q
    lies inside the rectangle surrounding line segment pr.
    """
    # Taken from:
    # http://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

    if (q.x <= max(p.x, r.x) and q.x >= min(p.x, r.x) and
        q.y <= max(p.y, r.y) and q.y >= min(p.y, r.y)):
        return True
    else:
        return False


def do_intersect(p1, q1, p2, q2):
    """
    Determines if line segment p1q1 and p2q2 intersect
    """
    # Taken from:
    # http://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

    # Threshold for being on a line
    epsilon = 0.01

    # Find the four orientations needed
    # TODO why do we need orientation?
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)


    # General case
    if (o1 != o2) and (o3 != o4):
        return True
    # Special cases
    # p1, q1, and p2 are colinear and p2 lies on segment p1q1
    elif o1 == 0 and inside_area(p1, p2, q1) and on_segment(p1, q1, p2, epsilon):
        return True
    # p1, q1, and p2 are colinear and q2 lies on segment p1q1
    elif o2 == 0 and inside_area(p1, q2, q1) and on_segment(p1, q1, q2, epsilon):
        return True
    # p2, q2, and p1 are colinear and p1 lies on segment p2q2
    elif o3 == 0 and inside_area(p2, p1, q2) and on_segment(p2, q2, p1, epsilon):
        return True
    # p2, q2, and q1 are colinear and q1 lies on segment p2q2
    elif o4 == 0 and inside_area(p2, q1, q2) and on_segment(p2, q2, q1, epsilon):
        return True
    # Doesn't fall into any of the above cases
    else:
        return False


def distance(p1, p2):
    """
    Given two points, return the distance between them.
    """
    return math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)


def point_inside_own_obstacle(p, O):
    """
    Given a point and an obstacle, determine if p is a vertex of O.
    """
    return p in [v[0] for v in O.lines]


def line_is_valid_for_own_obstacle(p, r, O):
    """
    Given a line pr and its member obstacle O, determine if line pr
    crosses through O.
    """
    line_in_obstacle = False

    for line in O.lines:
        if (sorted([p, r], key=lambda p: p.x + p.y) ==
            sorted(line, key=lambda p: p.x + p.y)):
            line_in_obstacle = True

    return line_in_obstacle


def point_in_any_obstacle(p, obstacles):
    """
    Given a point p and a list of obstacles, determine if p
    is inside any obstacles.
    """
    for obstacle in obstacles:
        unique_points = [(line[0].x, line[0].y) for line in obstacle.lines]
        matplot_obstacle = path.Path(unique_points)
        if matplot_obstacle.contains_point((p.x, p.y)):
            return True

    return False


def obstacle_inside_area(p, r, O):
    """
    Given an obstacle O, determine whether O lies
    inside the rectangle surrounding line segment pr
    """
    for vertex in [v[0] for v in O.lines]:
        if inside_area(p, vertex, r):
            return True

    return False


def obstacle_blocks_line(p, r, O):
    """
    Given a line segment pr and an obstacle O,
    determine if any of the line segments in O intersect pr.
    """
    for line in O.lines:
        if (p not in line and r not in line and
            do_intersect(p, r, line[0], line[1])):
            return True

    return False


def is_colinear(p, q, r, epsilon):
    """
    Returns true if points p, q, r, are colinear, otherwise returns false.
    pqr is considered colinear if the area of triangle pqr is less than
    the error tolerance epsilon.
    """
    # Get the area of triangle pqr
    area = ((p.x * (q.y - r.y)) + (q.x * (r.y - p.y)) + (r.x * (p.y - q.y))) / 2

    return area < epsilon


