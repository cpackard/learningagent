from lines import *

# TODO Since these are used by both lines and robot_maze,
# these definition should really be in a separate file
rectangle1 = Obstacle([[Point(6, 2), Point(18, 2)],
                       [Point(18, 2), Point(18, 10)],
                       [Point(18, 10), Point(6, 10)],
                       [Point(6, 10), Point(6, 2)]])

pentagon = Obstacle([[Point(7, 15), Point(10, 14)],
                     [Point(10, 14), Point(12, 19)],
                     [Point(12, 19), Point(8.5, 23)],
                     [Point(8.5, 23), Point(5, 20)],
                     [Point(5, 20), Point(7, 15)]])

triangle1 = Obstacle([[Point(13, 14), Point(16, 14)],
                      [Point(16, 14), Point(14.5, 21)],
                      [Point(14.5, 21), Point(13, 14)]])

trapezoid = Obstacle([[Point(16, 17), Point(21, 20)],
                      [Point(21, 20), Point(18.5, 23)],
                      [Point(18.5, 23), Point(16.5, 21.5)],
                      [Point(16.5, 21.5), Point(16, 17)]])

triangle2 = Obstacle([[Point(19, 7), Point(23, 9)],
                      [Point(23, 9), Point(19, 12)],
                      [Point(19, 12), Point(19, 7)]])

rectangle2 = Obstacle([[Point(22, 11), Point(27, 11)],
                       [Point(27, 11), Point(27, 23)],
                       [Point(27, 23), Point(22, 23)],
                       [Point(22, 23), Point(22, 11)]])

hexagon = Obstacle([[Point(29, 12), Point(32, 9)],
                    [Point(32, 9), Point(32, 6)],
                    [Point(32, 6), Point(29, 3)],
                    [Point(29, 3), Point(26, 6)],
                    [Point(26, 6), Point(26, 9)],
                    [Point(26, 9), Point(29, 12)]])

quadrilateral = Obstacle([[Point(32, 23), Point(35, 21)],
                          [Point(35, 21), Point(32, 11)],
                          [Point(32, 11), Point(29, 21)],
                          [Point(29, 11), Point(32, 23)]])


obstacles = [rectangle1, pentagon, triangle1, trapezoid,
             triangle2, rectangle2, hexagon, quadrilateral]



def distance(p1, p2):
    """
    Given two points, return the distance between them.
    """
    pass


# Visibility Polygon Algorithm (via Wikipedia)
def point_visibility_polygons(p, obstacles):
    """
    Given a point p and a set of obstacles S, return a list of vertices
    visible from p.
    """
    V = []

    for obstacle in obstacles:
        for vertex in [v[0] for v in obstacle.lines]:
            # # shoot a ray from p to vertex
            # r = distance(p, vertex)
            # # TODO convert this to add closest vertices instead!!!
            # theta = angle_of_v_with_respect_to_p(vertex, p)
            # for obstacle_prime in S:
            #     r = min(r, distance(p, obstacle_prime))
            # V.append(theta, r)
            valid_line = True

            for obstacle_prime in obstacles:
                # TODO Write tests for obstacle_crosses_line
                # Follow TDD motherfucker
                if (obstacle_inside_area(p, vertex, obstacle_prime) and
                    obstacle_blocks_line(p, vertex, obstacle_prime)):
                    valid_line = False

            if valid_line:
                V.append(vertex)

    return V



# persistent for LRTA*
result = {} # a table indexed by state and action, initially empty
H = {} # a table of cost estimates indexed by state, initially empty
s = None # the previous state, initially null
a = None # the previous action, initially null

def goal_test():
    pass


def heuristic():
    pass


def actions(state):
    pass





def LRTA_star_cost(prev_state, action, state, h_table):
    """
    Returns the f-value of a given state: g(s) + h(s)
    """
    if state is None:
        return heuristic(prev_state)
    else:
        return actual_cost(prev_state, action, state) + h_table[state]


# LRTA* algorithm from AIMA
def LRTA_star_agent(state):
    """
    Given a percept that identifies the current state, return the next
    action to take, or None if the goal is reached.
    """
    if goal_test(state):
        return None

    if state not in H:
        H[state] = heuristic(state)

    if s is not null:
        result[(s, a)] = state
        H[state] = min([LRTA_star_cost(s, b, result[(s, b)], H) for b in actions(s)])

    a = min([(b, LRTA_star_cost(state, b, result[(state, b)], H)) for b in actions(state)])
    s = state

    return a

