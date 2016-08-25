from lines import Point, Obstacle

# Farthest legal x and y coordinates, respectively
x_bounds = 40
y_bounds = 25

# Visible obstacles for the maze
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
                          [Point(29, 21), Point(32, 23)]])


visible_obstacles = [rectangle1, pentagon, triangle1, trapezoid,
             triangle2, rectangle2, hexagon, quadrilateral]
