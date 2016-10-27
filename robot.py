import numpy as np
import math

class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''

        self.location = (0, 0)
        self.heading = 'up'
        self.maze_dim = maze_dim
        self.exploring = True
        self.goal_location = None
        self.optimal_steps = None
        self.race_time_step = None
        self.maze_walls = self.initial_maze_walls(maze_dim)
        # maze_cell_possibilities will be used to find which areas of the maze
        # the robots has least knowledge of.
        self.maze_cell_possibilities = self.initial_cell_possibilities(maze_dim)

    def initial_maze_walls(self, maze_dim):
        '''
        Construct a two dimensional array that represents all of the walls in
        the maze including exterior walls.
        -1 = unknown
        0 = no wall
        1 = wall
        '''
        maze_walls = [[-1] * maze_dim] + [[-1] * (maze_dim + 1), [-1] * maze_dim] * maze_dim
        for i in range(maze_dim):
            for j in range(maze_dim):
                if i == 0 or i == maze_dim * 2:
                    # exterior wall
                    maze_walls[i][j] = 1
                elif i % 2 == 1 and (j == 0 or j == maze_dim - 1):
                    # exterior wall
                    maze_walls[i][j] = 1
                elif i == maze_dim and (j == maze_dim / 2 or j == maze_dim / 2 - 1):
                    # inside goal area
                    maze_walls[i][j] = 0
                elif (i == maze_dim - 1 or i == maze_dim + 1) and j == maze_dim / 2:
                    # inside goal area
                    maze_walls[i][j] = 0

        return maze_walls

    def initial_cell_possibilities(self, maze_dim):
        '''
        Construct a two dimensional array that represents the number of
        possibile shapes each cell could take. It is worth noting that corner,
        edge, and center cells have fewer possible shapes than other interior
        cells.
        '''
        cell_possibilities = [[15] * maze_dim] * maze_dim
        for i in range(maze_dim):
            for j in range(maze_dim):
                if i == 0 or i == maze_dim - 1:
                    if j == 0 or j == maze_dim - 1:
                        # corner
                        cell_possibilities[i][j] = 2
                    else:
                        # edge
                        cell_possibilities[i][j] = 7
                elif j == 0 or j == maze_dim - 1:
                    # edge
                    cell_possibilities[i][j] = 7
                elif i == maze_dim / 2 - 1 or i == maze_dim / 2 or j == maze_dim / 2 - 1 or j == maze_dim / 2:
                    # center
                    cell_possibilities[i][j] = 3

        return cell_possibilities

    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''

        if not self.exploring:
            step = self.optimal_steps[self.race_time_step]
            self.race_time_step += 1
            return step

        self.update_maze_walls(sensors)

        unknown_center = self.center_of_largest_unknown_area(self)
        maze_graph = self.convert_maze_map_to_graph(self, self.location, unknown_center)
        path = self.best_path_through_graph(maze_graph)
        steps = self.convert_path_to_steps(path)

        rotation = steps[0][0]
        movement = steps[0][1]

        self.heading = self.update_direction(self.heading, rotation)

        if self.exploring and self.optimal_path_has_been_found():
            self.exploring = False
            self.race_time_step = 0
            explored_maze_graph = self.convert_maze_map_to_graph(self, (0,0), self.goal_location)
            optimal_path = self.best_path_through_graph(explored_maze_graph)
            self.optimal_steps = self.convert_path_to_steps(optimal_path)
            return 'Reset', 'Reset'

        return rotation, movement

    def update_maze_walls(self, sensors):
        '''
        Update map of walls and opennings based on current location, heading,
        and sensor readings.
        '''
        x, y = self.location
        left_reading, forward_reading, right_reading = sensors

        if self.heading == 'up':
            for i in range(left_reading + 1):
                wall_value = 1 if i == left_reading else 0
                self.maze_walls[2 * x - i][y] = wall_value
            for j in range(forward_reading + 1):
                wall_value = 1 if j == forward_reading else 0
                self.maze_walls[2 * x + 1][y + 1 + j] = wall_value
            for k in range(right_reading + 1):
                wall_value = 1 if k == right_reading else 0
                self.maze_walls[2 * x + 2 + k][y] = wall_value

        elif self.heading == 'right':
            for i in range(left_reading + 1):
                wall_value = 1 if i == left_reading else 0
                self.maze_walls[2 * x + 1][y + 1 + i] = wall_value
            for j in range(forward_reading + 1):
                wall_value = 1 if j == forward_reading else 0
                self.maze_walls[2 * x + 2 + j][y] = wall_value
            for k in range(right_reading + 1):
                wall_value = 1 if k == right_reading else 0
                self.maze_walls[2 * x + 1][y - k] = wall_value

        elif self.heading == 'down':
            for i in range(left_reading + 1):
                wall_value = 1 if i == left_reading else 0
                self.maze_walls[2 * x + 2 + i][y] = wall_value
            for j in range(forward_reading + 1):
                wall_value = 1 if j == forward_reading else 0
                self.maze_walls[2 * x + 1][y - j] = wall_value
            for k in range(right_reading + 1):
                wall_value = 1 if k == right_reading else 0
                self.maze_walls[2 * x - k][y] = wall_value

        elif self.heading == 'left':
            for i in range(left_reading + 1):
                wall_value = 1 if i == left_reading else 0
                self.maze_walls[2 * x + 1][y - i] = wall_value
            for j in range(forward_reading + 1):
                wall_value = 1 if j == forward_reading else 0
                self.maze_walls[2 * x - j][y] = wall_value
            for k in range(right_reading + 1):
                wall_value = 1 if k == right_reading else 0
                self.maze_walls[2 * x + 1][y + 1 + k] = wall_value

        self.update_maze_cell_possibilities()

    def update_maze_cell_possibilities(self):
        for i in range(self.maze_dim):
            for j in range(self.maze_dim):
                top_wall = self.maze_walls[2 * i + 1][j + 1]
                right_wall = self.maze_walls[2 * i + 2][j]
                bottom_wall = self.maze_walls[2 * i + 1][j]
                left_wall = self.maze_walls[2 * i][j]
                wall_values = [top_wall, right_wall, bottom_wall, left_wall]

                top_wall_unknown = 1 if top_wall == -1 else 0
                right_wall_unknown = 1 if right_wall == -1 else 0
                bottom_wall_unknown = 1 if bottom_wall == -1 else 0
                left_wall_unknown = 1 if left_wall == -1 else 0
                num_unknown_walls = top_wall_unknown + right_wall_unknown + bottom_wall_unknown + left_wall_unknown

                # If the robot knows that a space is srrounded by three walls
                # but doesn't know about the 4th, then the 4th must be an
                # openning.
                if num_unknown_walls == 1 and sum(wall_values) == 2:
                    self.maze_cell_possibilities[i][j] = 1
                    if top_wall == -1:
                        self.maze_walls[2 * i + 1][j + 1] = 0
                    elif right_wall == -1:
                        self.maze_walls[2 * i + 2][j] = 0
                    elif bottom_wall == -1:
                        self.maze_walls[2 * i + 1][j] = 0
                    elif left_wall == -1:
                        self.maze_walls[2 * i][j] = 0
                else:
                    self.maze_cell_possibilities[i][j] = 2 ** num_unknown_walls
                    if 0 not in wall_values:
                        self.maze_cell_possibilities[i][j] -= 1

        self.check_goal_walls()

    def check_goal_walls(self):
        if self.goal_location != None:
            return
        dim = self.maze_dim
        goal_wall_coordinates = []
        goal_wall_coordinates.append((dim + 1, dim / 2 + 1))
        goal_wall_coordinates.append((dim + 2, dim / 2))
        goal_wall_coordinates.append((dim + 2, dim / 2 - 1))
        goal_wall_coordinates.append((dim + 1, dim / 2 - 1))
        goal_wall_coordinates.append((dim - 1, dim / 2 - 1))
        goal_wall_coordinates.append((dim - 2, dim / 2 - 1))
        goal_wall_coordinates.append((dim - 2, dim / 2))
        goal_wall_coordinates.append((dim - 1, dim / 2 + 1))

        vals = []
        for i, j in goal_wall_coordinates:
            vals.append(self.maze_walls[i][j])

        if 0 in vals:
            # The goal openning has been found.
            oppenning_index = vals.index(0)
        elif len([x for x in vals if x != -1]) == 7:
            # All 7 walls surrounding the goal have been found.
            oppenning_index = vals.index(-1)
        else:
            return

        if oppenning_index in [0,1]:
            self.goal_location = (dim / 2, dim / 2)
        elif oppenning_index in [2,3]:
            self.goal_location = (dim / 2, dim / 2 - 1)
        elif oppenning_index in [4,5]:
            self.goal_location = (dim / 2 - 1, dim / 2 - 1)
        elif oppenning_index in [6,7]:
            self.goal_location = (dim / 2 - 1, dim / 2)

        for k in range(len(goal_wall_coordinates)):
            i, j = goal_wall_coordinates[k]
            if k == oppenning_index:
                self.maze_walls[i][j] = 0
            else:
                self.maze_walls[i][j] = 1

    def center_of_largest_unknown_area(self):
        max_unscaled_uncertainty = max([max(column) for column in self.maze_cell_possibilities])
        scaled_uncertainties = self.maze_cell_possibilities
        for i in range(self.maze_dim):
            for j in range(self.maze_dim):
                if self.maze_cell_possibilities == max_unscaled_uncertainty:
                    search_radius = min(i, j, self.maze_dim - 1 - i, self.maze_dim - 1 - j)
                    if search_radius > 0:
                        for r in range(search_radius):
                            vals = []
                            vals.append(self.maze_cell_possibilities[i][j+r])
                            vals.append(self.maze_cell_possibilities[i+r][j+r])
                            vals.append(self.maze_cell_possibilities[i+r][j])
                            vals.append(self.maze_cell_possibilities[i+r][j-r])
                            vals.append(self.maze_cell_possibilities[i][j-r])
                            vals.append(self.maze_cell_possibilities[i-r][j-r])
                            vals.append(self.maze_cell_possibilities[i-r][j])
                            vals.append(self.maze_cell_possibilities[i-r][j+r])
                            if min(vals) < max_unscaled_uncertainty:
                                scaled_uncertainties[i][j] *= r
                                break
        max_scaled_uncertainty = max([max(column) for column in scaled_uncertainties])
        peak_locations = []
        for i in range(self.maze_dim):
            for j in range(self.maze_dim):
                if self.maze_cell_possibilities == max_scaled_uncertainty:
                    peak_locations.append((i, j))
        closest_peak = peak_locations[0]
        if len(peak_locations) > 1:
            x, y = self.location
            for k in range(len(peak_locations)):
                dist_a = math.sqrt((x-closest_peak[0])**2 + (y-closest_peak[1])**2)
                dist_b = math.sqrt((x-peak_locations[k][0])**2 + (y-peak_locations[k][1])**2)
                if dist_a > dist_b:
                    closest_peak = peak_locations[k]
        return closest_peak

    def convert_maze_map_to_graph(self, location, target):
        # Tomorrow
        # TODO: Figure out how to convert the current knowledge of the maze into a graph.
        pass

    def best_path_through_graph(self, graph):
        # Tomorrow
        ''' Djikstra's algorithm '''
        # TODO: Implement Djikstra's algorithm.
        pass

    def convert_path_to_steps(self, path):
        heading = self.heading
        x, y = path.pop(0)
        steps = []
        for node_x, node_y in path:
            if x == node_x:
                if node_y - y > 0:
                    if heading == 'up':
                        rotation = 0
                    elif heading == 'left':
                        rotation = 90
                    elif heading == 'right':
                        rotation = -90
                else:
                    if heading == 'down':
                        rotation = 0
                    elif heading == 'left':
                        rotation = -90
                    elif heading == 'right':
                        rotation = 90
            else:
                if node_x - x > 0:
                    if heading == 'right':
                        rotation = 0
                    elif heading == 'up':
                        rotation = 90
                    elif heading == 'down':
                        rotation = -90
                else:
                    if heading == 'left':
                        rotation = 0
                    elif heading == 'up':
                        rotation = -90
                    elif heading == 'down':
                        rotation = 90
            movement = max(abs(node_x - x), abs(node_y - y))
            if movement > 3:
                movement = 3
            steps.append((rotation, movement))
            heading = self.update_direction(heading, rotation)
            x, y = x + value_inside_range(node_x - x, -3, 3), y + value_inside_range(node_y - y, -3, 3)

        return steps

    def value_inside_range(value, min, max):
        if value < min:
            return min
        elif value > max
            return max
        else
            return value

    def update_direction(self, heading, rotation):
        if rotation == 0:
            return heading
        elif rotation == -90:
            if heading == 'up':
                return 'left'
            elif heading == 'right':
                return 'up'
            elif heading == 'down':
                return 'right'
            elif heading == 'left':
                return 'down'
        elif rotation == 90:
            if heading == 'up':
                return 'right'
            elif heading == 'right':
                return 'down'
            elif heading == 'down':
                return 'left'
            elif heading == 'left':
                return 'up'

    def optimal_path_has_been_found(self):
        # Tomorrow
        # TODO: Figure out how to determine if the optimal path has been found.

        if self.goal_location == None:
            return False

        shortest_known_path = ...
        shortest_possible_path = ...
        return len(shortest_known_path) <= len(shortest_possible_path)
