import numpy as np
import math
import pudb
import copy

# Maze 1: Score: 20.667 (17 steps)
# Maze 2: Score: 28.567 (22 steps)
# Maze 3: Score: 31.767 (25 steps)

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
        self.take_second_step = False
        self.second_step_instructions = None
        self.goal_visited = False
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
        maze_walls = [[-1] * maze_dim]
        for i in range(maze_dim):
            maze_walls.append([-1] * (maze_dim + 1))
            maze_walls.append([-1] * maze_dim)

        for i in range(2 * maze_dim + 1):
            for j in range(len(maze_walls[i])):
                if i == 0 or i == maze_dim * 2:
                    # exterior wall
                    maze_walls[i][j] = 1
                elif i % 2 == 1 and (j == 0 or j == len(maze_walls[i]) - 1):
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
        cell_possibilities = []
        for n in range(maze_dim):
            cell_possibilities.append([15] * maze_dim)

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

        if self.goal_location == self.location and not self.goal_visited:
            self.goal_visited = True

        self.update_maze_walls(sensors)

        if self.exploring and self.goal_visited and self.optimal_path_has_been_found():
            self.exploring = False
            self.race_time_step = 0
            explored_maze_graph = self.convert_maze_map_to_graph(True, True)
            optimal_path = self.best_path_through_graph(explored_maze_graph, (0,0), self.goal_location, True)
            self.optimal_steps = self.convert_path_to_steps(optimal_path, 'up')
            print "Best found path: {}".format(optimal_path)
            print "Best found path steps count: {}".format(len(self.optimal_steps))
            return 'Reset', 'Reset'


        second_step_node = None
        if self.take_second_step and self.second_step_instructions != None:
            second_step_node = self.calculate_node(self.location, self.heading, self.second_step_instructions)

        if second_step_node != None and self.move_is_valid(self.location, second_step_node):
            rotation = self.second_step_instructions[0]
            movement = self.second_step_instructions[1]
            self.take_second_step = False
        else:
            # Navigate to the location of the maze with least knowledge.
            target = self.closest_least_certain_node()
            # If the goal has been found, but not yet visited, go there instead.
            if not self.goal_visited and self.goal_location != None:
                target = self.goal_location
            maze_graph = self.convert_maze_map_to_graph()
            path = self.best_path_through_graph(maze_graph, self.location, target)
            steps = self.convert_path_to_steps(path, self.heading)

            if len(path) > 1:
                self.take_second_step = True
                self.second_step_instructions = steps[1]
            else:
                self.second_step_instructions = None

            rotation = steps[0][0]
            movement = steps[0][1]

        print "{} ||| {} | {} ||| {} | {}".format(sensors, self.location, self.heading, rotation, movement)
        self.pretty_print_maze_map(self.location, self.heading)

        self.location = self.calculate_node(self.location, self.heading, (rotation, movement))
        self.heading = self.update_direction(self.heading, rotation)

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
                self.maze_walls[2 * (x - i)][y] = wall_value
            for j in range(forward_reading + 1):
                wall_value = 1 if j == forward_reading else 0
                self.maze_walls[2 * x + 1][y + 1 + j] = wall_value
            for k in range(right_reading + 1):
                wall_value = 1 if k == right_reading else 0
                self.maze_walls[2 * (x + 1 + k)][y] = wall_value

        elif self.heading == 'right':
            for i in range(left_reading + 1):
                wall_value = 1 if i == left_reading else 0
                self.maze_walls[2 * x + 1][y + 1 + i] = wall_value
            for j in range(forward_reading + 1):
                wall_value = 1 if j == forward_reading else 0
                self.maze_walls[2 * (x + 1 + j)][y] = wall_value
            for k in range(right_reading + 1):
                wall_value = 1 if k == right_reading else 0
                self.maze_walls[2 * x + 1][y - k] = wall_value

        elif self.heading == 'down':
            for i in range(left_reading + 1):
                wall_value = 1 if i == left_reading else 0
                self.maze_walls[2 * (x + 1 + i)][y] = wall_value
            for j in range(forward_reading + 1):
                wall_value = 1 if j == forward_reading else 0
                self.maze_walls[2 * x + 1][y - j] = wall_value
            for k in range(right_reading + 1):
                wall_value = 1 if k == right_reading else 0
                self.maze_walls[2 * (x - k)][y] = wall_value

        elif self.heading == 'left':
            for i in range(left_reading + 1):
                wall_value = 1 if i == left_reading else 0
                self.maze_walls[2 * x + 1][y - i] = wall_value
            for j in range(forward_reading + 1):
                wall_value = 1 if j == forward_reading else 0
                self.maze_walls[2 * (x - j)][y] = wall_value
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

    def closest_least_certain_node(self):
        uncertainties = self.maze_cell_possibilities
        max_uncertainty = max([max(column) for column in uncertainties])
        peak_locations = []
        for i in range(self.maze_dim):
            for j in range(self.maze_dim):
                if uncertainties[i][j] == max_uncertainty:
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

    def convert_maze_map_to_graph(self, fastest_route = False, treat_unknown_as_walls = False):
        graph = {}
        open_list = set([(0,0)])

        while len(open_list) > 0:
            # Pop the next element of the open_list and set it as the current location.
            location = open_list.pop()
            # If the current location is a key in the graph, move to the next iteration.
            if location in graph.keys():
                next
            else:
                graph[location] = []
            # From current location, add all valid movements from 1-3 in all four
            # directions to the graph and the open_list, if fastest_route is False, don't allow the
            # robot to move past an unexplored space
            x, y = location
            for direction in ['up', 'right', 'down', 'left']:
                for i in range(1,4):
                    tx, ty = x, y
                    if direction == 'up':
                        tx  = x + i
                    elif direction == 'right':
                        ty  = y + i
                    elif direction == 'down':
                        tx  = x - i
                    elif direction == 'left':
                        ty  = y - i

                    target = (tx, ty)

                    if self.move_is_valid(location, target, treat_unknown_as_walls):
                        graph[location].append(target)
                        if target not in graph.keys():
                            open_list.add(target)
                        if not fastest_route and self.maze_cell_possibilities[tx][ty] > 1:
                            break
                    else:
                        break

        return graph

    def calculate_node(self, location, heading, instructions):
        rotation, movement = instructions
        x, y  = location
        up    = heading  == 'up'
        right = heading  == 'right'
        down  = heading  == 'down'
        left  = heading  == 'left'
        ccw   = rotation == -90
        fwd   = rotation == 0
        cw    = rotation == 90
        if (up and ccw) or (down and cw) or (left and fwd):
            x -= movement
        elif (up and fwd) or (right and ccw) or (left and cw):
            y += movement
        elif (up and cw) or (right and fwd) or (down and ccw) :
            x += movement
        elif (right and cw) or (down and fwd) or (left and ccw) :
            y -= movement

        return (x, y)

    def move_is_valid(self, location, target, treat_unknown_as_walls = False):
        '''
        Will moving from location to target given the current knowledge of the
        maze result in hitting a wall?
        '''
        valid_move = True
        x, y = location
        tx, ty = target

        wall_values = [1]
        if treat_unknown_as_walls:
            wall_values.append(-1)

        if y == ty:
            if tx < 0 or tx >= self.maze_dim:
                valid_move = False
            elif x < tx:
                for i in range(tx - x):
                    if self.maze_walls[2 * (x + i + 1)][y] in wall_values:
                        valid_move = False
                        break
            else:
                for i in range(x - tx):
                    if self.maze_walls[2 * (x - i)][y] in wall_values:
                        valid_move = False
                        break
        else:
            if ty < 0 or ty >= self.maze_dim:
                valid_move = False
            elif y < ty:
                for i in range(ty - y):
                    if self.maze_walls[2 * x + 1][y + i + 1] in wall_values:
                        valid_move = False
                        break
            else:
                for i in range(y - ty):
                    if self.maze_walls[2 * x + 1][y - i] in wall_values:
                        valid_move = False
                        break

        return valid_move


    def best_path_through_graph(self, graph, start, target, final_path = False):
        ''' Djikstra's algorithm '''

        # Let the node at which we are starting be called the initial node. Let the distance of node Y be the distance from the initial node to Y. Dijkstra's algorithm will assign some initial distance values and will try to improve them step by step.

        # Assign to every node a tentative distance value: set it to zero for our initial node and to infinity for all other nodes.

        path_costs = {}
        for node in graph.keys():
            path_costs[node] = float("inf")
        path_costs[start] = 0
        costs_updated = True

        # Set the initial node as current. Mark all other nodes unvisited. Create a set of all the unvisited nodes called the unvisited set.

        current_node = start

        # Repeat Djikstra's algorithm until the the path costs converge to
        # their true value. This has proven to be an important step as each
        # node can be accessed via many different paths.
        while costs_updated == True:
            costs_updated = False
            unvisited_list = copy.copy(graph.keys())
            while len(unvisited_list) > 0:
                # For the current node, consider all of its unvisited neighbors and calculate their tentative distances. Compare the newly calculated tentative distance to the current assigned value and assign the smaller one. For example, if the current node A is marked with a distance of 6, and the edge connecting it with a neighbor B has length 2, then the distance to B (through A) will be 6 + 2 = 8. If B was previously marked with a distance greater than 8 then change it to 8. Otherwise, keep the current value.

                cost = min(path_costs[current_node], min([path_costs[neighbour] + 1 for neighbour in graph[current_node]]))
                path_costs[current_node] = cost
                distance = cost + 1
                for neighbour in graph[current_node]:
                    if path_costs[neighbour] > distance:
                        path_costs[neighbour] = distance
                        costs_updated = True

                # When we are done considering all of the neighbors of the current node, mark the current node as visited and remove it from the unvisited set. A visited node will never be checked again.

                unvisited_list.remove(current_node)

                # Select the unvisited node that is marked with the smallest tentative distance, set it as the new "current node", and go back to the beginning of the loop.

                closest_distance = float("inf")
                for node in unvisited_list:
                    if path_costs[node] < closest_distance:
                        current_node = node

        if final_path:
            print 'Path costs for each explored space within the maze:'
            self.pretty_print_maze_map((0,0), 'up', path_costs)

        optimal_path = [target]
        current_node = target

        # Contrsuct the optimal path by following the gradient of path costs
        # from the goal to the start.
        while start not in optimal_path:
            neighbours = graph[current_node]
            optimal_step = neighbours[0]
            for neighbour in neighbours:
                if path_costs[neighbour] < path_costs[optimal_step]:
                    optimal_step = neighbour
            current_node = optimal_step
            optimal_path = [optimal_step] + optimal_path

        return optimal_path

    def convert_path_to_steps(self, path, heading):
        start = path.pop(0)
        steps = []
        deltas = self.convert_path_to_deltas_max_3(start, path)
        for delta_x, delta_y in deltas:
            rotation = 0
            if (heading == 'up' and delta_y < 0) or (heading == 'right' and delta_x < 0) or (heading == 'down' and delta_y > 0) or (heading == 'left' and delta_x > 0):
                movement = -max(abs(delta_x), abs(delta_y))
            else:
                if delta_y == 0:
                    if delta_x > 0:
                        if heading == 'up':
                            rotation = 90
                        elif heading == 'down':
                            rotation = -90
                    else:
                        if heading == 'up':
                            rotation = -90
                        elif heading == 'down':
                            rotation = 90
                else:
                    if delta_y > 0:
                        if heading == 'left':
                            rotation = 90
                        elif heading == 'right':
                            rotation = -90
                    else:
                        if heading == 'left':
                            rotation = -90
                        elif heading == 'right':
                            rotation = 90
                movement = max(abs(delta_x), abs(delta_y))
            steps.append((rotation, movement))
            heading = self.update_direction(heading, rotation)

        return steps

    def convert_path_to_deltas_max_3(self, start, path):
        x, y = start
        deltas = []
        for node_x, node_y in path:
            if y == node_y:
                step = node_x - x
                while step > 3 or step < -3:
                    if step > 0:
                        deltas.append((3,0))
                        step -= 3
                    else:
                        deltas.append((-3,0))
                        step += 3
                deltas.append((step,0))
            else:
                step = node_y - y
                while step > 3 or step < -3:
                    if step > 0:
                        deltas.append((0,3))
                        step -= 3
                    else:
                        deltas.append((0,-3))
                        step += 3
                deltas.append((0,step))

            x, y = node_x, node_y
        return deltas

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
        if self.goal_location == None:
            return False

        print "Goal Location is: {}".format(self.goal_location)

        known_maze_graph = self.convert_maze_map_to_graph(True, True)
        if self.goal_location not in known_maze_graph.keys():
            print "Goal not yet navigable!"
            return False

        open_maze_graph = self.convert_maze_map_to_graph(True, False)

        shortest_known_path = self.best_path_through_graph(known_maze_graph, (0,0), self.goal_location)
        shortest_possible_path = self.best_path_through_graph(open_maze_graph, (0,0), self.goal_location)
        return len(shortest_known_path) <= len(shortest_possible_path)

    def pretty_print_maze_map(self, location, heading, path_costs = None):
        x, y = location
        heading_representation = {'up': '/\\', 'right': '> ', 'down': '\\/', 'left': ' <'}
        vertical_wall_chars = {-1: ':', 0: ' ', 1: '|'}
        horizontal_wall_chars = {-1: '..', 0: '  ', 1: '--'}
        maze_rows = [''] * (2 * self.maze_dim + 1)
        for i in range(2 * self.maze_dim + 1):
            for j in range(len(self.maze_walls[i])):
                if i % 2 == 0:
                    # vertical walls
                    maze_rows[2*j] += '*'
                    maze_rows[2*j + 1] += vertical_wall_chars[self.maze_walls[i][j]]
                else:
                    # horizontal walls
                    maze_rows[2*j] += horizontal_wall_chars[self.maze_walls[i][j]]
                    if 2*j + 1 < len(maze_rows):
                        if path_costs == None:
                            space = '  '
                            if (i - 1) / 2 == x and j == y:
                                space = heading_representation[heading]
                        else:
                            loc = ((i - 1) / 2, j)
                            if loc in path_costs.keys():
                                space = str(path_costs[loc])
                                if len(space) == 1:
                                    space = ' ' + space
                            else:
                                space = '??'
                        maze_rows[2*j + 1] += space
            if i % 2 == 0:
                maze_rows[-1] += '*'
        maze_rows.reverse()
        maze_drawing = ''
        for row in maze_rows:
            maze_drawing += row + "\n"
        print maze_drawing
