from mapper import Mapper
import math
import copy
import operator

class Navigator(object):
    def __init__(self, maze_dim):
        self.maze_dim = maze_dim

        self.start_location = (0,0)
        self.location = self.start_location
        self.heading = 'up'
        self.latest_sensor_reading = None

        self.optimal_path = None
        self.optimal_steps = None
        self.goal_visited = False

        self.take_second_step = False
        self.second_step_instructions = None

        self.mapper = Mapper(maze_dim)

    def update_map(self, sensors):
        self.latest_sensor_reading = sensors
        self.mapper.update_wall_knowledge(self.location, self.heading, sensors)

    def explore(self):

        if self.mapper.goal_location == self.location and not self.goal_visited:
            self.goal_visited = True

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
            if not self.goal_visited and self.mapper.goal_found():
                target = self.mapper.goal_location
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

        print "{} ||| {} | {} ||| {} | {}".format(self.latest_sensor_reading, self.location, self.heading, rotation, movement)
        self.mapper.pretty_print_maze_map(self.location, self.heading)


        self.location = self.calculate_node(self.location, self.heading, (rotation, movement))
        self.heading = self.calculate_heading(self.heading, rotation)

        return rotation, movement

    def closest_least_certain_node(self):
        uncertainties = self.mapper.cell_possibilities
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
                        if not fastest_route and self.mapper.cell_possibilities[tx][ty] > 1:
                            break
                    else:
                        break

        return graph

    def calculate_node(self, location, heading, instructions):
        rotation, movement = instructions
        x, y  = location
        up, right, down, left, ccw, fwd, cw = self.heading_rotation_bools(heading, rotation)
        if (up and ccw) or (down and cw) or (left and fwd):
            x -= movement
        elif (up and fwd) or (right and ccw) or (left and cw):
            y += movement
        elif (up and cw) or (right and fwd) or (down and ccw) :
            x += movement
        elif (right and cw) or (down and fwd) or (left and ccw) :
            y -= movement

        return (x, y)

    def calculate_heading(self, heading, rotation):
        up, right, down, left, ccw, fwd, cw = self.heading_rotation_bools(heading, rotation)
        if fwd:
            return heading
        if (ccw and up) or (cw and down):
            return 'left'
        if (ccw and right) or (cw and left):
            return 'up'
        if (ccw and down) or (cw and up):
            return 'right'
        if (ccw and left) or (cw and right):
            return 'down'

    def heading_rotation_bools(self, heading, rotation):
        up    = heading  == 'up'
        right = heading  == 'right'
        down  = heading  == 'down'
        left  = heading  == 'left'
        ccw   = rotation == -90
        fwd   = rotation == 0
        cw    = rotation == 90
        return up, right, down, left, ccw, fwd, cw

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
                    if self.mapper.walls[2 * (x + i + 1)][y] in wall_values:
                        valid_move = False
                        break
            else:
                for i in range(x - tx):
                    if self.mapper.walls[2 * (x - i)][y] in wall_values:
                        valid_move = False
                        break
        else:
            if ty < 0 or ty >= self.maze_dim:
                valid_move = False
            elif y < ty:
                for i in range(ty - y):
                    if self.mapper.walls[2 * x + 1][y + i + 1] in wall_values:
                        valid_move = False
                        break
            else:
                for i in range(y - ty):
                    if self.mapper.walls[2 * x + 1][y - i] in wall_values:
                        valid_move = False
                        break

        return valid_move


    def best_path_through_graph(self, graph, start, target, print_path_costs = False):
        '''
        Use Djikstra's algorithm to find the fastest path from start to target
        through the the given undirected graph.
        '''

        # Assign to every node a tentative distance value: set it to zero for
        # our initial node and to infinity for all other nodes.

        largest_possible_cost = self.maze_dim ** 2

        path_costs = {}

        # Used for sorting by path cost.
        cost_for_node = lambda n: path_costs[n]

        for node in graph.keys():
            path_costs[node] = largest_possible_cost
        path_costs[start] = 0

        # Set the initial node as current. Mark all other nodes unvisited.
        # Create a set of all the unvisited nodes called the unvisited set.
        current_node = start
        unvisited_list = copy.copy(graph.keys())

        while len(unvisited_list) > 0:
            # For the current node, consider all of its neighbours and
            # calculate their tentative distances. Compare the newly
            # calculated tentative distance to the current assigned value
            # and assign the smaller one otherwise, keep the current value.

            distance = path_costs[current_node] + 1
            for neighbour in graph[current_node]:
                if path_costs[neighbour] > distance:
                    path_costs[neighbour] = distance

            # When we are done considering all of the neighbors of the current
            # node, mark the current node as visited and remove it from the
            # unvisited set. A visited node will never be checked again.

            unvisited_list.remove(current_node)

            if len(unvisited_list) > 0:
                # Select the unvisited node that is marked with the
                # smallest tentative distance, set it as the new
                # "current node", and go back to the beginning of the loop.
                current_node = sorted(unvisited_list, key=cost_for_node)[0]

        if print_path_costs:
            print 'Path costs for each explored space within the maze:'
            self.mapper.pretty_print_maze_map((0,0), 'up', path_costs)

        optimal_path = [target]
        current_node = target

        # Construct the optimal path by following the gradient of path costs
        # from the goal to the start.
        while start not in optimal_path:
            current_node = sorted(graph[current_node], key=cost_for_node)[0]
            optimal_path = [current_node] + optimal_path

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
            heading = self.calculate_heading(heading, rotation)

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

    def found_optimal_path(self):
        if not self.mapper.goal_found():
            return False
        goal_location = self.mapper.goal_location

        print "Goal Location is: {}".format(goal_location)

        known_maze_graph = self.convert_maze_map_to_graph(True, True)
        if goal_location not in known_maze_graph.keys():
            print "Goal not yet navigable!"
            return False

        open_maze_graph = self.convert_maze_map_to_graph(True, False)

        shortest_known_path = self.best_path_through_graph(known_maze_graph, self.start_location, goal_location)
        shortest_possible_path = self.best_path_through_graph(open_maze_graph, self.start_location, goal_location)
        optimal_path_has_been_found = len(shortest_known_path) <= len(shortest_possible_path)

        if optimal_path_has_been_found:
            self.optimal_path = shortest_known_path
            self.optimal_steps = self.convert_path_to_steps(self.optimal_path, 'up')
        return optimal_path_has_been_found

    def print_maze_with_path_costs(self):
        if not self.mapper.goal_found():
            print "Can not print maze with path costs. The goal has not yet been found."
            return False
        known_maze_graph = self.convert_maze_map_to_graph(True, True)
        self.best_path_through_graph(known_maze_graph, self.start_location, self.mapper.goal_location, True)
