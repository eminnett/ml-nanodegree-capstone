from mapper import Mapper
import math
import copy
import operator

class Navigator(object):
    '''
    The Navigator encapsulates the core Robot logic and is responsible for
    the following:
        - Maintaining the current loctation and heading values on behalf of
            for the Robot given the movements and rotations issued while
            exploring.
        - Deciding the best rotation and movement to make while exploring
            the maze given the current state of the Navigator's Mapper instance.
        - Finding the optimal path through the maze.
    '''
    def __init__(self, maze_dim):
        self.maze_dim = maze_dim

        self.start_location = (0,0)
        self.location = self.start_location
        self.heading = 'up'
        self.latest_sensor_reading = None

        self.optimal_path = None
        self.optimal_steps = None
        self.goal_visited = False

        self.take_additional_steps = False
        self.additional_step_instructions = []
        self.nodes_visited = [self.start_location]
        self.take_second_step = False
        self.second_step_instructions = None

        self.mapper = Mapper(maze_dim)

    def update_map(self, sensors):
        '''
        Save the latest sensor readings and the mapper to update its knowledge
        about the mze walls.
        '''
        self.latest_sensor_reading = sensors
        self.mapper.update_wall_knowledge(self.location, self.heading, sensors)

    def explore(self):
        '''
        Decide the rotation and movement the robot should make in order to
        maximise knowledge of the maze.
        '''
        if self.mapper.goal_location == self.location and not self.goal_visited:
            self.goal_visited = True

        step = self.calculate_next_step()
        rotation, movement = step

        print "{} ||| {} | {} ||| {}".format(self.latest_sensor_reading,
            self.location, self.heading, step)
        self.mapper.pretty_print_maze_map(self.location, self.heading)

        self.location = self.calculate_node(self.location, self.heading, step)
        self.heading = self.calculate_heading(self.heading, rotation)

        self.nodes_visited.append(self.location)

        return step

    def calculate_next_step(self):
        '''
        In order to improve the efficiency of the Robot's traversal of the maze
        and to avoid the Robot getting stuck 'ping-ponging' between two nodes
        indefinitely, the Navigator will attempt to follow the first two steps
        of each calculated path through the maze. The only cases when this isn't
        done is when the path has only one step or when the second step becomes
        invalid as a result of the sensor reaidngs following the first step.

        When attempting to calculate a new path, the target node is either the
        closest node with the greatest uncertainty or the goal node if its
        location is known, but has not yet been visited.
        '''
        loc = self.location
        heading = self.heading

        if self.take_additional_steps:
            next_step = self.additional_step_instructions.pop()
            next_node = self.calculate_node(loc, heading, next_step)
            if next_node != None and self.move_is_valid(loc, next_node):
                self.take_additional_steps = len(self.additional_step_instructions) > 0
                return next_step
            else:
                self.take_additional_steps = False

        second_step_node = None
        second_step = self.second_step_instructions

        if self.take_second_step and second_step != None:
            second_step_node = self.calculate_node(loc, heading, second_step)

        if second_step_node != None and self.move_is_valid(loc, second_step_node):
            self.take_second_step = False
            return second_step

        # Navigate to the location of the maze with least knowledge.
        target = self.closest_least_certain_node()
        # If the goal has been found, but not yet visited, go there instead.
        if not self.goal_visited and self.mapper.goal_found():
            target = self.mapper.goal_location
        maze_graph = self.convert_maze_map_to_graph()
        path = self.best_path_through_graph(maze_graph, loc, target)
        steps = self.convert_path_to_steps(path, heading)

        repeat_length = self.check_for_repeats_in_visited_nodes()
        if repeat_length > 0:
            self.take_additional_steps = True
            self.additional_step_instructions = steps[1:repeat_length + 1]

        if len(steps) > 1:
            self.take_second_step = True
            self.second_step_instructions = steps[1]
        else:
            self.second_step_instructions = None

        return steps[0]

    def check_for_repeats_in_visited_nodes(self):
        '''
        Check to see if the Robot is stuck 'ping-ponging' between a set of nodes. This checks for repeated paths of lengths between 2 and 6. The robot is conidered to be in a stuck state if it follows a path, retraces its steps, and then follows the original path again. It is assumed that if this happens, the Robot would continue this pattern indefinitely.
        '''
        loop_lengths = range(2, 6)
        robot_is_stuck = False
        repeat_length = 0
        for length in loop_lengths:
            first_path  = self.nodes_visited[-length:]
            second_path = self.nodes_visited[-length * 2 + 1:-length + 1]
            second_path.reverse()
            third_path  = self.nodes_visited[-length * 3 + 2:-length * 2 + 2]
            if first_path == second_path and second_path == third_path:
                repeat_length = length
                break

        return repeat_length


    def closest_least_certain_node(self):
        '''
        Find the node with the greatest uncertainty (greatest number of
        possible shapes) that is closest to the current location.
        '''
        uncertainties = self.mapper.cell_possibilities
        max_uncertainty = max([max(column) for column in uncertainties])
        peak_locations = []
        for i in range(self.maze_dim):
            for j in range(self.maze_dim):
                if uncertainties[i][j] == max_uncertainty:
                    peak_locations.append((i, j))
        closest_peak = peak_locations[0]
        if len(peak_locations) > 1:
            loc = self.location
            for k in range(len(peak_locations)):
                dist_a = self.distance_between_nodes(loc, closest_peak)
                dist_b = self.distance_between_nodes(loc, peak_locations[k])
                if dist_a > dist_b:
                    closest_peak = peak_locations[k]
        return closest_peak

    def convert_maze_map_to_graph(self, fastest_route = False, treat_unknown_as_walls = False):
        '''
        Convert the maze map to an undirected graph.
        - If fastest_route, allow the path to include steps with maximum strides
            even if this means moving past unvisited nodes.
        - If treat_unknown_as_walls, prevent the path from passing between nodes
            when the state of the wall / opening between them is unknown.
        '''
        graph = {}
        open_list = set([self.start_location])

        while len(open_list) > 0:
            # Pop the next element of the open_list and set it as the current
            # location.
            location = open_list.pop()
            # If the current location is a key in the graph, move to the next
            # iteration.
            if location in graph.keys():
                next
            else:
                graph[location] = []
            # From current location, add all valid movements from 1-3 in all
            # four directions to the graph and the open_list.
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
                        # Unless the path should include the fastest route,
                        # ensure that the graph does not allow skipping over
                        # unexplored nodes. This helps improve the efficacy
                        # of exploration.
                        if not fastest_route and self.mapper.cell_possibilities[tx][ty] > 1:
                            break
                    else:
                        break

        return graph

    def move_is_valid(self, location, target, treat_unknown_as_walls = False):
        '''
        Will moving from location to target, given the current knowledge of the
        maze, result in hitting a wall?
        - If treat_unknown_as_walls, an attempt to move from location to target
            through a wall / openning of unknown state is considered invalid.
        '''
        valid_move = True
        x, y   = location
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
        Use Dijkstra's algorithm to find the fastest path from start to target
        through the the given undirected graph.
        '''
        optimal_path = []

        # Make sure the target is in the graph
        if target in graph.keys():
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

            optimal_path.append(target)
            current_node = target
            # Construct the optimal path by following the gradient of path costs
            # from the goal to the start.
            while start not in optimal_path:
                current_node = sorted(graph[current_node], key=cost_for_node)[0]
                optimal_path = [current_node] + optimal_path

        return optimal_path

    def convert_path_to_steps(self, path, initial_heading):
        '''
        Convert the given path to a list of step instructions
        (rotation, movement) given the initial heading.
        '''
        start = path.pop(0)
        heading = initial_heading
        steps = []
        deltas = self.convert_path_to_deltas_max_3(start, path)
        for delta_x, delta_y in deltas:
            up    = heading  == 'up'
            right = heading  == 'right'
            down  = heading  == 'down'
            left  = heading  == 'left'
            rotation = 0
            if ((up and delta_y < 0) or (right and delta_x < 0) or
                    (down and delta_y > 0) or (left and delta_x > 0)):
                movement = -max(abs(delta_x), abs(delta_y))
            else:
                if delta_y == 0:
                    if delta_x > 0:
                        if up:
                            rotation = 90
                        elif down:
                            rotation = -90
                    else:
                        if up:
                            rotation = -90
                        elif down:
                            rotation = 90
                else:
                    if delta_y > 0:
                        if left:
                            rotation = 90
                        elif right:
                            rotation = -90
                    else:
                        if left:
                            rotation = -90
                        elif right:
                            rotation = 90
                movement = max(abs(delta_x), abs(delta_y))
            steps.append((rotation, movement))
            heading = self.calculate_heading(heading, rotation)

        return steps

    def convert_path_to_deltas_max_3(self, start, path):
        '''
        Break down the path to the x/y difference between each node in the
        path with a maximum chnage of 3. This will ensure that maximum movement
        made by the Robot while navigating path is not exceeded.
        '''
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
        '''
        Determine whether the optimal path through the maze has been found.
        If this is the first time the optimal path has been found, save it.
        '''
        if not self.mapper.goal_found():
            return False
        goal_location = self.mapper.goal_location

        # print "Goal Location is: {}".format(goal_location)

        if self.optimal_path != None:
            return True

        known_maze_graph = self.convert_maze_map_to_graph(True, True)
        if goal_location not in known_maze_graph.keys():
            print "Goal not yet navigable!"
            return False

        open_maze_graph = self.convert_maze_map_to_graph(True, False)

        # Compare the best path through the maze assuming all unknown walls
        # are walls vs all unknown walls are opennings. If the path lengths are
        # the same, the optimal path has been found.
        shortest_known_path = self.best_path_through_graph(known_maze_graph,
                                            self.start_location, goal_location)
        shortest_possible_path = self.best_path_through_graph(open_maze_graph,
                                            self.start_location, goal_location)
        optimal_path_found = len(shortest_known_path) == len(shortest_possible_path)

        if optimal_path_found:
            self.optimal_path = shortest_known_path
            self.optimal_steps = self.convert_path_to_steps(self.optimal_path, 'up')
        return optimal_path_found

    def print_maze_with_path_costs(self):
        '''
        Print the explored map including the path costs for each explored cell.
        '''
        if not self.mapper.goal_found():
            print "Can not print maze with path costs. The goal has not been found."
            return False
        known_maze_graph = self.convert_maze_map_to_graph(True, True)
        self.best_path_through_graph(known_maze_graph, self.start_location,
                                                self.mapper.goal_location, True)

    # Navigation utility methods:

    def distance_between_nodes(self, a, b):
        ''' Return the distance between the two given nodes. '''
        xa, ya = a
        xb, yb = b
        return math.hypot(xb-xa, yb-ya)

    def calculate_node(self, location, heading, instructions):
        '''
        Given a location and heading, determine which node a set of instructions
        would lead to.
        '''
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
        '''
        Given a heading and rotation, wwhat would the new heading be if the
        rotation was made?
        '''
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
        '''
        Convert the heading and rotation values to booleans.
        '''
        up    = heading  == 'up'
        right = heading  == 'right'
        down  = heading  == 'down'
        left  = heading  == 'left'
        # counterclockwise
        ccw   = rotation == -90
        # forward
        fwd   = rotation == 0
        # clockwise
        cw    = rotation == 90
        return up, right, down, left, ccw, fwd, cw
