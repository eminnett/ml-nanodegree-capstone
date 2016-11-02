from navigator import Navigator
from random import randint, shuffle, choice
import copy
import math
import os
import errno

class MazeGenerator(object):
    '''
    The MazeGenerator generates a random maze of dimension maze_dim and
    ensures that all spaces are accessible, and as a result, ensuring
    that the maze is solvable.
    '''
    def __init__(self, maze_dim):
        self.maze_dim  = maze_dim
        self.navigator = Navigator(maze_dim)

        self.update_walls_initial_conditions()
        self.define_goal()

        self.generate_maze()

    def update_walls_initial_conditions(self):
        '''
        Make sure the initial conditions for the edges around the start cell
        are met.
        '''
        # The edge to the right of the start position must be a wall.
        self.navigator.mapper.walls[2][0] = 1
        # The edge above the start position must be an openning.
        self.navigator.mapper.walls[1][1] = 0

    def define_goal(self):
        '''
        Randomly set the goal entrance to one of the 8 possible locations.
        '''
        entrance_coord = self.navigator.mapper.goal_wall_coords[randint(0,7)]
        x, y = entrance_coord
        self.navigator.mapper.walls[x][y] = 0
        self.navigator.mapper.check_goal_walls()

    def generate_maze(self):
        '''
        Generates the maze using a modified version of the
        'Recursive Division Method' as oulined on the Maze generation algorithm
        Wikipedia page. https://en.wikipedia.org/wiki/Maze_generation_algorithm
        '''
        walls = self.navigator.mapper.walls
        traverse_vertically = bool(randint(0,1))
        all_undecided_indices = self.all_undecided_wall_indices()
        while len(all_undecided_indices) > 0:
            undecided_indices = self.all_undecided_walls_by_direction(traverse_vertically)
            if len(undecided_indices) > 0:
                initial_edge = choice(undecided_indices)
                selected_indices = self.find_consecutive_undecided_indices(
                                            traverse_vertically, initial_edge)
                self.decide_edge_state(selected_indices)
            traverse_vertically = not traverse_vertically
            all_undecided_indices = self.all_undecided_wall_indices()


    def all_undecided_wall_indices(self):
        '''
        Find all wall indices that represent edges with an undecided state.
        '''
        vertical_undecided = self.all_undecided_walls_by_direction(True)
        horizontal_undecided = self.all_undecided_walls_by_direction(False)

        return vertical_undecided + horizontal_undecided

    def all_undecided_walls_by_direction(self, vertical):
        '''
        Find all wall indices that represent either vertical or horizontal
        edges with an undecided state.
        '''
        walls = self.navigator.mapper.walls
        undecided_indices = []
        for i in range(len(walls)):
            if (i % 2 == 0 and vertical) or (i % 2 == 1 and not vertical):
                for j in range(len(walls[i])):
                    if walls[i][j] == -1:
                        undecided_indices.append((i,j))

        return undecided_indices

    def find_consecutive_undecided_indices(self, traverse_vertically, location):
        '''
        Given an undecided edge within the maze, select neighbouring edges
        that are either horizontal or vertical that are also undecided.
        '''
        walls = self.navigator.mapper.walls
        x, y = location
        undecided_indices = [location]
        edge_not_reached = True
        if traverse_vertically:
            while edge_not_reached:
                edge_not_reached = self.continue_selecting_indices(
                            traverse_vertically, x, y, True, undecided_indices)
                if edge_not_reached:
                    y += 1
                    undecided_indices.append((x,y))

            edge_not_reached = True
            x, y = location
            while edge_not_reached:
                edge_not_reached = self.continue_selecting_indices(
                            traverse_vertically, x, y, False, undecided_indices)
                if edge_not_reached:
                    y -= 1
                    undecided_indices.append((x,y))
        else:
            while edge_not_reached:
                edge_not_reached = self.continue_selecting_indices(
                            traverse_vertically, x, y, True, undecided_indices)
                if edge_not_reached:
                    x += 2
                    undecided_indices.append((x,y))

            edge_not_reached = True
            x, y = location
            while edge_not_reached:
                edge_not_reached = self.continue_selecting_indices(
                            traverse_vertically, x, y, False, undecided_indices)
                if edge_not_reached:
                    x -= 2
                    undecided_indices.append((x,y))


        return undecided_indices

    def continue_selecting_indices(self, traverse_vertically, x, y, increasing, undecided_indices):
        '''
        Given the x, y coordinates for an edge within the maze, whether the
        coordiantes are increasing or decreasing, whether the edges are
        horizontal or vertical and the list of already selected edges, should
        the algorithm continue selecting more edges? Each selection step in the
        algorithm should only select a maximum of 4 undecided edges. This helps
        minimise very long coridors within the maze with very few opennings.
        '''
        if len(undecided_indices) >= 5 :
            return False
        walls = self.navigator.mapper.walls
        if traverse_vertically:
            index_in_bounds = y + 1 < len(walls[x]) if increasing else y >= 0
        else:
            index_in_bounds = x + 2 < len(walls) if increasing else x - 2 >= 0

        if not index_in_bounds:
            return False

        if traverse_vertically:
            if increasing:
                return walls[x - 1][y + 1] == -1 and walls[x + 1][y + 1] == -1
            else:
                return walls[x - 1][y] == -1 and walls[x + 1][y] == -1
        else:
            if increasing:
                return walls[x + 1][y] == -1 and walls[x + 1][y - 1] == -1
            else:
                return walls[x - 1][y] == -1 and walls[x - 1][y - 1] == -1

    def decide_edge_state(self, undecided_indices):
        '''
        For the given set of edges with undecided state, set the state as an
        openning for one of the dges randomly selected and ste the
        remainder as walls.
        '''
        walls = self.navigator.mapper.walls

        shuffle(undecided_indices)
        x, y = undecided_indices.pop()
        walls[x][y] = 0

        for x, y in undecided_indices:
            walls[x][y] = 1

    def calculate_benchmark_score(self, upper = True):
        '''
        Given the shape of the maze, what is the becnhmark score
        (upper or lower)?
        '''
        self.navigator.found_optimal_path()
        path_length = len(self.navigator.optimal_steps)
        exploration_score = self.maze_dim ** 2 / 30.
        if upper:
            exploration_score *= 2

        return path_length + exploration_score

    def save_maze_to_file(self, file_name_base):
        '''
        Write the maze to a text file as per the requirements outlined for
        this project.
        '''
        walls = self.navigator.mapper.walls
        dim = self.maze_dim
        data = "{}\n".format(dim)
        for i in range(dim):
            row_data = []
            for j in range(dim):
                # The values  have to be reversed to meet the expectation that
                # 0 is a wall and 1 is an openning.
                t = (walls[2 * i + 1  ][j + 1] + 1) % 2
                r = (walls[2 * (i + 1)][j    ] + 1) % 2
                b = (walls[2 * i + 1  ][j    ] + 1) % 2
                l = (walls[2 * i      ][j    ] + 1) % 2
                cell_value = t + 2 * r + 4 * b + 8 * l
                row_data.append(str(cell_value))

            data += "{}\n".format(','.join(row_data))

        upper_benchmark = str(self.calculate_benchmark_score())
        lower_benchmark = str(self.calculate_benchmark_score(False))

        file_name = file_name_base + "_ub-" + upper_benchmark + "_lb-" + lower_benchmark + ".txt"

        if not os.path.exists(os.path.dirname(file_name)):
            try:
                os.makedirs(os.path.dirname(file_name))
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        f = open(file_name, "w")
        f.write(data)
        f.close()




if __name__ == '__main__':
    dimensions = [2 * d for d in range(6,17)]
    mazes_per_dimension = 100

    for maze_dim in dimensions:
        for i in range(mazes_per_dimension):
            if i < 10:
                i = '0{}'.format(i)
            print "Generating maze {} : {}".format(maze_dim, i)
            generator = MazeGenerator(maze_dim)
            base_file_name = "./generated_mazes/{}/maze_{}".format(maze_dim, i)
            generator.save_maze_to_file(base_file_name)
