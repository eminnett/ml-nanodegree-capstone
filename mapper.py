class Mapper(object):
    def __init__(self, maze_dim):
        self.maze_dim = maze_dim

        self.goal_location = None

        self.walls = self.initial_walls(maze_dim)
        self.cell_possibilities = self.initial_cell_possibilities(maze_dim)

    def goal_found(self):
        return self.goal_location != None

    def initial_walls(self, maze_dim):
        '''
        Construct a two dimensional array that represents all of the walls in
        the maze including exterior walls.
        -1 = unknown
        0 = no wall
        1 = wall
        '''
        walls = [[-1] * maze_dim]
        for i in range(maze_dim):
            walls.append([-1] * (maze_dim + 1))
            walls.append([-1] * maze_dim)

        for i in range(2 * maze_dim + 1):
            for j in range(len(walls[i])):
                if i == 0 or i == maze_dim * 2:
                    # exterior wall
                    walls[i][j] = 1
                elif i % 2 == 1 and (j == 0 or j == len(walls[i]) - 1):
                    # exterior wall
                    walls[i][j] = 1
                elif i == maze_dim and (j == maze_dim / 2 or j == maze_dim / 2 - 1):
                    # inside goal area
                    walls[i][j] = 0
                elif (i == maze_dim - 1 or i == maze_dim + 1) and j == maze_dim / 2:
                    # inside goal area
                    walls[i][j] = 0

        return walls

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

    def update_wall_knowledge(self, location, heading, sensors):
        '''
        Update map of walls and opennings based on current location, heading,
        and sensor readings.
        '''
        x, y = location
        left_reading, forward_reading, right_reading = sensors

        if heading == 'up':
            for i in range(left_reading + 1):
                wall_value = 1 if i == left_reading else 0
                self.walls[2 * (x - i)][y] = wall_value
            for j in range(forward_reading + 1):
                wall_value = 1 if j == forward_reading else 0
                self.walls[2 * x + 1][y + 1 + j] = wall_value
            for k in range(right_reading + 1):
                wall_value = 1 if k == right_reading else 0
                self.walls[2 * (x + 1 + k)][y] = wall_value

        elif heading == 'right':
            for i in range(left_reading + 1):
                wall_value = 1 if i == left_reading else 0
                self.walls[2 * x + 1][y + 1 + i] = wall_value
            for j in range(forward_reading + 1):
                wall_value = 1 if j == forward_reading else 0
                self.walls[2 * (x + 1 + j)][y] = wall_value
            for k in range(right_reading + 1):
                wall_value = 1 if k == right_reading else 0
                self.walls[2 * x + 1][y - k] = wall_value

        elif heading == 'down':
            for i in range(left_reading + 1):
                wall_value = 1 if i == left_reading else 0
                self.walls[2 * (x + 1 + i)][y] = wall_value
            for j in range(forward_reading + 1):
                wall_value = 1 if j == forward_reading else 0
                self.walls[2 * x + 1][y - j] = wall_value
            for k in range(right_reading + 1):
                wall_value = 1 if k == right_reading else 0
                self.walls[2 * (x - k)][y] = wall_value

        elif heading == 'left':
            for i in range(left_reading + 1):
                wall_value = 1 if i == left_reading else 0
                self.walls[2 * x + 1][y - i] = wall_value
            for j in range(forward_reading + 1):
                wall_value = 1 if j == forward_reading else 0
                self.walls[2 * (x - j)][y] = wall_value
            for k in range(right_reading + 1):
                wall_value = 1 if k == right_reading else 0
                self.walls[2 * x + 1][y + 1 + k] = wall_value

        self.update_cell_possibilities()

    def update_cell_possibilities(self):
        for i in range(self.maze_dim):
            for j in range(self.maze_dim):
                top_wall = self.walls[2 * i + 1][j + 1]
                right_wall = self.walls[2 * i + 2][j]
                bottom_wall = self.walls[2 * i + 1][j]
                left_wall = self.walls[2 * i][j]
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
                    self.cell_possibilities[i][j] = 1
                    if top_wall == -1:
                        self.walls[2 * i + 1][j + 1] = 0
                    elif right_wall == -1:
                        self.walls[2 * i + 2][j] = 0
                    elif bottom_wall == -1:
                        self.walls[2 * i + 1][j] = 0
                    elif left_wall == -1:
                        self.walls[2 * i][j] = 0
                else:
                    self.cell_possibilities[i][j] = 2 ** num_unknown_walls
                    if 0 not in wall_values:
                        self.cell_possibilities[i][j] -= 1

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
            vals.append(self.walls[i][j])

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
                self.walls[i][j] = 0
            else:
                self.walls[i][j] = 1

    def pretty_print_maze_map(self, location, heading, cell_values = None):
        x, y = location
        heading_representation = {'up': '/\\', 'right': '> ', 'down': '\\/', 'left': ' <'}
        vertical_wall_chars = {-1: ':', 0: ' ', 1: '|'}
        horizontal_wall_chars = {-1: '..', 0: '  ', 1: '--'}
        maze_rows = [''] * (2 * self.maze_dim + 1)
        for i in range(2 * self.maze_dim + 1):
            for j in range(len(self.walls[i])):
                if i % 2 == 0:
                    # vertical walls
                    maze_rows[2*j] += '*'
                    maze_rows[2*j + 1] += vertical_wall_chars[self.walls[i][j]]
                else:
                    # horizontal walls
                    maze_rows[2*j] += horizontal_wall_chars[self.walls[i][j]]
                    if 2*j + 1 < len(maze_rows):
                        if cell_values == None:
                            cell_value = '  '
                            if (i - 1) / 2 == x and j == y:
                                cell_value = heading_representation[heading]
                        else:
                            loc = ((i - 1) / 2, j)
                            if loc in cell_values.keys():
                                cell_value = str(cell_values[loc])
                                if len(cell_value) == 1:
                                    cell_value = ' ' + cell_value
                            else:
                                cell_value = '??'
                        maze_rows[2*j + 1] += cell_value
            if i % 2 == 0:
                maze_rows[-1] += '*'
        maze_rows.reverse()
        maze_drawing = ''
        for row in maze_rows:
            maze_drawing += row + "\n"
        print maze_drawing
