class Mapper(object):
    '''
    The Mapper is responsible for maintaining a map of what is known about
    the maze. This is achieved by storing and upding two sets of two-dimensional
    lists. One stores knowledge about the walls and oppennings in the maze
    along with where in the maze the state of a wall or oppenning is unknown.
    The other two-dimensional list stores information about how many possible
    shapes each cell could have given the current knowledge of the maze walls.
    This second two-dimensional list, in effect, represents the uncertainty
    within the current maze map.
    '''
    def __init__(self, maze_dim):
        self.maze_dim = maze_dim

        self.goal_location = None

        self.walls = self.initial_walls(maze_dim)
        self.cell_possibilities = self.initial_cell_possibilities(maze_dim)

    def goal_found(self):
        ''' Has the goal been found? '''
        return self.goal_location != None

    def initial_walls(self, maze_dim):
        '''
        Construct the initial state for the two-dimensional list that
        represents all of the walls in the maze including exterior walls.
        -1 = unknown
        0  = no wall
        1  = wall

        NB: The nature of storing both horizontal and vertical walls in the same
        two-dimensional list results in slightly unconventional indexing. For
        a given maze_dim, there will be 2 * maze_dim + 1 sets of walls: the
        vertical left exterior walls, maze_dim sets of horizontal walls
        (interior and exterior), maze_dim - 1 sets of interior vertical walls,
        and the right vertical extreior walls. This also results in the
        following additional quirk: lists representing vertical walls will
        have length maze_dim while lists representing horizontal walls will
        have length maze_dim + 1 (because of the exterior walls top and bottom).
        '''
        walls = [[-1] * maze_dim]
        for i in range(maze_dim):
            walls.append([-1] * (maze_dim + 1))
            walls.append([-1] * maze_dim)

        for i in range(2 * maze_dim + 1):
            for j in range(len(walls[i])):
                # One of the 4 sets of exterior walls?
                top    = i % 2 == 1 and j == len(walls[i]) - 1
                right  = i == maze_dim * 2
                bottom = i % 2 == 1 and j == 0
                left   = i == 0
                # One of the four oppennings interior to the goal area?
                goal_top    = i == maze_dim and j == maze_dim / 2
                goal_right  = i == maze_dim + 1 and j == maze_dim / 2
                goal_bottom = i == maze_dim and j == maze_dim / 2 - 1
                goal_left   = i == maze_dim - 1 and j == maze_dim / 2
                if top or right or bottom or left:
                    walls[i][j] = 1
                elif goal_top or goal_right or goal_bottom or goal_left:
                    walls[i][j] = 0

        return walls

    def initial_cell_possibilities(self, maze_dim):
        '''
        Construct the initial state of the two-dimensional list that represents
        the number of possibile shapes each cell could take. It is worth noting
        that corner, edge, and center cells have fewer initial possible shapes
        than other interior cells given the necasary structure of any maze
        (every maze is fully enclosed and the 2x2 goal area at the center of
        the maze has no interior walls).
        '''
        dim = maze_dim
        cell_possibilities = []
        for n in range(dim):
            cell_possibilities.append([15] * dim)

        for i in range(dim):
            for j in range(dim):
                # Is the cell an edge or corner?
                top    = j == dim - 1
                right  = i == dim - 1
                bottom = j == 0
                left   = i == 0
                # Is the cell inside the goal area?
                left_top     = i == dim / 2 - 1 and j == dim / 2
                right_top    = i == dim / 2 and j == dim / 2
                right_bottom = i == dim / 2 and j == dim / 2 - 1
                left_bottom  = i == dim / 2 - 1 and j == dim / 2 - 1
                if top or bottom:
                    #                          corner                  edge
                    cell_possibilities[i][j] = 2 if left or right else 7
                elif left or right:
                    # edge
                    cell_possibilities[i][j] = 7
                elif left_top or right_top or right_bottom or left_bottom:
                    # goal area
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
        '''
        Given the current knowledge of the maze walls, update the
        two-dimensional list of possible cell shapes taking advantage
        of the knowledge that every cell must have at most 3 walls among its
        four sides.
        '''
        for i in range(self.maze_dim):
            for j in range(self.maze_dim):
                top    = self.walls[2 * i + 1][j + 1]
                right  = self.walls[2 * i + 2][j    ]
                bottom = self.walls[2 * i + 1][j    ]
                left   = self.walls[2 * i    ][j    ]
                wall_values = [top, right, bottom, left]

                top_unknown    = 1 if top    == -1 else 0
                right_unknown  = 1 if right  == -1 else 0
                bottom_unknown = 1 if bottom == -1 else 0
                left_unknown   = 1 if left   == -1 else 0
                num_unknown    = (top_unknown + right_unknown +
                                  bottom_unknown + left_unknown)

                # If the robot knows that a space is srrounded by three walls
                # but doesn't know about the 4th, then the 4th must be an
                # openning.
                if num_unknown == 1 and sum(wall_values) == 2:
                    self.cell_possibilities[i][j] = 1
                    if top == -1:
                        self.walls[2 * i + 1][j + 1] = 0
                    elif right == -1:
                        self.walls[2 * i + 2][j] = 0
                    elif bottom == -1:
                        self.walls[2 * i + 1][j] = 0
                    elif left == -1:
                        self.walls[2 * i][j] = 0
                else:
                    self.cell_possibilities[i][j] = 2 ** num_unknown
                    if 0 not in wall_values:
                        self.cell_possibilities[i][j] -= 1

        self.check_goal_walls()

    def check_goal_walls(self):
        '''
        Check to see if the goal entrance has been discovered. If either the
        goal oppenning has been found or all 7 goal walls have been found then
        the remaining wall locations or oppenning can be inferred respectively.
        '''
        if self.goal_location != None:
            return
        dim = self.maze_dim
        goal_wall_coordinates = [(dim + 1, dim / 2 + 1), (dim + 2, dim / 2),
                                 (dim + 2, dim / 2 - 1), (dim + 1, dim / 2 - 1),
                                 (dim - 1, dim / 2 - 1), (dim - 2, dim / 2 - 1),
                                 (dim - 2, dim / 2), (dim - 1, dim / 2 + 1)]

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
        '''
        Print and Ascii drawing of the maze depicting what is known
        (line and space) and what is unknown (dots). If vallues are given
        for the cells (in the form of a dictionary with the coordinate tuples
        as the keys) then those values will be displayed. If cell values are to
        be displayed but no value is known for a given cell, its value will
        be '??'.
        '''
        x, y = location
        heading_representation = {'up': '/\\', 'right': '> ',
                                  'down': '\\/', 'left': ' <'}
        vertical_wall_chars    = {-1: ':',  0: ' ',  1: '|'}
        horizontal_wall_chars  = {-1: '..', 0: '  ', 1: '--'}

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
