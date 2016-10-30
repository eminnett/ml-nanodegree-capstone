from navigator import Navigator

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

        self.navigator = Navigator(maze_dim)

        self.racing = False
        self.race_time_step = 0

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

        nav = self.navigator

        if self.racing:
            step = nav.optimal_steps[self.race_time_step]
            self.race_time_step += 1
            return step

        nav.update_map(sensors)

        if nav.goal_visited and nav.found_optimal_path():
            self.racing = True
            nav.print_maze_with_path_costs()
            print "Best found path: {}".format(nav.optimal_path)
            print "Best found path steps count: {}".format(len(nav.optimal_steps))
            return 'Reset', 'Reset'

        return nav.explore()
