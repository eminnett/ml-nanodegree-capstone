# Plot and Navigate a Virtual Maze: Python Files

The following 5 Python files were written to solve and complete this project

## robot.py

The Robot class encapsulates very little logic and information. It relies very
heavily upon its instance of the Navigator class to map and navigate the maze.

## navigator.py

The Navigator encapsulates the core Robot logic and is responsible for
the following:
    - Maintaining the current location and heading values on behalf of
        for the Robot given the movements and rotations issued while
        exploring.
    - Deciding the best rotation and movement to make while exploring
        the maze given the current state of the Navigator's Mapper instance.
    - Finding the optimal path through the maze.

## mapper.py

The Mapper is responsible for maintaining a map of what is known about
the maze. This is achieved by storing and updating two sets of two-dimensional
lists. One stores knowledge about the walls and openings in the maze
along with where in the maze the state of a wall or opening is unknown.
The other two-dimensional list stores information about how many possible
shapes each cell could have given the current knowledge of the maze walls.
This second two-dimensional list, in effect, represents the uncertainty
within the current maze map.

## maze_generator.py

The MazeGenerator class generates a random maze of dimension maze_dim and
ensures that all spaces are accessible, and as a result, ensuring
that the maze is solvable.

When run in the terminal, maze_generator.py is setup to generate 100 random
mazes for each even dimension from 12 and 20.

## batch_maze_runner.py

The batch_maze_runner.py script runs the robot against all of the mazes in the
generated_mazes folder and save the results to a CSV text file for each maze
dimension. If there are a large number of generated mazes, this script can
take several hours to run.
