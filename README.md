# Plot and Navigate a Virtual Maze
## Machine Learning Capstone Project
#### Machine Learning Nanodegree (Udacity)
Project submission by Edward Minnett (ed@methodic.io).

November 3rd, 2016. (Revision 2)

----------

## Overview

The primary purpose of this project is to program a virtual robot to explore a simple maze, find a route from a corner of that maze to its centre, and traverse the path from start to finish. Of course, there will be an optimal path and less optimal paths as well as efficient and inefficient ways of finding those paths. The virtual robot’s score for a given maze is a calculation based upon how many steps it uses first to explore and then race through the maze. The parameters and context of this project are inspired by the Micromouse robot competition [1] and is, in effect, a virtual version of the Micromouse problem. The robot mouse traverses the maze twice. The first traversal offers the opportunity to explore and map the maze while the second run requires the robot mouse attempt to reach the centre of the maze as quickly as it can using the knowledge acquired while exploring the maze. The goal of this project is to define and implement a strategy to consistently discover an optimal path through a series of test mazes that exist within a virtual world inspired by the Micromouse problem.

For more information, please see the 'Plot and Navigate a Virtual Maze.pdf' report.

## Setup

This project has the following dependencies:

- Python 2.7
- IPython Notebook
- Turtle (for drawing a graphical representation of a maze)

## Display a graphical representation of a maze

```
$ python showmaze.py test_maze_01.txt
```

## Run the robot through a maze

```
$ python tester.py test_maze_01.txt
```

## Randomly generate new mazes

This could overwrite existing mazes. Be sure to back up existing mazes if you wish to keep them.
To modify the generation parameters (size of mazes, number of mazes for each size), edit these values in maze_generator.py.

```
$ python maze_generator.py
```

## Batch run the robot through the randomly generated maze

This script will run the robot against all of the mazes in the generated_mazes folder and save the results to a CSV text file for each maze dimension. If there are a large number of generated mazes, this script can take several hours to run.

```
$ python batch_maze_runner.py
```
