from glob import glob
import re
import subprocess

if __name__ == '__main__':
    '''
    Batch run all of the mazes in the generated_mazes_directory and save the
    results to text files in a CSV format for each maze dimension found.
    '''
    generated_mazes_directory = './generated_mazes'

    dimension_paths = glob(generated_mazes_directory + '/*/')

    for path in dimension_paths:
        maze_dim = path.split('/')[2]
        maze_files = glob(path + '/*.txt')
        results = 'file_name, maze_dim, upper_benchmark, lower_benchmark, score\n'
        for maze_file in maze_files:
            print "Running maze test for: {}".format(maze_file)

            benchmark_pattern = re.compile("[u|l]b-(\d+\.\d*)")
            matches = benchmark_pattern.findall(maze_file)
            upper_benchmark = matches[0]
            lower_benchmark = matches[1]

            proc = subprocess.Popen(['python', 'tester.py',  maze_file],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            tester_output = proc.communicate()[0]

            score_pattern = re.compile("Task complete! Score: (\d+\.\d*)")
            match = score_pattern.search(tester_output)
            if match != None:
                score = score_pattern.search(tester_output).group(1)
            else:
                score = 1000 + 1000 / 30.

            result = '{}, {}, {}, {}, {}\n'.format(maze_file, maze_dim,
                                        upper_benchmark, lower_benchmark, score)
            results += result

        results_file_name = '/{}x{}_batch_maze_runner_results.txt'.format(maze_dim, maze_dim)
        results_path = generated_mazes_directory + results_file_name
        f = open(results_path, "w")
        f.write(results)
        f.close()
