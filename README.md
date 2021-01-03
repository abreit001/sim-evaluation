# sim-evaluation

CURRENT FILES:

dataFetcherCombined.sh:

Use dataFetcherCombined.sh (run using the command ./dataFetcherCombined.sh) to extract data from simulation trials to csv files. This will create a csv file with goal data and a csv file with pose data. Specify the name of these files within the script. This script will label the data based on the trial number so we can detect which trials are missing data. Files created using this script can be fed into sim_eval.py and sim_eval_out_and_back.py.

sim_eval.py:

Use this to evaluate one-goal treks. This script will parse in data from the csv files created by dataFetcherCombined.sh. This file contains functions for plotting the data and evaluating the failures as well.

sim_eval.py:

Use this to evaluate two-goal (out and back) treks. This script will parse in data from the csv files created by dataFetcherCombined.sh. This file contains functions for plotting the data and evaluating the failures as well.
