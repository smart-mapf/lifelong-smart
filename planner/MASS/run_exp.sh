#/bin/bash

map="$1"
output="$2"

# Loop through agents from 10 to 100 with a step of 10
for agents in {5..56..10}; do
    # Loop through seeds from 1 to 10 with a step of 1
    for seed in {1..5}; do
        # Run the MASS planner with the specified number of agents and seed
        echo "Running MASS planner with $agents agents and seed $seed"
        ./MASS -k $agents -p 1 -s 0 -d $seed -m $map -c $output &
    done
done

# Wait for all background processes to finish
wait
