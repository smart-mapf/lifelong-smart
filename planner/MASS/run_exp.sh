#/bin/bash

# Loop through agents from 10 to 100 with a step of 10
for agents in {20..81..10}; do
    # Loop through seeds from 1 to 10 with a step of 1
    for seed in {1..5}; do
        # Run the MASS planner with the specified number of agents and seed
        echo "Running MASS planner with $agents agents and seed $seed"
        ./MASS -k $agents -p 1 -s 0 -d $seed -m ./random-32-32-20.map -a ./random-32-32-20-random-1.scen -c ./output/random-32-32-20.csv &
    done
done

# Wait for all background processes to finish
wait
