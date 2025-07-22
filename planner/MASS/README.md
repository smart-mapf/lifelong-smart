## Code for AAAI submission

The repository is developed and tested in Ubuntu 20.04.

## Requirements
The code makes use of the external library [boost](https://www.boost.org/) and [Cplex](https://www.ibm.com/products/ilog-cplex-optimization-studio/cplex-optimizer).

If you are using Ubuntu, you can install the boost by
```shell script
sudo apt install libboost-all-dev
``` 

If the above method does not work, you can also follow the instructions
on the [boost](https://www.boost.org/) website and install it manually.

The required external library Cplex is from IBM. You can follow this guide [link](https://www.ibm.com/docs/en/icos/20.1.0?topic=cplex-installing) from IBM to install the package.




## Overall Usage
To compile the code:
```shell script
cd MASS
mkdir build
cd build
cmake ..
make
cd ..
```

To run the code
```shell script
./MASS -k 10 -p 1 -s 0 -m ./random-32-32-20.map -a ./random-32-32-20-random-1.scen -c ./output/random-32-32-20.csv
```

- m: the map file from the MAPF benchmark
- a: the scenario file from the MAPF benchmark
- c: the output file that contains the search statistics
- p: Determine if enable partial expansion: 0 for disable partial expansion, 1 for enable partial expansion
- s: Determine which solver to use: 0 for BAS, 1 for BCS
- k: the number of agents
