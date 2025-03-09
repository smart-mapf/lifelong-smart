cd ./server/build
./ADG_server -p ../data/maze-32-32-4_paths.txt &
cd ../../client
argos3 -c ./experiments/maze-32-32-4-random-1.argos &
