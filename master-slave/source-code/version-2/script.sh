mpirun --allow-run-as-root -np 10 ./build/tabu_search run \
    ../../../data/200.40.4.txt \
    --adaptive-iterations 5 \
    --adaptive-pull-elite-segments 3 \
    --adaptive-pull-elite-limit 5 \
    --diversity-weight-edge 0.9 \
    --diversity-weight-assignment 0.1