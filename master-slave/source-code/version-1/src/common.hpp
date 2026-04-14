#pragma once
#include <iostream>
#include <vector>
#include <utility>
#include <mpi.h>
#include <omp.h>

inline constexpr int MASTER_RANK = 0;

inline constexpr int TAG_ELITE_WORKER_SEND_2_MASTER = 1;
inline constexpr int TAG_ELITE_MASTER_SEND_2_WORKER = 2;

struct Trip {
    std::vector<std::pair<int, int>> customers; // <customer, next customer (-1: last customer)>
};

struct EliteElement {
    int type; // 0: truck, 1: drone
    int vehicle_number;
    std::vector<Trip> trips;
};

struct Elite {
    int worker_rank;
    std::vector<EliteElement> elements;
};

std::vector<int> pack_elite(const Elite &e);
Elite unpack_elite(const std::vector<int> &buf);
