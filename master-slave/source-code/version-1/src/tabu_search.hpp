#pragma once

#include "common.hpp"
#include "config.hpp"
#include <vector>
#include <random>
#include <omp.h>

namespace tabu_search {

// Six neighborhoods
enum class Neighborhood : int {
    Relocate   = 1,   // Move customer to different position/trip/vehicle
    Swap       = 2,   // Swap two customers
    TwoOpt     = 3,   // 2-opt move (reverse segment in route)
    Move2      = 4,   // Move 2 consecutive customers
    ChangeVehicle = 5, // Reassign customer to different vehicle type
    Exchange   = 6    // Exchange routes/customers between vehicles
};

// Generate random neighborhood order for tabu search
std::vector<int> generate_neighborhood_order(std::mt19937_64 &rng);

// Neighborhood move operators
common::Elite apply_relocate(const Config &cfg, const common::Elite &elite, std::mt19937_64 &rng);
common::Elite apply_swap(const Config &cfg, const common::Elite &elite, std::mt19937_64 &rng);
common::Elite apply_twoopt(const Config &cfg, const common::Elite &elite, std::mt19937_64 &rng);
common::Elite apply_move2(const Config &cfg, const common::Elite &elite, std::mt19937_64 &rng);

// Run tabu search on initial solution
common::Elite tabu_search_run(
    const Config &cfg,
    common::Elite initial_solution,
    std::mt19937_64 &rng,
    int max_iterations = 100);

// Compute objective (total cost/distance) of elite solution
double compute_elite_cost(const Config &cfg, const common::Elite &elite);

} // namespace tabu_search
