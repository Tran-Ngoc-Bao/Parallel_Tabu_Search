#include "tabu_search.hpp"
#include "config.hpp"
#include <algorithm>
#include <random>
#include <set>
#include <cmath>

namespace tabu_search {

std::vector<int> generate_neighborhood_order(std::mt19937_64 &rng) {
    std::vector<int> order = {1, 2, 3, 4, 5, 6};
    std::shuffle(order.begin(), order.end(), rng);
    return order;
}

double compute_elite_cost(const Config &cfg, const common::Elite &elite) {
    double total_cost = 0.0;

    for (const auto &element : elite.elements) {
        for (const auto &trip : element.trips) {
            if (trip.customers.empty()) continue;

            // Find start node (not appearing as "next")
            std::set<int> next_nodes;
            for (const auto &[cus, nxt] : trip.customers) {
                if (nxt != -1) {
                    next_nodes.insert(nxt);
                }
            }

            int current = 0;
            for (const auto &[cus, nxt] : trip.customers) {
                if (!next_nodes.count(cus)) {
                    current = cus;
                    break;
                }
            }

            // Traverse route
            while (current != -1) {
                int next = -1;
                auto it = trip.customers.find(current);
                if (it != trip.customers.end()) {
                    next = it->second;
                }

                if (element.type == 0) {
                    total_cost += cfg.truck_distances[current == 0 ? 0 : current][next == -1 ? 0 : next];
                } else {
                    total_cost += cfg.drone_distances[current == 0 ? 0 : current][next == -1 ? 0 : next];
                }

                current = next;
                if (current == -1) break;
            }
        }
    }

    return total_cost;
}

// Helper: Get all customers in route order
static std::vector<int> get_route_sequence(const common::Trip &trip) {
    std::vector<int> route_seq;
    
    if (trip.customers.empty()) return route_seq;
    
    std::set<int> next_set;
    for (const auto &[cus, nxt] : trip.customers) {
        if (nxt != -1) next_set.insert(nxt);
    }
    
    int start = 0;
    for (const auto &[cus, _] : trip.customers) {
        if (!next_set.count(cus)) {
            start = cus;
            break;
        }
    }
    
    int current = start;
    while (current != -1) {
        route_seq.push_back(current);
        auto it = trip.customers.find(current);
        current = (it != trip.customers.end()) ? it->second : -1;
    }
    
    return route_seq;
}

// Helper: Calculate cost of a trip after removing and reinserting a customer
static double calculate_relocate_cost(const Config &cfg, int vehicle_type, 
                                       const std::vector<int> &route_seq, int move_pos, int new_pos) {
    if (route_seq.size() < 2) return 0.0;
    
    std::vector<int> new_route = route_seq;
    if (move_pos >= 0 && move_pos < (int)new_route.size()) {
        int customer = new_route[move_pos];
        new_route.erase(new_route.begin() + move_pos);
        
        if (new_pos > move_pos) new_pos--;
        if (new_pos >= 0 && new_pos <= (int)new_route.size()) {
            new_route.insert(new_route.begin() + new_pos, customer);
            
            // Calculate cost difference
            double old_cost = 0.0, new_cost = 0.0;
            
            if (vehicle_type == 0) {
                // Truck distances
                int prev = 0;
                for (int cus : route_seq) {
                    old_cost += cfg.truck_distances[prev][cus];
                    prev = cus;
                }
                old_cost += cfg.truck_distances[prev][0];
                
                prev = 0;
                for (int cus : new_route) {
                    new_cost += cfg.truck_distances[prev][cus];
                    prev = cus;
                }
                new_cost += cfg.truck_distances[prev][0];
            } else {
                // Drone distances
                int prev = 0;
                for (int cus : route_seq) {
                    new_cost += cfg.drone_distances[prev][cus];
                    prev = cus;
                }
                new_cost += cfg.drone_distances[prev][0];
                
                prev = 0;
                for (int cus : new_route) {
                    new_cost += cfg.drone_distances[prev][cus];
                    prev = cus;
                }
                new_cost += cfg.drone_distances[prev][0];
            }
            
            return new_cost - old_cost;
        }
    }
    
    return 0.0;
}

// Neighborhood 1: Relocate - move customer to different position (PARALLEL)
common::Elite apply_relocate(
    const Config &cfg,
    const common::Elite &elite,
    std::mt19937_64 &rng)
{
    auto result = elite;
    
    if (result.elements.empty()) return result;
    
    // Pick random element
    std::uniform_int_distribution<std::size_t> pick_el(0, result.elements.size() - 1);
    std::size_t el_idx = pick_el(rng);
    auto &element = result.elements[el_idx];
    
    if (element.trips.empty() || element.trips[0].customers.empty()) {
        return result;
    }

    auto route_seq = get_route_sequence(element.trips[0]);
    if (route_seq.size() < 2) return result;
    
    // Find best relocate move (PARALLEL)
    int best_move_pos = 0;
    int best_new_pos = 0;
    double best_cost_delta = 0.0;
    
    #pragma omp parallel for collapse(2) reduction(min:best_cost_delta) shared(best_move_pos, best_new_pos)
    for (int move_pos = 0; move_pos < (int)route_seq.size(); ++move_pos) {
        for (int new_pos = 0; new_pos <= (int)route_seq.size(); ++new_pos) {
            if (move_pos == new_pos || (move_pos + 1 == new_pos)) continue;
            
            double delta = calculate_relocate_cost(cfg, element.type, route_seq, move_pos, new_pos);
            
            #pragma omp critical
            {
                if (delta < best_cost_delta) {
                    best_cost_delta = delta;
                    best_move_pos = move_pos;
                    best_new_pos = new_pos;
                }
            }
        }
    }
    
    // Apply best move if improvement found
    if (best_cost_delta < 0.0) {
        int customer = route_seq[best_move_pos];
        route_seq.erase(route_seq.begin() + best_move_pos);
        
        int final_pos = best_new_pos;
        if (final_pos > best_move_pos) final_pos--;
        route_seq.insert(route_seq.begin() + final_pos, customer);
        
        // Rebuild trip structure
        element.trips[0].customers.clear();
        for (std::size_t i = 0; i < route_seq.size(); ++i) {
            int next = (i + 1 < route_seq.size()) ? route_seq[i + 1] : -1;
            element.trips[0].customers[route_seq[i]] = next;
        }
    }
    
    return result;
}

// Neighborhood 2: Swap - swap two customers (PARALLEL)
common::Elite apply_swap(
    const Config &cfg,
    const common::Elite &elite,
    std::mt19937_64 &rng)
{
    auto result = elite;
    
    if (result.elements.empty()) return result;
    
    std::uniform_int_distribution<std::size_t> pick_el(0, result.elements.size() - 1);
    std::size_t el_idx = pick_el(rng);
    auto &element = result.elements[el_idx];
    
    if (element.trips.empty() || element.trips[0].customers.size() < 2) {
        return result;
    }

    auto route_seq = get_route_sequence(element.trips[0]);
    if (route_seq.size() < 2) return result;
    
    // Find best swap move (PARALLEL)
    int best_pos1 = 0;
    int best_pos2 = 1;
    double best_cost_delta = 0.0;
    
    #pragma omp parallel for collapse(2) reduction(min:best_cost_delta) shared(best_pos1, best_pos2)
    for (int pos1 = 0; pos1 < (int)route_seq.size(); ++pos1) {
        for (int pos2 = pos1 + 1; pos2 < (int)route_seq.size(); ++pos2) {
            // Calculate cost delta for swapping pos1 and pos2
            std::vector<int> new_route = route_seq;
            std::swap(new_route[pos1], new_route[pos2]);
            
            double old_cost = 0.0, new_cost = 0.0;
            
            if (element.type == 0) {
                // Truck distances
                int prev = 0;
                for (int cus : route_seq) {
                    old_cost += cfg.truck_distances[prev][cus];
                    prev = cus;
                }
                old_cost += cfg.truck_distances[prev][0];
                
                prev = 0;
                for (int cus : new_route) {
                    new_cost += cfg.truck_distances[prev][cus];
                    prev = cus;
                }
                new_cost += cfg.truck_distances[prev][0];
            } else {
                // Drone distances
                int prev = 0;
                for (int cus : route_seq) {
                    old_cost += cfg.drone_distances[prev][cus];
                    prev = cus;
                }
                old_cost += cfg.drone_distances[prev][0];
                
                prev = 0;
                for (int cus : new_route) {
                    new_cost += cfg.drone_distances[prev][cus];
                    prev = cus;
                }
                new_cost += cfg.drone_distances[prev][0];
            }
            
            double delta = new_cost - old_cost;
            
            #pragma omp critical
            {
                if (delta < best_cost_delta) {
                    best_cost_delta = delta;
                    best_pos1 = pos1;
                    best_pos2 = pos2;
                }
            }
        }
    }
    
    // Apply best swap if improvement found
    if (best_cost_delta < 0.0) {
        std::swap(route_seq[best_pos1], route_seq[best_pos2]);
        
        element.trips[0].customers.clear();
        for (std::size_t i = 0; i < route_seq.size(); ++i) {
            int next = (i + 1 < route_seq.size()) ? route_seq[i + 1] : -1;
            element.trips[0].customers[route_seq[i]] = next;
        }
    }
    
    return result;
}

// Neighborhood 3: TwoOpt - reverse segment (PARALLEL)
common::Elite apply_twoopt(
    const Config &cfg,
    const common::Elite &elite,
    std::mt19937_64 &rng)
{
    auto result = elite;
    
    if (result.elements.empty()) return result;
    
    std::uniform_int_distribution<std::size_t> pick_el(0, result.elements.size() - 1);
    std::size_t el_idx = pick_el(rng);
    auto &element = result.elements[el_idx];
    
    if (element.trips.empty() || element.trips[0].customers.size() < 3) {
        return result;
    }

    auto route_seq = get_route_sequence(element.trips[0]);
    if (route_seq.size() < 3) return result;
    
    // Find best 2-opt move (PARALLEL)
    int best_i = 0;
    int best_j = 1;
    double best_cost_delta = 0.0;
    
    #pragma omp parallel for collapse(2) reduction(min:best_cost_delta) shared(best_i, best_j)
    for (int i = 0; i < (int)route_seq.size() - 1; ++i) {
        for (int j = i + 1; j < (int)route_seq.size(); ++j) {
            // Calculate cost delta for reversing segment [i, j]
            std::vector<int> new_route = route_seq;
            std::reverse(new_route.begin() + i, new_route.begin() + j + 1);
            
            double old_cost = 0.0, new_cost = 0.0;
            
            if (element.type == 0) {
                // Truck distances
                int prev = 0;
                for (int cus : route_seq) {
                    old_cost += cfg.truck_distances[prev][cus];
                    prev = cus;
                }
                old_cost += cfg.truck_distances[prev][0];
                
                prev = 0;
                for (int cus : new_route) {
                    new_cost += cfg.truck_distances[prev][cus];
                    prev = cus;
                }
                new_cost += cfg.truck_distances[prev][0];
            } else {
                // Drone distances
                int prev = 0;
                for (int cus : route_seq) {
                    old_cost += cfg.drone_distances[prev][cus];
                    prev = cus;
                }
                old_cost += cfg.drone_distances[prev][0];
                
                prev = 0;
                for (int cus : new_route) {
                    new_cost += cfg.drone_distances[prev][cus];
                    prev = cus;
                }
                new_cost += cfg.drone_distances[prev][0];
            }
            
            double delta = new_cost - old_cost;
            
            #pragma omp critical
            {
                if (delta < best_cost_delta) {
                    best_cost_delta = delta;
                    best_i = i;
                    best_j = j;
                }
            }
        }
    }
    
    // Apply best 2-opt if improvement found
    if (best_cost_delta < 0.0) {
        std::reverse(route_seq.begin() + best_i, route_seq.begin() + best_j + 1);
        
        element.trips[0].customers.clear();
        for (std::size_t k = 0; k < route_seq.size(); ++k) {
            int next = (k + 1 < route_seq.size()) ? route_seq[k + 1] : -1;
            element.trips[0].customers[route_seq[k]] = next;
        }
    }
    
    return result;
}

// Neighborhood 4: Move2 - move 2 consecutive customers (PARALLEL)
common::Elite apply_move2(
    const Config &cfg,
    const common::Elite &elite,
    std::mt19937_64 &rng)
{
    auto result = elite;
    if (result.elements.empty()) return result;
    
    std::uniform_int_distribution<std::size_t> pick_el(0, result.elements.size() - 1);
    std::size_t el_idx = pick_el(rng);
    auto &element = result.elements[el_idx];
    
    if (element.trips.empty() || element.trips[0].customers.size() < 3) {
        return result;
    }

    auto route_seq = get_route_sequence(element.trips[0]);
    if (route_seq.size() < 3) return result;
    
    // Find best move of 2 consecutive customers (PARALLEL)
    int best_move_i = 0;
    int best_new_pos = 0;
    double best_cost_delta = 0.0;
    
    #pragma omp parallel for collapse(2) reduction(min:best_cost_delta) shared(best_move_i, best_new_pos)
    for (int i = 0; i < (int)route_seq.size() - 1; ++i) {
        for (int j = 0; j < (int)route_seq.size(); ++j) {
            if (i == j || (i + 1 == j)) continue;
            
            // Create new route by moving customers at [i, i+1] to position j
            std::vector<int> new_route = route_seq;
            int cus1 = new_route[i];
            int cus2 = new_route[i + 1];
            
            new_route.erase(new_route.begin() + i, new_route.begin() + i + 2);
            int final_j = (j > i) ? j - 2 : j;
            if (final_j >= 0 && final_j <= (int)new_route.size()) {
                new_route.insert(new_route.begin() + final_j, {cus1, cus2});
            } else {
                continue;
            }
            
            // Calculate cost delta
            double old_cost = 0.0, new_cost = 0.0;
            
            if (element.type == 0) {
                // Truck distances
                int prev = 0;
                for (int cus : route_seq) {
                    old_cost += cfg.truck_distances[prev][cus];
                    prev = cus;
                }
                old_cost += cfg.truck_distances[prev][0];
                
                prev = 0;
                for (int cus : new_route) {
                    new_cost += cfg.truck_distances[prev][cus];
                    prev = cus;
                }
                new_cost += cfg.truck_distances[prev][0];
            } else {
                // Drone distances
                int prev = 0;
                for (int cus : route_seq) {
                    old_cost += cfg.drone_distances[prev][cus];
                    prev = cus;
                }
                old_cost += cfg.drone_distances[prev][0];
                
                prev = 0;
                for (int cus : new_route) {
                    new_cost += cfg.drone_distances[prev][cus];
                    prev = cus;
                }
                new_cost += cfg.drone_distances[prev][0];
            }
            
            double delta = new_cost - old_cost;
            
            #pragma omp critical
            {
                if (delta < best_cost_delta) {
                    best_cost_delta = delta;
                    best_move_i = i;
                    best_new_pos = j;
                }
            }
        }
    }
    
    // Apply best move if improvement found
    if (best_cost_delta < 0.0) {
        int cus1 = route_seq[best_move_i];
        int cus2 = route_seq[best_move_i + 1];
        
        route_seq.erase(route_seq.begin() + best_move_i, route_seq.begin() + best_move_i + 2);
        int final_pos = (best_new_pos > best_move_i) ? best_new_pos - 2 : best_new_pos;
        route_seq.insert(route_seq.begin() + final_pos, {cus1, cus2});
        
        element.trips[0].customers.clear();
        for (std::size_t k = 0; k < route_seq.size(); ++k) {
            int next = (k + 1 < route_seq.size()) ? route_seq[k + 1] : -1;
            element.trips[0].customers[route_seq[k]] = next;
        }
    }
    
    return result;
}

// Neighborhood 5: ChangeVehicle (simplified - do nothing for now)
static common::Elite apply_change_vehicle(
    const Config &cfg,
    const common::Elite &elite,
    std::mt19937_64 &rng)
{
    // This requires complex feasibility checking
    // For now, just return unchanged
    return elite;
}

// Neighborhood 6: Exchange (simplified - do nothing for now)
static common::Elite apply_exchange(
    const Config &cfg,
    const common::Elite &elite,
    std::mt19937_64 &rng)
{
    // This requires complex feasibility checking
    // For now, just return unchanged
    return elite;
}

common::Elite tabu_search_run(
    const Config &cfg,
    common::Elite initial_solution,
    std::mt19937_64 &rng,
    int max_iterations)
{
    auto best_elite = initial_solution;
    double best_cost = compute_elite_cost(cfg, best_elite);
    
    // Generate random neighborhood order (fixed for this worker)
    std::vector<int> neighborhood_order = generate_neighborhood_order(rng);
    
    // Main loop
    for (int iter = 0; iter < max_iterations; ++iter) {
        bool improved = false;
        
        for (int nh_idx : neighborhood_order) {
            common::Elite candidate = best_elite;
            
            switch (nh_idx) {
                case 1:
                    candidate = apply_relocate(cfg, candidate, rng);
                    break;
                case 2:
                    candidate = apply_swap(cfg, candidate, rng);
                    break;
                case 3:
                    candidate = apply_twoopt(cfg, candidate, rng);
                    break;
                case 4:
                    candidate = apply_move2(cfg, candidate, rng);
                    break;
                case 5:
                    candidate = apply_change_vehicle(cfg, candidate, rng);
                    break;
                case 6:
                    candidate = apply_exchange(cfg, candidate, rng);
                    break;
                default:
                    break;
            }
            
            double candidate_cost = compute_elite_cost(cfg, candidate);
            
            if (candidate_cost < best_cost) {
                best_elite = candidate;
                best_cost = candidate_cost;
                improved = true;
                break;
            }
        }
        
        if (!improved) {
            // No improvement in this iteration, could apply diversification here
        }
    }
    
    return best_elite;
}

} // namespace tabu_search
