#include "worker.hpp"
#include "common.hpp"
#include "config.hpp"
#include "solutions.hpp"

#include <algorithm>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <stdexcept>
#include <vector>
#include <mpi.h>

const int PULL_ELITE_NO_IMPROVE_INTERVAL = 50;
const int STOP_NO_IMPROVE_THRESHOLD = 1000;

static std::mt19937_64 make_rng(const Config &cfg) {
    if (cfg.seed) {
        return std::mt19937_64(*cfg.seed);
    }

    std::random_device rd;
    std::seed_seq seq{rd(), rd(), rd(), rd()};
    return std::mt19937_64(seq);
}

static double customer_demand(const Config &cfg, std::size_t customer) {
    return cfg.demands[customer];
}

static bool is_truck_customer_feasible(const Config &cfg, std::size_t customer) {
    return customer_demand(cfg, customer) <= cfg.truck.capacity;
}

static bool is_drone_customer_feasible(const Config &cfg, std::size_t customer) {
    if (!cfg.dronable[customer]) {
        return false;
    }

    const auto &drone = cfg.drone;
    const double dem = customer_demand(cfg, customer);
    if (dem > drone.capacity()) {
        return false;
    }

    const double takeoff = drone.takeoff_time();
    const double landing = drone.landing_time();
    const double cruise_dist = cfg.drone_distances[0][customer] + cfg.drone_distances[customer][0];
    if (takeoff + drone.cruise_time(cruise_dist) + landing > drone.fixed_time()) {
        return false;
    }

    const double takeoff_from_depot = drone.takeoff_power(0.0);
    const double landing_from_depot = drone.landing_power(0.0);
    const double cruise_from_depot = drone.cruise_power(0.0);

    const double energy =
        (landing_from_depot + drone.landing_power(dem)) * landing
        + drone.cruise_power(dem) * drone.cruise_time(cfg.drone_distances[customer][0])
        + (takeoff_from_depot + drone.takeoff_power(dem)) * takeoff
        + cruise_from_depot * drone.cruise_time(cfg.drone_distances[0][customer]);

    return energy <= drone.battery();
}

static bool is_drone_route_feasible(const Config &cfg, const std::vector<int> &route) {
    if (route.empty()) {
        return true;
    }

    const auto &drone = cfg.drone;
    const double total_demand = std::accumulate(
        route.begin(), route.end(), 0.0,
        [&](double acc, int customer) {
            return acc + customer_demand(cfg, static_cast<std::size_t>(customer));
        });

    if (total_demand > drone.capacity()) {
        return false;
    }

    double cruise_distance = cfg.drone_distances[0][route.front()];
    for (std::size_t i = 0; i + 1 < route.size(); ++i) {
        cruise_distance += cfg.drone_distances[route[i]][route[i + 1]];
    }
    cruise_distance += cfg.drone_distances[route.back()][0];

    const double total_time = drone.takeoff_time() + drone.cruise_time(cruise_distance) + drone.landing_time();
    if (total_time > drone.fixed_time()) {
        return false;
    }

    const double takeoff_from_depot = drone.takeoff_power(0.0);
    const double landing_from_depot = drone.landing_power(0.0);

    double energy = (takeoff_from_depot + drone.takeoff_power(total_demand)) * drone.takeoff_time();
    double payload = total_demand;
    int prev = 0;

    for (int customer : route) {
        const double distance = cfg.drone_distances[prev][customer];
        energy += drone.cruise_power(payload) * drone.cruise_time(distance);
        payload -= customer_demand(cfg, static_cast<std::size_t>(customer));
        prev = customer;
    }

    energy += drone.cruise_power(payload) * drone.cruise_time(cfg.drone_distances[prev][0]);
    energy += (landing_from_depot + drone.landing_power(payload)) * drone.landing_time();

    return energy <= drone.battery();
}

static common::Trip route_to_trip(const std::vector<int> &route) {
    common::Trip trip;
    for (std::size_t i = 0; i < route.size(); ++i) {
        int next = (i + 1 < route.size()) ? route[i + 1] : -1;
        trip.customers[route[i]] = next;
    }
    return trip;
}

// Random each customer to a random feasible vehicle, then devide it into trips in the next functions
static std::vector<std::vector<int>> build_vehicle_customers(const Config &cfg, std::mt19937_64 &rng) {
    std::vector<std::vector<int>> vehicle_customers(cfg.trucks_count + cfg.drones_count);

    for (std::size_t customer = 1; customer <= cfg.customers_count; ++customer) {
        std::vector<std::size_t> candidates;
        candidates.reserve(cfg.trucks_count + cfg.drones_count);

        for (std::size_t truck = 0; truck < cfg.trucks_count; ++truck) {
            if (is_truck_customer_feasible(cfg, customer)) {
                candidates.push_back(truck);
            }
        }

        if (is_drone_customer_feasible(cfg, customer)) {
            for (std::size_t drone = 0; drone < cfg.drones_count; ++drone) {
                candidates.push_back(cfg.trucks_count + drone);
            }
        }

        if (candidates.empty()) {
            throw std::runtime_error("No feasible vehicle available for customer " + std::to_string(customer));
        }

        std::uniform_int_distribution<std::size_t> pick(0, candidates.size() - 1);
        vehicle_customers[candidates[pick(rng)]].push_back(static_cast<int>(customer));
    }

    return vehicle_customers;
}

// Devide customers assigned to a truck into trips
static common::EliteElement build_truck_element(const Config &cfg, std::size_t vehicle_number, std::vector<int> &customers, std::mt19937_64 &rng) {
    std::shuffle(customers.begin(), customers.end(), rng);

    common::EliteElement element;
    element.type = 0;
    element.vehicle_number = static_cast<int>(vehicle_number);
    if (customers.empty()) {
        return element;
    }

    std::vector<double> trip_loads;

    for (int customer : customers) {
        const double demand = customer_demand(cfg, static_cast<std::size_t>(customer));
        bool placed = false;

        for (std::size_t ti = 0; ti < element.trips.size(); ++ti) {
            if (trip_loads[ti] + demand <= cfg.truck.capacity) {
                auto &trip_customers = element.trips[ti].customers;
                if (trip_customers.empty()) {
                    trip_customers[customer] = -1;
                } else {
                    int tail = trip_customers.begin()->first;
                    while (trip_customers[tail] != -1) {
                        tail = trip_customers[tail];
                    }
                    trip_customers[tail] = customer;
                    trip_customers[customer] = -1;
                }
                trip_loads[ti] += demand;
                placed = true;
                break;
            }
        }

        if (!placed) {
            common::Trip trip;
            trip.customers[customer] = -1;
            element.trips.push_back(std::move(trip));
            trip_loads.push_back(demand);
        }
    }

    return element;
}

// Devide customers assigned to a drone into trips
static common::EliteElement build_drone_element(const Config &cfg, std::size_t vehicle_number, std::vector<int> &customers, std::mt19937_64 &rng) {
    std::shuffle(customers.begin(), customers.end(), rng);

    common::EliteElement element;
    element.type = 1;
    element.vehicle_number = static_cast<int>(vehicle_number);

    std::vector<std::vector<int>> routes;

    for (int customer : customers) {
        bool placed = false;

        std::vector<std::size_t> trip_order(routes.size());
        for (std::size_t i = 0; i < trip_order.size(); ++i) {
            trip_order[i] = i;
        }
        std::shuffle(trip_order.begin(), trip_order.end(), rng);

        for (std::size_t ti : trip_order) {
            auto candidate = routes[ti];

            std::vector<std::size_t> positions(candidate.size() + 1);
            for (std::size_t p = 0; p < positions.size(); ++p) {
                positions[p] = p;
            }
            std::shuffle(positions.begin(), positions.end(), rng);

            bool inserted = false;
            for (std::size_t pos : positions) {
                auto trial = candidate;
                trial.insert(trial.begin() + static_cast<std::vector<int>::difference_type>(pos), customer);
                if (is_drone_route_feasible(cfg, trial)) {
                    routes[ti] = std::move(trial);
                    placed = true;
                    inserted = true;
                    break;
                }
            }

            if (inserted) {
                break;
            }
        }

        if (!placed) {
            std::vector<int> route{customer};
            if (!is_drone_route_feasible(cfg, route)) {
                throw std::runtime_error("Infeasible drone customer in random elite: " + std::to_string(customer));
            }
            routes.push_back(std::move(route));
        }
    }

    element.trips.reserve(routes.size());
    for (const auto &route : routes) {
        element.trips.push_back(route_to_trip(route));
    }

    return element;
}

static common::EliteElement build_element(const Config &cfg, int type, std::size_t vehicle_number, std::vector<int> &customers, std::mt19937_64 &rng) {
    if (type == 0) {
        return build_truck_element(cfg, vehicle_number, customers, rng);
    }
    if (type == 1) {
        return build_drone_element(cfg, vehicle_number, customers, rng);
    }

    throw std::runtime_error("Unsupported vehicle type in build_element");
}

static common::Elite build_random_elite(const Config &cfg, std::mt19937_64 &rng) {
    auto vehicle_customers = build_vehicle_customers(cfg, rng);

    common::Elite elite;
    elite.elements.reserve(cfg.trucks_count + cfg.drones_count);

    for (std::size_t truck = 0; truck < cfg.trucks_count; ++truck) {
        elite.elements.push_back(build_element(cfg, 0, truck, vehicle_customers[truck], rng));
    }

    for (std::size_t drone = 0; drone < cfg.drones_count; ++drone) {
        elite.elements.push_back(build_element(cfg, 1, drone, vehicle_customers[cfg.trucks_count + drone], rng));
    }

    return elite;
}

static common::Elite random_elite() {
    const Config &cfg = global_config();
    std::mt19937_64 rng = make_rng(cfg);
    return build_random_elite(cfg, rng);
}

void worker(int rank) {
    const Config &cfg = global_config();
    std::mt19937_64 rng = make_rng(cfg);
    
    common::Elite elite = random_elite();
    elite.worker_rank = rank;
    
    double best_cost = solutions::compute_elite_cost(cfg, elite);
    std::cerr << "[Worker " << rank << "] Initial elite cost: " << best_cost << std::endl;
    
    int no_improve_count = 0;
    int iter = 0;
    
    std::vector<int> neighborhood_order = solutions::generate_neighborhood_order(rng);
    std::cerr << "[Worker " << rank << "] Neighborhood order: ";
    for (int nh : neighborhood_order) {
        std::cerr << nh << " ";
    }
    std::cerr << std::endl;
    
    if (neighborhood_order.empty()) {
        throw std::runtime_error("Neighborhood order is empty");
    }

    while (no_improve_count < STOP_NO_IMPROVE_THRESHOLD) {
        const int nh_idx = neighborhood_order[iter % static_cast<int>(neighborhood_order.size())];
        common::Elite candidate = elite;
        iter++;

        switch (static_cast<solutions::Neighborhood>(nh_idx)) {
            case solutions::Neighborhood::Move10:
                candidate = solutions::apply_move10(cfg, candidate, rng);
                break;
            case solutions::Neighborhood::Move11:
                candidate = solutions::apply_move11(cfg, candidate, rng);
                break;
            case solutions::Neighborhood::Move20:
                candidate = solutions::apply_move20(cfg, candidate, rng);
                break;
            case solutions::Neighborhood::Move21:
                candidate = solutions::apply_move21(cfg, candidate, rng);
                break;
            case solutions::Neighborhood::Move22:
                candidate = solutions::apply_move22(cfg, candidate, rng);
                break;
            case solutions::Neighborhood::TwoOpt:
                candidate = solutions::apply_twoopt(cfg, candidate, rng);
                break;
            case solutions::Neighborhood::EjectionChain:
                candidate = solutions::apply_ejection_chain(cfg, candidate, rng);
                break;
            default:
                break;
        }

        double candidate_cost = solutions::compute_elite_cost(cfg, candidate);

        if (candidate_cost < best_cost) {
            elite = candidate;
            best_cost = candidate_cost;
            no_improve_count = 0;

            std::cerr << "[Worker " << rank << "] Improvement at iter " << iter
                      << ", nh " << nh_idx << ", cost " << best_cost << std::endl;

            elite.worker_rank = rank;
            auto buf = common::pack_elite(elite);
            int n = (int) buf.size();
            MPI_Send(&n, 1, MPI_INT, common::MASTER_RANK, common::TAG_PUSH_ELITE_WORKER_REQUEST, MPI_COMM_WORLD);
            MPI_Send(buf.data(), n, MPI_INT, common::MASTER_RANK, common::TAG_PUSH_ELITE_WORKER_REQUEST, MPI_COMM_WORLD);
            continue;
        }

        no_improve_count++;

        if (no_improve_count % PULL_ELITE_NO_IMPROVE_INTERVAL == 0) {
            std::cerr << "[Worker " << rank << "] No improvement for " << no_improve_count
                      << " evaluations, pulling elite from master" << std::endl;

            int req = rank;
            MPI_Send(&req, 1, MPI_INT, common::MASTER_RANK, common::TAG_PULL_ELITE_WORKER_REQUEST, MPI_COMM_WORLD);

            int n;
            MPI_Recv(&n, 1, MPI_INT, common::MASTER_RANK, common::TAG_ELITE_MASTER_SEND_PULLED, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

            std::vector<int> buf(n);
            MPI_Recv(buf.data(), n, MPI_INT, common::MASTER_RANK, common::TAG_ELITE_MASTER_SEND_PULLED, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

            elite = common::unpack_elite(buf);
            elite.worker_rank = rank;

            std::cerr << "[Worker " << rank << "] Pulled elite from master; keep local best_cost = "
                      << best_cost << std::endl;
        }
    }

    int done_rank = rank;
    MPI_Send(&done_rank, 1, MPI_INT, common::MASTER_RANK, common::TAG_WORKER_DONE, MPI_COMM_WORLD);

    std::cerr << "[Worker " << rank << "] Finished after " << iter
              << " neighborhood evaluations with best_cost " << best_cost << std::endl;
}
