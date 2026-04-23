#include "master.hpp"
#include "common.hpp"
#include "config.hpp"
#include "solutions.hpp"

#include <iostream>
#include <map>
#include <random>
#include <thread>
#include <chrono>
#include <vector>
#include <mpi.h>

const int ELITE_POOL_SIZE = 5;

static int count_diff_elite(const common::Elite &a, const common::Elite &b) {
    auto extract = [](const common::Elite &e) {
        std::map<int, int> m;
        for (const auto &el : e.elements)
            for (const auto &trip : el.trips)
                for (const auto &[cus, nxt] : trip.customers)
                    m[cus] = nxt;
        return m;
    };

    auto map_a = extract(a);
    auto map_b = extract(b);

    int diff = 0;

    for (const auto &[cus, next_a] : map_a) {
        if (map_b[cus] != next_a)
            diff++;
    }

    return diff;
}

static void push_elite(const common::Elite &e, int &elite_pool_count, std::vector<common::Elite> &elite_pool) {
    if (elite_pool_count < ELITE_POOL_SIZE) {
        elite_pool[elite_pool_count++] = e;
    } else {
        int replace = 0;
        int max_diff = count_diff_elite(e, elite_pool[0]);
        for (int i = 1; i < ELITE_POOL_SIZE; ++i) {
            int diff = count_diff_elite(e, elite_pool[i]);
            if (diff > max_diff) {
                max_diff = diff;
                replace = i;
            }
        }
        elite_pool[replace] = e;
    }
}

static void pull_elite(common::Elite &e, const std::vector<common::Elite> &elite_pool, int elite_pool_count, int requester_rank) {
    if (elite_pool_count == 0) {
        return;
    }

    std::vector<int> candidate_indices;
    candidate_indices.reserve(elite_pool_count);
    for (int i = 0; i < elite_pool_count; ++i) {
        if (elite_pool[i].worker_rank != requester_rank) {
            candidate_indices.push_back(i);
        }
    }

    if (candidate_indices.empty()) {
        return;
    }

    static std::random_device rd;
    static std::mt19937_64 gen(rd());
    std::uniform_int_distribution<int> dist(0, static_cast<int>(candidate_indices.size()) - 1);
    int idx = candidate_indices[dist(gen)];
    e = elite_pool[idx];
}

void master(int size) {
    int elite_pool_count = 0;
    std::vector<common::Elite> elite_pool(ELITE_POOL_SIZE);
    std::vector<char> worker_done(size, 0);
    int done_workers = 0;

    MPI_Status status;
    int iterations = 0;
    
    while (done_workers < size - 1) {
        iterations++;
        int flag_push = 0, flag_pull = 0, flag_done = 0;

        MPI_Iprobe(MPI_ANY_SOURCE, common::TAG_PUSH_ELITE_WORKER_REQUEST, MPI_COMM_WORLD, &flag_push, &status);

        if (flag_push) {
            int n;
            MPI_Recv(&n, 1, MPI_INT, status.MPI_SOURCE, common::TAG_PUSH_ELITE_WORKER_REQUEST, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

            std::vector<int> buf(n);
            MPI_Recv(buf.data(), n, MPI_INT, status.MPI_SOURCE, common::TAG_PUSH_ELITE_WORKER_REQUEST, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

            common::Elite new_elite = common::unpack_elite(buf);
            push_elite(new_elite, elite_pool_count, elite_pool);
            
            std::cerr << "[Master] Received elite from worker " << status.MPI_SOURCE 
                      << ", pool size: " << elite_pool_count << "\n";
        }
        
        MPI_Iprobe(MPI_ANY_SOURCE, common::TAG_PULL_ELITE_WORKER_REQUEST, MPI_COMM_WORLD, &flag_pull, &status);
        
        if (flag_pull) {
            int worker_rank;
            MPI_Recv(&worker_rank, 1, MPI_INT, status.MPI_SOURCE, common::TAG_PULL_ELITE_WORKER_REQUEST, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
            
            common::Elite pulled_elite;
            pull_elite(pulled_elite, elite_pool, elite_pool_count, worker_rank);
            
            auto buf = common::pack_elite(pulled_elite);
            int n = (int) buf.size();
            MPI_Send(&n, 1, MPI_INT, worker_rank, common::TAG_ELITE_MASTER_SEND_PULLED, MPI_COMM_WORLD);
            MPI_Send(buf.data(), n, MPI_INT, worker_rank, common::TAG_ELITE_MASTER_SEND_PULLED, MPI_COMM_WORLD);
            
            std::cerr << "[Master] Sent pulled elite to worker " << worker_rank << "\n";
        }

        MPI_Iprobe(MPI_ANY_SOURCE, common::TAG_WORKER_DONE, MPI_COMM_WORLD, &flag_done, &status);

        if (flag_done) {
            int worker_rank;
            MPI_Recv(&worker_rank, 1, MPI_INT, status.MPI_SOURCE, common::TAG_WORKER_DONE, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

            if (worker_rank > common::MASTER_RANK && worker_rank < size && !worker_done[worker_rank]) {
                worker_done[worker_rank] = 1;
                done_workers++;
                std::cerr << "[Master] Worker " << worker_rank << " finished (" << done_workers
                          << "/" << (size - 1) << ")\n";
            }
        }
        
        if (!flag_push && !flag_pull && !flag_done) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // Drain remaining pushed elites that may arrive just before/with worker done signals.
    while (true) {
        int flag_push = 0;
        MPI_Iprobe(MPI_ANY_SOURCE, common::TAG_PUSH_ELITE_WORKER_REQUEST, MPI_COMM_WORLD, &flag_push, &status);
        if (!flag_push) {
            break;
        }

        int n;
        MPI_Recv(&n, 1, MPI_INT, status.MPI_SOURCE, common::TAG_PUSH_ELITE_WORKER_REQUEST, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

        std::vector<int> buf(n);
        MPI_Recv(buf.data(), n, MPI_INT, status.MPI_SOURCE, common::TAG_PUSH_ELITE_WORKER_REQUEST, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

        common::Elite new_elite = common::unpack_elite(buf);
        push_elite(new_elite, elite_pool_count, elite_pool);
    }

    if (elite_pool_count > 0) {
        const Config &cfg = global_config();
        int best_idx = 0;
        double best_cost = solutions::compute_elite_cost(cfg, elite_pool[0]);

        for (int i = 1; i < elite_pool_count; ++i) {
            double cost = solutions::compute_elite_cost(cfg, elite_pool[i]);
            if (cost < best_cost) {
                best_cost = cost;
                best_idx = i;
            }
        }

        std::cerr << "[Master] Best elite in pool has cost " << best_cost
                  << " from worker " << elite_pool[best_idx].worker_rank << "\n";
        common::print_elite(elite_pool[best_idx], std::cerr);
    } else {
        std::cerr << "[Master] Elite pool is empty at finish\n";
    }
    
    std::cerr << "[Master] Finished after " << iterations << " iterations with all workers done\n";
}
