#include "master.hpp"
#include <random>
#include <thread>
#include <chrono>

const int ELITE_POOL_SIZE = 6;

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
    if(elite_pool_count < ELITE_POOL_SIZE) {
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

static void pull_elite(common::Elite &e, const std::vector<common::Elite> &elite_pool, int elite_pool_count) {
    if (elite_pool_count == 0) {
        return;
    }
    
    // Pick random elite from pool
    static std::random_device rd;
    static std::mt19937_64 gen(rd());
    std::uniform_int_distribution<int> dist(0, elite_pool_count - 1);
    int idx = dist(gen);
    e = elite_pool[idx];
}

void master(int size) {
    int elite_pool_count = 0;
    std::vector<common::Elite> elite_pool(ELITE_POOL_SIZE);

    MPI_Status status;
    bool running = true;
    int iterations = 0;
    
    // Run master loop for a fixed number of iterations or until no messages
    while(running && iterations < 1000) {
        iterations++;
        int flag_push = 0, flag_pull = 0;

        // Check for elite push from workers
        MPI_Iprobe(MPI_ANY_SOURCE, common::TAG_ELITE_WORKER_SEND_2_MASTER, MPI_COMM_WORLD, &flag_push, &status);

        if(flag_push) {
            int n;
            MPI_Recv(&n, 1, MPI_INT, status.MPI_SOURCE, common::TAG_ELITE_WORKER_SEND_2_MASTER, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

            std::vector<int> buf(n);
            MPI_Recv(buf.data(), n, MPI_INT, status.MPI_SOURCE, common::TAG_ELITE_WORKER_SEND_2_MASTER, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

            common::Elite new_elite = common::unpack_elite(buf);
            push_elite(new_elite, elite_pool_count, elite_pool);
            
            std::cerr << "[Master] Received elite from worker " << status.MPI_SOURCE 
                      << ", pool size: " << elite_pool_count << "\n";
        }
        
        // Check for elite pull requests from workers
        MPI_Iprobe(MPI_ANY_SOURCE, common::TAG_PULL_ELITE_WORKER_REQUEST, MPI_COMM_WORLD, &flag_pull, &status);
        
        if(flag_pull) {
            int worker_rank;
            MPI_Recv(&worker_rank, 1, MPI_INT, status.MPI_SOURCE, common::TAG_PULL_ELITE_WORKER_REQUEST, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
            
            common::Elite pulled_elite;
            pull_elite(pulled_elite, elite_pool, elite_pool_count);
            
            auto buf = common::pack_elite(pulled_elite);
            int n = (int) buf.size();
            MPI_Send(&n, 1, MPI_INT, worker_rank, common::TAG_ELITE_MASTER_SEND_PULLED, MPI_COMM_WORLD);
            MPI_Send(buf.data(), n, MPI_INT, worker_rank, common::TAG_ELITE_MASTER_SEND_PULLED, MPI_COMM_WORLD);
            
            std::cerr << "[Master] Sent pulled elite to worker " << worker_rank << "\n";
        }
        
        if (!flag_push && !flag_pull) {
            // No messages, but keep waiting a bit
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    std::cerr << "[Master] Finished after " << iterations << " iterations\n";
}
