#include "master.hpp"

const int ELITE_POOL_SIZE = 6;

int count_diff_elite(const Elite &a, const Elite &b) {
    auto extract = [](const Elite &e) {
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

void push_elite(const Elite &e, int &elite_pool_count, std::vector<Elite> &elite_pool) {
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

void pull_elite(Elite &e, int worker_id) {
    
}

void master(int size, const ProblemInput &problem_input) {
    (void) size;

    std::cout << "Loaded dataset: " << problem_input.file_path
              << " (customers=" << problem_input.customers_count
              << ", trucks=" << problem_input.trucks_count
              << ", drones=" << problem_input.drones_count << ")\n";

    int elite_pool_count = 0;
    std::vector<Elite> elite_pool(ELITE_POOL_SIZE);

    MPI_Status status;

    while(true) {
        int flag = 0;

        MPI_Iprobe(MPI_ANY_SOURCE, TAG_ELITE_WORKER_SEND_2_MASTER, MPI_COMM_WORLD, &flag, &status);

        if(flag) {
            int n;
            MPI_Recv(&n, 1, MPI_INT, status.MPI_SOURCE, TAG_ELITE_WORKER_SEND_2_MASTER, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

            std::vector<int> buf(n);
            MPI_Recv(buf.data(), n, MPI_INT, status.MPI_SOURCE, TAG_ELITE_WORKER_SEND_2_MASTER, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

            Elite new_elite = unpack_elite(buf);
            push_elite(new_elite, elite_pool_count, elite_pool);

        }
    }
}
