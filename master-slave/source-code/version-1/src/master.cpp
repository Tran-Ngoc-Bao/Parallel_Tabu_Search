#include "master.hpp"

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

static void pull_elite(common::Elite &e, int worker_id) {
    
}

void master(int size) {
    int elite_pool_count = 0;
    std::vector<common::Elite> elite_pool(ELITE_POOL_SIZE);

    MPI_Status status;
    
    while(true) {
        int flag = 0;

        MPI_Iprobe(MPI_ANY_SOURCE, common::TAG_ELITE_WORKER_SEND_2_MASTER, MPI_COMM_WORLD, &flag, &status);

        if(flag) {
            int n;
            MPI_Recv(&n, 1, MPI_INT, status.MPI_SOURCE, common::TAG_ELITE_WORKER_SEND_2_MASTER, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

            std::vector<int> buf(n);
            MPI_Recv(buf.data(), n, MPI_INT, status.MPI_SOURCE, common::TAG_ELITE_WORKER_SEND_2_MASTER, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

            common::Elite new_elite = common::unpack_elite(buf);
            push_elite(new_elite, elite_pool_count, elite_pool);

            return;
        }
    }
}
