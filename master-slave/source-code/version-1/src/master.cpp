#include "common.hpp"
#include "master.hpp"

const int ELITE_POOL_SIZE = 6;

void master(int size) {
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
        }
    }
}