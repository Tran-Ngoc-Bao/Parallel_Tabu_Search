#include "common.hpp"
#include "worker.hpp"

void worker(int rank) {
    auto buf = pack_elite({rank, {}});

    int n = (int) buf.size();
    MPI_Send(&n, 1, MPI_INT, MASTER_RANK, TAG_ELITE_WORKER_SEND_2_MASTER, MPI_COMM_WORLD);
    MPI_Send(buf.data(), n, MPI_INT, MASTER_RANK, TAG_ELITE_WORKER_SEND_2_MASTER, MPI_COMM_WORLD);
    std::cout << "HELLO";
}
