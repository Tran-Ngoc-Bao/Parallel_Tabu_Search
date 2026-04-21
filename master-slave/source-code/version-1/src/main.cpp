#include "common.hpp"
#include "input.hpp"
#include "master.hpp"
#include "worker.hpp"

int main(int argc, char* *argv) {
    input(argc, argv);

    MPI_Init(&argc, &argv);

    double t0 = MPI_Wtime();

    int rank, size;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &size);

    if(rank == common::MASTER_RANK) {
        master(size);
    } else {
        worker(rank);
    }

    double t1 = MPI_Wtime();

    if(rank == common::MASTER_RANK) {
        std::cout << "Total wall time = " << (t1 - t0) << " s\n";
    }

    MPI_Finalize();
    return 0;
}
