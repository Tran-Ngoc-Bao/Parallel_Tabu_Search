#include <mpi.h>
#include <omp.h>

#include "input.hpp"
#include "master.hpp"
#include "worker.hpp"

int main(int argc, char** argv) {

    MPI_Init(&argc, &argv);

    int rank, size;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &size);

    if(rank == 0) {
        master(size);
    } else {
        worker(rank);
    }

    MPI_Finalize();
    return 0;
}