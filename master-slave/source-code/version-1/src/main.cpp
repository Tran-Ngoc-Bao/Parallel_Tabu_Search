#include <mpi.h>
#include <omp.h>
#include <exception>
#include <iostream>
#include <string>

#include "common.hpp"
#include "input.hpp"
#include "master.hpp"
#include "worker.hpp"

namespace {

struct RuntimeOptions {
    std::string dataset_name = "6.10.1.txt";
    std::string data_dir;
    bool show_help = false;
};

RuntimeOptions parse_runtime_options(int argc, char **argv) {
    RuntimeOptions options;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--dataset" && i + 1 < argc) {
            options.dataset_name = argv[++i];
            continue;
        }
        if (arg == "--data-dir" && i + 1 < argc) {
            options.data_dir = argv[++i];
            continue;
        }
        if (!arg.empty() && arg[0] != '-') {
            options.dataset_name = arg;
            continue;
        }
        if (arg == "--help") {
            options.show_help = true;
            break;
        }
    }

    return options;
}

} // namespace

int main(int argc, char** argv) {
    RuntimeOptions options = parse_runtime_options(argc, argv);
    if (options.show_help) {
        std::cout << "Usage: <program> [dataset] [--dataset <name>] [--data-dir <path>]\n";
        return 0;
    }

    MPI_Init(&argc, &argv);

    double t0 = MPI_Wtime();

    int rank, size;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &size);

    if(rank == MASTER_RANK) {
        ProblemInput problem_input;
        try {
            problem_input = load_problem_input(options.dataset_name, options.data_dir);
        } catch (const std::exception &e) {
            std::cerr << e.what() << "\n";
            MPI_Abort(MPI_COMM_WORLD, 1);
        }
        master(size, problem_input);
    } else {
        worker(rank);
    }

    double t1 = MPI_Wtime();

    if(rank == MASTER_RANK) {
        std::cout << "Total wall time = " << (t1 - t0) << " s\n";
    }

    MPI_Finalize();
    return 0;
}
