#include <mpi.h>
#include <omp.h>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <algorithm>

const int SOL_SIZE = 1000;
const int TAG_SOL = 1;
const int TAG_ELITE = 2;
const int TAG_DONE = 3;

int evaluate(const std::vector<int>& sol){
    int s = 0;
    for(int v : sol) s += v;
    return s;
}

std::vector<int> random_solution(){
    std::vector<int> sol(SOL_SIZE);
    for(int i=0;i<SOL_SIZE;i++)
        sol[i] = rand()%100;
    return sol;
}

std::vector<int> best_neighbor_parallel(std::vector<int>& sol){

    int best_cost = 1e9;
    std::vector<int> best_sol = sol;

    #pragma omp parallel
    {
        std::vector<int> local_sol = sol;
        int local_best = best_cost;
        std::vector<int> local_best_sol = sol;

        #pragma omp for
        for(int i=0;i<100;i++){

            std::vector<int> candidate = local_sol;

            int pos = rand()%SOL_SIZE;
            candidate[pos] = rand()%100;

            int cost = evaluate(candidate);

            if(cost < local_best){
                local_best = cost;
                local_best_sol = candidate;
            }
        }

        #pragma omp critical
        {
            if(local_best < best_cost){
                best_cost = local_best;
                best_sol = local_best_sol;
            }
        }
    }

    return best_sol;
}

void run_worker(int rank){

    std::vector<int> sol = random_solution();
    int best_cost = evaluate(sol);
    std::vector<int> best_sol = sol;

    MPI_Request req = MPI_REQUEST_NULL;

    for(int iter=0; iter<50000; iter++){

        sol = best_neighbor_parallel(sol);
        int cost = evaluate(sol);

        if(cost < best_cost){
            best_cost = cost;
            best_sol = sol;

            // Avoid overwriting an in-flight request (simple & safe)
            if(req != MPI_REQUEST_NULL){
                MPI_Wait(&req, MPI_STATUS_IGNORE);
                req = MPI_REQUEST_NULL;
            }

            MPI_Isend(best_sol.data(),
                      SOL_SIZE,
                      MPI_INT,
                      0,
                      TAG_SOL,
                      MPI_COMM_WORLD,
                      &req);
        }

        int flag;
        MPI_Status status;

        MPI_Iprobe(0, TAG_ELITE, MPI_COMM_WORLD, &flag, &status);

        if(flag){
            std::vector<int> elite(SOL_SIZE);

            MPI_Recv(elite.data(),
                     SOL_SIZE,
                     MPI_INT,
                     0,
                     TAG_ELITE,
                     MPI_COMM_WORLD,
                     &status);

            int elite_cost = evaluate(elite);

            if(elite_cost < best_cost){
                sol = elite;
                best_sol = elite;
                best_cost = elite_cost;
            }
        }
    }

    // Ensure last Isend completed (if any) before finishing
    if(req != MPI_REQUEST_NULL){
        MPI_Wait(&req, MPI_STATUS_IGNORE);
        req = MPI_REQUEST_NULL;
    }

    std::cout << "Worker " << rank
              << " best = " << best_cost
              << std::endl;

    // Notify master: this worker finished
    int done = 1;
    MPI_Send(&done, 1, MPI_INT, 0, TAG_DONE, MPI_COMM_WORLD);
}

void run_master(int size){

    std::vector<int> elite(SOL_SIZE, 999999);
    int elite_cost = 1e9;

    int done_count = 0;
    const int num_workers = size - 1;

    MPI_Status status;

    while(done_count < num_workers){

        int flag = 0;

        // 1) Check DONE first so we can exit as soon as possible
        MPI_Iprobe(MPI_ANY_SOURCE, TAG_DONE, MPI_COMM_WORLD, &flag, &status);
        if(flag){
            int dummy;
            MPI_Recv(&dummy, 1, MPI_INT, status.MPI_SOURCE, TAG_DONE, MPI_COMM_WORLD, &status);
            done_count++;

            std::cout << "Master: received DONE from worker " << status.MPI_SOURCE
                      << " (" << done_count << "/" << num_workers << ")"
                      << std::endl;
            continue;
        }

        // 2) Check solutions
        MPI_Iprobe(MPI_ANY_SOURCE, TAG_SOL, MPI_COMM_WORLD, &flag, &status);
        if(flag){

            std::vector<int> sol(SOL_SIZE);

            MPI_Recv(sol.data(),
                     SOL_SIZE,
                     MPI_INT,
                     status.MPI_SOURCE,
                     TAG_SOL,
                     MPI_COMM_WORLD,
                     &status);

            int cost = evaluate(sol);

            if(cost < elite_cost){

                elite = sol;
                elite_cost = cost;

                std::cout << "New elite = "
                          << elite_cost
                          << std::endl;

                /* // Broadcast elite (blocking send as in your original code)
                for(int w=1; w<size; w++){
                    MPI_Send(elite.data(),
                             SOL_SIZE,
                             MPI_INT,
                             w,
                             TAG_ELITE,
                             MPI_COMM_WORLD);
                } */

                // Đổi broadcast elite ở master sang non-blocking send
                std::vector<MPI_Request> elite_req(size, MPI_REQUEST_NULL);

                for (int w=1; w<size; w++) {
                    // nếu lần trước gửi cho worker w chưa xong thì đợi/hoặc test
                    if (elite_req[w] != MPI_REQUEST_NULL) {
                        int done = 0;
                        MPI_Test(&elite_req[w], &done, MPI_STATUS_IGNORE);
                        if (!done) MPI_Wait(&elite_req[w], MPI_STATUS_IGNORE);
                        elite_req[w] = MPI_REQUEST_NULL;
                    }

                    MPI_Isend(elite.data(), SOL_SIZE, MPI_INT, w, TAG_ELITE, MPI_COMM_WORLD, &elite_req[w]);
                }
            }
        }
    }

    std::cout << "Master: all workers finished. Exiting." << std::endl;
}

int main(int argc, char** argv){
    MPI_Init(&argc, &argv);

    double t0 = MPI_Wtime();

    int rank, size;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &size);

    if(rank == 0) run_master(size);
    else run_worker(rank);

    double t1 = MPI_Wtime();

    if(rank == 0) {
        std::cout << "Total wall time = " << (t1 - t0) << " s\n";
    }

    MPI_Finalize();
    return 0;
}