#include <mpi.h>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <algorithm>

const int SOL_SIZE = 20;
const int TAG_SOLUTION = 1;
const int TAG_ELITE = 2;
const int TAG_DONE = 3;

/* ---------- fake objective ---------- */
int evaluate(const std::vector<int>& sol){
    int s = 0;
    for(int v : sol) s += v;
    return s;
}

/* ---------- random solution ---------- */
std::vector<int> random_solution(){
    std::vector<int> sol(SOL_SIZE);
    for(int i=0;i<SOL_SIZE;i++)
        sol[i] = rand()%100;
    return sol;
}

/* ---------- neighborhood move ---------- */
void random_move(std::vector<int>& sol){
    int i = rand()%SOL_SIZE;
    sol[i] = rand()%100;
}

/* ---------- worker tabu search ---------- */
void run_worker(int rank){

    std::vector<int> sol = random_solution();
    int best_cost = evaluate(sol);
    std::vector<int> best_sol = sol;

    MPI_Request req = MPI_REQUEST_NULL;

    for(int iter=0; iter<100000; iter++){

        random_move(sol);
        int cost = evaluate(sol);

        if(cost < best_cost){
            best_cost = cost;
            best_sol = sol;

            // (Good practice) avoid overwriting an in-flight request
            if(req != MPI_REQUEST_NULL){
                MPI_Wait(&req, MPI_STATUS_IGNORE);
                req = MPI_REQUEST_NULL;
            }

            MPI_Isend(best_sol.data(),
                      SOL_SIZE,
                      MPI_INT,
                      0,
                      TAG_SOLUTION,
                      MPI_COMM_WORLD,
                      &req);
        }

        // Check elite solutions from master (asynchronous probe)
        int flag = 0;
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

    // Ensure last Isend (if any) completed before finishing
    if(req != MPI_REQUEST_NULL){
        MPI_Wait(&req, MPI_STATUS_IGNORE);
        req = MPI_REQUEST_NULL;
    }

    std::cout << "Worker " << rank
              << " best cost = " << best_cost
              << std::endl;

    // Notify master that this worker is done
    int done = 1;
    MPI_Send(&done, 1, MPI_INT, 0, TAG_DONE, MPI_COMM_WORLD);
}

/* ---------- master process ---------- */
void run_master(int world_size){

    std::vector<int> elite(SOL_SIZE, 999999);
    int elite_cost = 999999999;

    int done_count = 0;
    const int num_workers = world_size - 1;

    MPI_Status status;

    while(done_count < num_workers){

        int flag = 0;

        // 1) Check DONE messages first (so we can exit)
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

        // 2) Check improved solutions
        MPI_Iprobe(MPI_ANY_SOURCE, TAG_SOLUTION, MPI_COMM_WORLD, &flag, &status);
        if(flag){

            std::vector<int> sol(SOL_SIZE);

            MPI_Recv(sol.data(),
                     SOL_SIZE,
                     MPI_INT,
                     status.MPI_SOURCE,
                     TAG_SOLUTION,
                     MPI_COMM_WORLD,
                     &status);

            int cost = evaluate(sol);

            if(cost < elite_cost){
                elite = sol;
                elite_cost = cost;

                std::cout << "New elite cost = "
                          << elite_cost
                          << std::endl;

                // Broadcast elite solution (blocking is OK for demo, but Isend is safer)
                for(int w=1; w<world_size; w++){
                    MPI_Send(elite.data(),
                             SOL_SIZE,
                             MPI_INT,
                             w,
                             TAG_ELITE,
                             MPI_COMM_WORLD);
                }
            }
        }

        // Optional: prevent busy-spin from pegging CPU at 100%
        // You can remove this if you prefer.
        // MPI_Barrier is NOT needed; just a tiny sleep would be fine too.
        // (No standard sleep in MPI; in C++ you could use std::this_thread::sleep_for)
    }

    std::cout << "Master: all workers finished. Exiting master loop." << std::endl;
}

int main(int argc, char** argv){

    MPI_Init(&argc,&argv);

    int rank, size;
    MPI_Comm_rank(MPI_COMM_WORLD,&rank);
    MPI_Comm_size(MPI_COMM_WORLD,&size);

    srand(time(NULL)+rank);

    if(rank == 0)
        run_master(size);
    else
        run_worker(rank);

    MPI_Finalize();
    return 0;
}