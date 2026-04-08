#include <mpi.h>
#include <omp.h>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <algorithm>

const int SOL_SIZE = 50;
const int TAG_SOL = 1;
const int TAG_ELITE = 2;

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

    MPI_Request req;

    for(int iter=0; iter<50000; iter++){

        sol = best_neighbor_parallel(sol);

        int cost = evaluate(sol);

        if(cost < best_cost){
            best_cost = cost;
            best_sol = sol;

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

        MPI_Iprobe(0,TAG_ELITE,MPI_COMM_WORLD,&flag,&status);

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

    std::cout << "Worker " << rank
              << " best = " << best_cost
              << std::endl;
}

void run_master(int size){

    std::vector<int> elite(SOL_SIZE,999999);
    int elite_cost = 1e9;

    MPI_Status status;

    while(true){

        int flag;

        MPI_Iprobe(MPI_ANY_SOURCE,
                   TAG_SOL,
                   MPI_COMM_WORLD,
                   &flag,
                   &status);

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

                for(int w=1; w<size; w++){

                    MPI_Send(elite.data(),
                             SOL_SIZE,
                             MPI_INT,
                             w,
                             TAG_ELITE,
                             MPI_COMM_WORLD);
                }
            }
        }
    }
}

int main(int argc,char** argv){

    MPI_Init(&argc,&argv);

    int rank,size;

    MPI_Comm_rank(MPI_COMM_WORLD,&rank);
    MPI_Comm_size(MPI_COMM_WORLD,&size);

    srand(time(NULL)+rank);

    if(rank==0)
        run_master(size);
    else
        run_worker(rank);

    MPI_Finalize();

    return 0;
}
