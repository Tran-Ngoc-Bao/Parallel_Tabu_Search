/*Rank 0: master (quản lý elite pool)

Rank ≥1: worker (chạy tabu search)

Workers gửi best solution lên master bằng MPI_Isend

Workers không chờ nhau (asynchronous)

Master nhận solution bằng MPI_Iprobe*/

#include <mpi.h>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <algorithm>

const int SOL_SIZE = 20;
const int TAG_SOLUTION = 1;
const int TAG_ELITE = 2;

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

    MPI_Request req;

    for(int iter=0; iter<100000; iter++){

        /* simple neighborhood */
        random_move(sol);

        int cost = evaluate(sol);

        if(cost < best_cost){
            best_cost = cost;
            best_sol = sol;

            /* send solution to master */
            MPI_Isend(best_sol.data(),
                      SOL_SIZE,
                      MPI_INT,
                      0,
                      TAG_SOLUTION,
                      MPI_COMM_WORLD,
                      &req);
        }

        /* check elite solutions from master */
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

    std::cout << "Worker " << rank
              << " best cost = " << best_cost
              << std::endl;
}

/* ---------- master process ---------- */
void run_master(int world_size){

    std::vector<int> elite(SOL_SIZE, 999999);

    int elite_cost = 999999999;

    MPI_Status status;

    while(true){

        int flag;

        MPI_Iprobe(MPI_ANY_SOURCE,
                   TAG_SOLUTION,
                   MPI_COMM_WORLD,
                   &flag,
                   &status);

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

                /* broadcast elite solution */
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
    }
}

int main(int argc, char** argv){

    MPI_Init(&argc,&argv);

    int rank;
    int size;

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
