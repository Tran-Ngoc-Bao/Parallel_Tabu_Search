#pragma once
#include <cstddef>
#include <vector>

struct LocalSolutionMetrics {
	bool feasible;
	std::vector<double> truck_working_time;
	std::vector<double> drone_working_time;
	double energy_violation;
	double capacity_violation;
	double waiting_time_violation;
	double fixed_time_violation;
};

struct InitialEliteState {
	double working_time;
	std::size_t vehicle;
	std::size_t parent;
	std::size_t index;
	bool is_truck;
	bool operator>(const InitialEliteState& other) const {
		return working_time > other.working_time;
	}
};

void worker(int rank);
