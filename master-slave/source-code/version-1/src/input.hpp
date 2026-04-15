#pragma once

#include <cstddef>
#include <string>
#include <vector>

struct ProblemInput {
    std::size_t customers_count = 0;
    std::size_t trucks_count = 0;
    std::size_t drones_count = 0;

    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> demands;
    std::vector<bool> dronable;

    std::string file_path;
};

std::string resolve_data_dir(const std::string &base_data_dir = "");
ProblemInput load_problem_input(const std::string &dataset_name,
                                const std::string &base_data_dir = "");
