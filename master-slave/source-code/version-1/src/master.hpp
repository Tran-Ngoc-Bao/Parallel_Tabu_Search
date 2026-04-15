#pragma once
#include "common.hpp"
#include "input.hpp"

int count_diff_elite(const Elite &a, const Elite &b);

void push_elite(const Elite &e, int &elite_pool_count, std::vector<Elite> &elite_pool);

Elite pop_elite();

void master(int size, const ProblemInput &problem_input);
