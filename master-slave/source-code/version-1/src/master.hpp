#pragma once
#include "common.hpp"

int count_diff_elite(const Elite &a, const Elite &b);

void push_elite(const Elite &e, int &elite_pool_count, std::vector<Elite> &elite_pool);

Elite pull_elite(int worker_rank);

void master(int size);
