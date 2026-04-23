#pragma once
#include "common.hpp"
#include "config.hpp"

#include <string>
#include <random>
#include <vector>

namespace solutions {

enum class Neighborhood {
    Move10,
    Move11,
    Move20,
    Move21,
    Move22,
    TwoOpt,
    EjectionChain
};

inline std::string neighborhood_to_str(Neighborhood n) {
    switch(n) {
        case Neighborhood::Move10:        return "Move (1, 0)";
        case Neighborhood::Move11:        return "Move (1, 1)";
        case Neighborhood::Move20:        return "Move (2, 0)";
        case Neighborhood::Move21:        return "Move (2, 1)";
        case Neighborhood::Move22:        return "Move (2, 2)";
        case Neighborhood::TwoOpt:        return "2-opt";
        case Neighborhood::EjectionChain: return "Ejection-chain";
    }
    return "";
}

common::Elite apply_move10(const Config &cfg, const common::Elite &elite, std::mt19937_64 &rng);
common::Elite apply_move11(const Config &cfg, const common::Elite &elite, std::mt19937_64 &rng);
common::Elite apply_move20(const Config &cfg, const common::Elite &elite, std::mt19937_64 &rng);
common::Elite apply_move21(const Config &cfg, const common::Elite &elite, std::mt19937_64 &rng);
common::Elite apply_move22(const Config &cfg, const common::Elite &elite, std::mt19937_64 &rng);
common::Elite apply_twoopt(const Config &cfg, const common::Elite &elite, std::mt19937_64 &rng);
common::Elite apply_ejection_chain(const Config &cfg, const common::Elite &elite, std::mt19937_64 &rng);

std::vector<int> generate_neighborhood_order(std::mt19937_64 &rng);

double compute_elite_cost(const Config &cfg, const common::Elite &elite);

} // namespace solutions
