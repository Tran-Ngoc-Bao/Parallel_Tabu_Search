#include "input.hpp"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <regex>
#include <stdexcept>

namespace {

std::string read_file(const std::string &path) {
    std::ifstream f(path);
    if (!f) {
        throw std::runtime_error("Cannot open file: " + path);
    }
    return std::string((std::istreambuf_iterator<char>(f)),
                       std::istreambuf_iterator<char>());
}

std::string resolve_problem_file(const std::string &dataset_name,
                                 const std::string &base_data_dir) {
    namespace fs = std::filesystem;

    fs::path dataset_path(dataset_name);
    if (dataset_path.is_absolute() && fs::exists(dataset_path)) {
        return dataset_path.string();
    }
    if (fs::exists(dataset_path)) {
        return fs::absolute(dataset_path).string();
    }

    const fs::path data_dir(resolve_data_dir(base_data_dir));
    fs::path candidate = data_dir / dataset_name;
    if (fs::exists(candidate)) {
        return candidate.string();
    }

    if (dataset_name.size() < 4 || dataset_name.substr(dataset_name.size() - 4) != ".txt") {
        fs::path txt_candidate = data_dir / (dataset_name + ".txt");
        if (fs::exists(txt_candidate)) {
            return txt_candidate.string();
        }
    }

    throw std::runtime_error("Cannot find dataset: " + dataset_name);
}

} // namespace

std::string resolve_data_dir(const std::string &base_data_dir) {
    namespace fs = std::filesystem;

    if (!base_data_dir.empty()) {
        fs::path p(base_data_dir);
        if (!p.is_absolute()) {
            p = fs::absolute(p);
        }
        return p.string();
    }

    fs::path current = fs::current_path();
    for (;;) {
        fs::path candidate = current / "data";
        if (fs::exists(candidate) && fs::is_directory(candidate)) {
            return candidate.string();
        }
        if (!current.has_parent_path() || current.parent_path() == current) {
            break;
        }
        current = current.parent_path();
    }

    return (fs::current_path() / "data").string();
}

ProblemInput load_problem_input(const std::string &dataset_name,
                                const std::string &base_data_dir) {
    ProblemInput input;
    input.file_path = resolve_problem_file(dataset_name, base_data_dir);

    std::string data = read_file(input.file_path);

    {
        std::regex re_trucks(R"(trucks_count (\d+))");
        std::smatch m;
        if (!std::regex_search(data, m, re_trucks)) {
            throw std::runtime_error("Missing trucks count");
        }
        input.trucks_count = static_cast<std::size_t>(std::stoul(m[1].str()));
    }
    {
        std::regex re_drones(R"(drones_count (\d+))");
        std::smatch m;
        if (!std::regex_search(data, m, re_drones)) {
            throw std::runtime_error("Missing drones count");
        }
        input.drones_count = static_cast<std::size_t>(std::stoul(m[1].str()));
    }

    std::pair<double, double> depot;
    {
        std::regex re_depot(R"(depot (-?[\d\.]+)\s+(-?[\d\.]+))");
        std::smatch m;
        if (!std::regex_search(data, m, re_depot)) {
            throw std::runtime_error("Missing depot coordinates");
        }
        depot = {std::stod(m[1].str()), std::stod(m[2].str())};
    }

    input.x = {depot.first};
    input.y = {depot.second};
    input.demands = {0.0};
    input.dronable = {true};

    std::regex re_customer(
        R"(^\s*(-?[\d\.]+)\s+(-?[\d\.]+)\s+(0|1)\s+([\d\.]+)\s*$)",
        std::regex::multiline);
    auto begin = std::sregex_iterator(data.begin(), data.end(), re_customer);
    auto end = std::sregex_iterator();
    for (auto it = begin; it != end; ++it) {
        const std::smatch &match = *it;
        input.x.push_back(std::stod(match[1].str()));
        input.y.push_back(std::stod(match[2].str()));
        input.dronable.push_back(match[3].str() == "1");
        input.demands.push_back(std::stod(match[4].str()));
        ++input.customers_count;
    }

    return input;
}
