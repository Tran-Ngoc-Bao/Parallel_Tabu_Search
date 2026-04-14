#include "common.hpp"

static void pack_int(std::vector<int> &buf, int x) {
    buf.push_back(x);
}

static int unpack_int(const std::vector<int> &buf, size_t &i) {
    return buf[i++];
}

std::vector<int> pack_elite(const Elite &e) {
    std::vector<int> buf;
    buf.reserve(128);

    pack_int(buf, e.worker_rank);
    pack_int(buf, (int) e.elements.size());
    for (const auto &el : e.elements) {
        pack_int(buf, el.type);
        pack_int(buf, el.vehicle_number);

        pack_int(buf, (int) el.trips.size());
        for (const auto &tr : el.trips) {
            pack_int(buf, (int) tr.customers.size());
            for (const auto &[cus, nxt] : tr.customers) {
                pack_int(buf, cus);
                pack_int(buf, nxt);
            }
        }
    }
    return buf;
}

Elite unpack_elite(const std::vector<int> &buf) {
    Elite e;
    size_t i = 0;

    e.worker_rank = unpack_int(buf, i);
    int elements_count = unpack_int(buf, i);
    e.elements.resize(elements_count);

    for (int ei = 0; ei < elements_count; ++ei) {
        auto &el = e.elements[ei];
        el.type = unpack_int(buf, i);
        el.vehicle_number = unpack_int(buf, i);

        int trips_count = unpack_int(buf, i);
        el.trips.resize(trips_count);

        for (int ti = 0; ti < trips_count; ++ti) {
            auto &tr = el.trips[ti];

            int customers_count = unpack_int(buf, i);

            for (int ci = 0; ci < customers_count; ++ci) {
                int cus = unpack_int(buf, i);
                int nxt = unpack_int(buf, i);
                tr.customers[cus] = nxt;
            }
        }
    }
    return e;
}
