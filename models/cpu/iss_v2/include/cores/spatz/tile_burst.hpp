// Tile-contained VLSU segmentation. Addresses and lengths are bytes.
#pragma once
#include <algorithm>
#include <cstdint>
namespace spatz_burst {
inline uint64_t segment(uint64_t addr, uint64_t bytes, unsigned banks, unsigned maximum) {
    return std::min({bytes, uint64_t(maximum)*4, (banks-(addr/4)%banks)*4});
}
inline bool eligible(uint64_t addr, uint64_t bytes, unsigned banks, unsigned maximum,
                     unsigned ports, unsigned capacity_words) {
    return addr%4 == 0 && bytes%4 == 0 && bytes >= 8 && bytes <= uint64_t(capacity_words)*4
        && (maximum%ports == 0 || bytes <= uint64_t(maximum)*4)
        && (bytes <= (banks-(addr/4)%banks)*4 ||
            (addr%(ports*4) == 0 && banks%ports == 0 && maximum%ports == 0));
}
inline unsigned count(uint64_t addr, uint64_t bytes, unsigned banks, unsigned maximum) {
    unsigned n=0;
    while (bytes) { auto size=segment(addr,bytes,banks,maximum); addr+=size; bytes-=size; ++n; }
    return n;
}
}
