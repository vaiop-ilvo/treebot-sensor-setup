#include <Utils/OccupancyMap.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

OccupancyMap buildOccupancyMap(const std::vector<BirdEyePoint>& points, const OccupancyMapConfig& config, bool labelled)
{
    OccupancyMap map;
    map.cellSize = std::max(0.01f, config.cellSize);
    map.cellsX = std::max(1, static_cast<int>(std::lround(config.size / map.cellSize)));
    map.cellsY = map.cellsX;
    map.originX = -0.5f * map.cellsX * map.cellSize;
    map.originY = -0.5f * map.cellsY * map.cellSize;
    map.minHeight = std::min(config.minHeight, config.maxHeight);
    map.maxHeight = std::max(config.minHeight, config.maxHeight);
    map.hasClasses = labelled;

    const size_t nCells = static_cast<size_t>(map.cellsX) * map.cellsY;
    map.occupancy.assign(nCells, OccupancyMap::UNKNOWN);
    map.elevation.assign(nCells, std::numeric_limits<float>::quiet_NaN());
    map.classes.assign(nCells, BirdEyePoint::NO_LABEL);

    // Votes per class in the occupied cells, only for the cells that have labelled points
    std::vector<std::array<uint16_t, 256>> votes;
    std::vector<int32_t> voteIndex(labelled ? nCells : 0, -1);

    for (const BirdEyePoint& p : points) {
        const int ix = static_cast<int>(std::floor((p.forward - map.originX) / map.cellSize));
        const int iy = static_cast<int>(std::floor((p.left - map.originY) / map.cellSize));
        if (ix < 0 || ix >= map.cellsX || iy < 0 || iy >= map.cellsY || p.height > map.maxHeight) {
            continue;   // Outside the map, or above the height range: says nothing about the ground below
        }
        const size_t cell = static_cast<size_t>(iy) * map.cellsX + ix;
        if (p.height < map.minHeight) {
            if (map.occupancy[cell] == OccupancyMap::UNKNOWN) {
                map.occupancy[cell] = OccupancyMap::FREE;
            }
            continue;
        }
        map.occupancy[cell] = OccupancyMap::OCCUPIED;
        if (!(map.elevation[cell] >= p.height)) {     // Also when NaN
            map.elevation[cell] = p.height;
        }
        if (labelled && p.label != BirdEyePoint::NO_LABEL) {
            if (voteIndex[cell] < 0) {
                voteIndex[cell] = static_cast<int32_t>(votes.size());
                votes.push_back({});
            }
            uint16_t& count = votes[voteIndex[cell]][p.label];
            count = static_cast<uint16_t>(std::min<int>(count + 1, UINT16_MAX));
        }
    }

    // Class of a cell: the most common one
    for (size_t cell = 0; cell < voteIndex.size(); ++cell) {
        if (voteIndex[cell] >= 0) {
            const auto& counts = votes[voteIndex[cell]];
            map.classes[cell] = static_cast<uint8_t>(std::max_element(counts.begin(), counts.end()) - counts.begin());
        }
    }
    return map;
}
