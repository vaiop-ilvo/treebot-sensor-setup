#include <Utils/Blobs.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <unordered_map>

namespace {
    // Label (8 bits) and cell index (18 bits per axis, offset so negative indices fit) in one key
    constexpr int OFFSET = 1 << 17;
    uint64_t cellKey(int ix, int iy, int iz, uint8_t label)
    {
        constexpr uint64_t MASK = (1u << 18) - 1;
        return (static_cast<uint64_t>(label) << 54) | ((static_cast<uint64_t>(ix + OFFSET) & MASK) << 36)
             | ((static_cast<uint64_t>(iy + OFFSET) & MASK) << 18) | (static_cast<uint64_t>(iz + OFFSET) & MASK);
    }

    // Union-find over the occupied cells
    struct DisjointSets {
        std::vector<uint32_t> parent;
        explicit DisjointSets(size_t n) : parent(n) { std::iota(parent.begin(), parent.end(), 0u); }
        uint32_t find(uint32_t a)
        {
            while (parent[a] != a) {
                parent[a] = parent[parent[a]];
                a = parent[a];
            }
            return a;
        }
        void unite(uint32_t a, uint32_t b)
        {
            a = find(a);
            b = find(b);
            if (a != b) {
                parent[std::max(a, b)] = std::min(a, b);
            }
        }
    };

    // Share of the points the circle covers
    constexpr float RADIUS_QUANTILE = 0.9f;
    constexpr float MIN_RADIUS = 0.1f;
}

BlobResult findBlobs(const std::vector<vaiop::QtPoint>& cloud, const std::vector<uint8_t>& labels,
                     const std::vector<cv::Vec3b>& palette, uint32_t minPoints, float linkDistance,
                     const std::vector<uint8_t>& include)
{
    BlobResult result;
    result.pointBlob.assign(cloud.size(), -1);
    if (linkDistance <= 0.0f || labels.size() != cloud.size()) {
        return result;
    }

    // 1. Occupied cells per class
    struct Cell {
        int ix, iy, iz;
        uint8_t label;
    };
    std::unordered_map<uint64_t, uint32_t> cellIndex;
    std::vector<Cell> cells;
    std::vector<int32_t> pointCell(cloud.size(), -1);
    cellIndex.reserve(cloud.size() / 4);
    for (size_t i = 0; i < cloud.size(); ++i) {
        const uint8_t label = labels[i];
        if (label >= palette.size() || (!include.empty() && !include[i])) {
            continue;
        }
        const QVector3D& p = cloud[i].position;
        const int ix = static_cast<int>(std::floor(p.x() / linkDistance));
        const int iy = static_cast<int>(std::floor(p.y() / linkDistance));
        const int iz = static_cast<int>(std::floor(p.z() / linkDistance));
        const auto [it, inserted] = cellIndex.try_emplace(cellKey(ix, iy, iz, label), static_cast<uint32_t>(cells.size()));
        if (inserted) {
            cells.push_back({ix, iy, iz, label});
        }
        pointCell[i] = static_cast<int32_t>(it->second);
    }

    // 2. Connect each cell with its occupied neighbours of the same class
    DisjointSets sets(cells.size());
    for (uint32_t c = 0; c < cells.size(); ++c) {
        const Cell& cell = cells[c];
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    // Each pair once: only the neighbours "after" this cell
                    if (dx < 0 || (dx == 0 && (dy < 0 || (dy == 0 && dz <= 0)))) {
                        continue;
                    }
                    const auto it = cellIndex.find(cellKey(cell.ix + dx, cell.iy + dy, cell.iz + dz, cell.label));
                    if (it != cellIndex.end()) {
                        sets.unite(c, it->second);
                    }
                }
            }
        }
    }

    // 3. Points and centroid per group
    std::vector<uint32_t> groupPoints(cells.size(), 0);
    std::vector<QVector3D> groupSum(cells.size());
    for (size_t i = 0; i < cloud.size(); ++i) {
        if (pointCell[i] >= 0) {
            const uint32_t group = sets.find(static_cast<uint32_t>(pointCell[i]));
            groupPoints[group]++;
            groupSum[group] += cloud[i].position;
        }
    }

    // 4. Keep the large groups
    std::vector<int32_t> groupBlob(cells.size(), -1);
    for (uint32_t g = 0; g < cells.size(); ++g) {
        if (groupPoints[g] == 0 || groupPoints[g] < minPoints) {
            continue;
        }
        const cv::Vec3b& rgb = palette[cells[g].label];
        groupBlob[g] = static_cast<int32_t>(result.blobs.size());
        result.blobs.push_back({groupSum[g] / static_cast<float>(groupPoints[g]), QVector3D(rgb[0], rgb[1], rgb[2]) / 255.0f,
                                0.0f, cells[g].label, groupPoints[g]});
    }

    // 5. Radius: covers most points of the blob
    std::vector<std::vector<float>> distances(result.blobs.size());
    for (size_t i = 0; i < cloud.size(); ++i) {
        if (pointCell[i] < 0) {
            continue;
        }
        const int32_t blob = groupBlob[sets.find(static_cast<uint32_t>(pointCell[i]))];
        if (blob >= 0) {
            result.pointBlob[i] = blob;
            distances[blob].push_back((cloud[i].position - result.blobs[blob].center).length());
        }
    }
    for (size_t b = 0; b < result.blobs.size(); ++b) {
        std::vector<float>& d = distances[b];
        const auto nth = d.begin() + static_cast<std::ptrdiff_t>(RADIUS_QUANTILE * (d.size() - 1));
        std::nth_element(d.begin(), nth, d.end());
        result.blobs[b].radius = std::max(MIN_RADIUS, *nth);
    }
    return result;
}
