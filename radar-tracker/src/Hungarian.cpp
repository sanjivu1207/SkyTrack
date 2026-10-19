#include "Hungarian.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>

std::vector<int> hungarian(const std::vector<std::vector<double>>& cost) {
    const std::size_t rows = cost.size();
    if (rows == 0) return {};
    const std::size_t cols = cost[0].size();
    const std::size_t n = std::max(rows, cols);
    const double kInf = std::numeric_limits<double>::infinity();

    auto c = [&](std::size_t i, std::size_t j) {
        return (i < rows && j < cols) ? cost[i][j] : 0.0;
    };

    std::vector<double> u(n + 1, 0.0), v(n + 1, 0.0);
    std::vector<std::size_t> p(n + 1, 0), way(n + 1, 0);

    for (std::size_t i = 1; i <= n; ++i) {
        p[0] = i;
        std::size_t j0 = 0;
        std::vector<double> minv(n + 1, kInf);
        std::vector<bool> used(n + 1, false);
        do {
            used[j0] = true;
            const std::size_t i0 = p[j0];
            double delta = kInf;
            std::size_t j1 = 0;
            for (std::size_t j = 1; j <= n; ++j) {
                if (used[j]) continue;
                const double cur = c(i0 - 1, j - 1) - u[i0] - v[j];
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }
            for (std::size_t j = 0; j <= n; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do {
            const std::size_t j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    std::vector<int> assignment(rows, -1);
    for (std::size_t j = 1; j <= n; ++j) {
        const std::size_t i = p[j];
        if (i >= 1 && i <= rows && j <= cols) assignment[i - 1] = static_cast<int>(j - 1);
    }
    return assignment;
}
