#ifndef EQN_SYSTEM_H
#define EQN_SYSTEM_H

#include <utility>
#include <vector>

#include "preface.h"

NPNR_NAMESPACE_BEGIN

// A simple internal representation for a sparse system of equations Ax = rhs
// This is designed to decouple the functions that build the matrix to the engine that
// solves it, and the representation that requires
struct EquationSystem
{

    EquationSystem(size_t rows, size_t cols)
    {
        A.resize(cols);
        rhs.resize(rows);
    }

    // Simple sparse format, easy to convert to CCS for solver
    std::vector<std::vector<std::pair<int, double>>> A; // col -> (row, x[row, col]) sorted by row
    std::vector<double> rhs;                            // RHS vector
    void reset()
    {
        for (auto &col : A)
            col.clear();
        std::fill(rhs.begin(), rhs.end(), 0.0);
    }

    void add_coeff(int row, int col, double val)
    {
        auto &Ac = A.at(col);
        // Binary search
        int b = 0, e = int(Ac.size()) - 1;
        while (b <= e) {
            int i = (b + e) / 2;
            if (Ac.at(i).first == row) {
                Ac.at(i).second += val;
                return;
            }
            if (Ac.at(i).first > row)
                e = i - 1;
            else
                b = i + 1;
        }
        Ac.insert(Ac.begin() + b, std::make_pair(row, val));
    }

    void add_rhs(int row, double val) { rhs[row] += val; }

    void solve(std::vector<double> &x, float tolerance);
};

NPNR_NAMESPACE_END
#endif
