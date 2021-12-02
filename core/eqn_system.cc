
#include "eqn_system.h"

#include <Eigen/Core>
#include <Eigen/IterativeLinearSolvers>

NPNR_NAMESPACE_BEGIN

void EquationSystem::solve(std::vector<double> &x, float tolerance)
{
    using namespace Eigen;
    if (x.empty())
        return;
    NPNR_ASSERT(x.size() == A.size());

    VectorXd vx(x.size()), vb(rhs.size());
    SparseMatrix<double> mat(A.size(), A.size());

    std::vector<int> colnnz;
    for (auto &Ac : A)
        colnnz.push_back(int(Ac.size()));
    mat.reserve(colnnz);
    for (int col = 0; col < int(A.size()); col++) {
        auto &Ac = A.at(col);
        for (auto &el : Ac)
            mat.insert(el.first, col) = el.second;
    }

    for (int i = 0; i < int(x.size()); i++)
        vx[i] = x.at(i);
    for (int i = 0; i < int(rhs.size()); i++)
        vb[i] = rhs.at(i);

    ConjugateGradient<SparseMatrix<double>, Lower | Upper> solver;
    solver.setTolerance(tolerance);
    VectorXd xr = solver.compute(mat).solveWithGuess(vb, vx);
    for (int i = 0; i < int(x.size()); i++)
        x.at(i) = xr[i];
}

NPNR_NAMESPACE_END
