#include "global_placer.h"
#include "eqn_system.h"
#include "log.h"
#include "netlist.h"
#include "netlist_access.h"

NPNR_NAMESPACE_BEGIN

namespace {
struct GlobalPlacer
{
    GlobalPlacer(Context &ctx, Module &mod) : ctx(ctx), netlist(ctx.netlist()), mod(mod){};
    Context &ctx;
    Netlist &netlist;
    Module &mod;

    Size grid;
    int rows;
    int cols;

    struct InstData
    {
        int row, col;
        index_t var_idx = -1;
        double solver_x, solver_y;
        int width, height;
        bool fixed = false;
        bool flip_x = false;
        Orientation get_orient() const
        {
            // Odd rows are flipped in y
            return (row % 1) ? (flip_x ? Orientation::FN : Orientation::S)
                             : (flip_x ? Orientation::FS : Orientation::N);
        }
    };

    std::vector<InstData> insts;
    std::vector<index_t> var2idx;

    dist_t row_pos(int row) { return ((row + 1) / 2) * (2 * grid.height); }

    void setup_cells()
    {
        insts.resize(mod.insts.size());
        for (auto &inst : mod.insts) {
            auto &data = insts.at(inst->index.idx());
            auto &type = netlist.cell_types[inst->type];
            NPNR_ASSERT((type.size.width % grid.width) == 0);
            data.width = type.size.width / grid.width;
            NPNR_ASSERT((type.size.height % grid.height) == 0);
            data.height = type.size.height / grid.height;
            if (inst->fixed) {
                // Fixed placement
                data.fixed = true;
                data.col = (inst->placement->loc.x / grid.width);
                data.row = (inst->placement->loc.y / (2 * grid.height)) * 2;
                if (inst->placement->orient == Orientation::FN || inst->placement->orient == Orientation::S)
                    data.row -= 1;
                if (inst->placement->orient == Orientation::FN || inst->placement->orient == Orientation::FS)
                    data.flip_x = true;
                else
                    data.flip_x = false;
            } else {
                // TODO: better seed placement than random
                data.fixed = false;
                data.col = ctx.rng.rng((cols - data.width) + 1);
                data.row = ctx.rng.rng(rows);
                data.flip_x = false;
                // Assign a matrix variable for this cell
                data.var_idx = int(var2idx.size());
                var2idx.push_back(inst->index.idx());
            }
            data.solver_x = data.col;
            data.solver_y = data.row;
        }
    }

    InstData &inst_data(store_index<CellInst> inst) { return insts.at(inst.idx()); }

    const float alpha = 0.1;

    struct PortOrPin
    {
        PortRef ref;
        double pos;
        PortOrPin min(const PortOrPin &other) { return (other.pos < pos) ? other : *this; }
        PortOrPin max(const PortOrPin &other) { return (other.pos > pos) ? other : *this; }
    };

    void build_equations(EquationSystem &es, bool yaxis, int iter = -1)
    {
        // Return the x or y position of a cell, depending on ydir
        auto solver_pos = [&](store_index<CellInst> inst) {
            return yaxis ? inst_data(inst).solver_y : inst_data(inst).solver_x;
        };
        auto legaliser_pos = [&](store_index<CellInst> inst) {
            return yaxis ? inst_data(inst).row : inst_data(inst).col;
        };
        auto pin_pos = [&](Point pin) { return yaxis ? (double(pin.y) / grid.height) : (double(pin.x) / grid.width); };
        es.reset();

        // Find the bounds of the net in this axis, and the ports that correspond to these bounds
        PortOrPin lbport{{}, std::numeric_limits<double>::max()}, ubport{{}, std::numeric_limits<double>::min()};

        for (auto &net : mod.nets) {
            if ((index_t(net->inst_ports.size()) + index_t(net->ext_pins.size())) < 2)
                continue;
            // Find the bounding ports/pins of the net
            for (auto pin : net->ext_pins) {
                lbport = lbport.min({{}, pin_pos(pin)});
                ubport = ubport.max({{}, pin_pos(pin)});
            }
            for (auto port : net->inst_ports) {
                double pos = solver_pos(port.inst);
                lbport = lbport.min({port, pos});
                ubport = ubport.max({port, pos});
            }
            // Add each pin and port to the matrix
            auto stamp_equation = [&](PortOrPin var, PortOrPin eqn, double weight) {
                if (!eqn.ref || inst_data(eqn.ref.inst).var_idx == -1)
                    return;
                int mat_row = inst_data(eqn.ref.inst).var_idx;
                int mat_col = var.ref ? inst_data(var.ref.inst).var_idx : -1;
                if (mat_col != -1) {
                    es.add_coeff(mat_row, mat_col, weight);
                } else {
                    es.add_rhs(mat_row, -var.pos * weight);
                }
                // TODO: pin offset within cell
            };
            auto process_arc = [&](PortOrPin a, PortOrPin b) {
                if (a.ref == b.ref)
                    return;
                double weight = 1.0 / ((index_t(net->inst_ports.size()) + index_t(net->ext_pins.size()) - 1) *
                                       std::max<double>(1, std::abs(a.pos - b.pos)));
                stamp_equation(a, a, weight);
                stamp_equation(a, b, -weight);
                stamp_equation(b, b, weight);
                stamp_equation(b, a, -weight);
            };
            for (auto pin : net->ext_pins) {
                process_arc({{}, pin_pos(pin)}, lbport);
                process_arc({{}, pin_pos(pin)}, ubport);
            }
            for (auto port : net->inst_ports) {
                double pos = solver_pos(port.inst);
                process_arc({port, pos}, lbport);
                process_arc({port, pos}, ubport);
            }
        }
        // Add arcs from legalised position for subsequent iterations
        if (iter != -1) {
            for (index_t mat_row = 0; mat_row < index_t(var2idx.size()); mat_row++) {
                auto inst = store_index<CellInst>(var2idx.at(mat_row));
                double l_pos = legaliser_pos(inst);
                double c_pos = solver_pos(inst);
                double weight = (alpha * iter) / std::max<double>(1, std::abs(l_pos - c_pos));
                es.add_coeff(mat_row, mat_row, weight);
                es.add_rhs(mat_row, weight * l_pos);
            }
        }
    }

    const double solverTolerance = 1e-5;

    void solve_equations(EquationSystem &es, bool yaxis)
    {
        // Return the x or y position of a cell, depending on ydir
        auto solver_pos = [&](store_index<CellInst> inst) {
            return yaxis ? inst_data(inst).solver_y : inst_data(inst).solver_x;
        };
        std::vector<double> vals;
        std::transform(var2idx.begin(), var2idx.end(), std::back_inserter(vals),
                       [&](int idx) { return solver_pos(store_index<CellInst>(idx)); });
        es.solve(vals, solverTolerance);
        for (int i = 0; i < int(var2idx.size()); i++) {
            auto &data = insts.at(i);
            (yaxis ? data.solver_y : data.solver_x) = vals.at(i);
        }
    }

    void build_solve_direction(bool yaxis, int iter)
    {
        // Back-annotate positions
        for (int idx : var2idx) {
            auto &data = insts.at(idx);
            if (yaxis)
                data.solver_y = data.row;
            else
                data.solver_x = data.col;
        }
        // Inner iterative solver loop
        // (because we linearise the HPWL problem using bound2bound and net weights)
        for (int i = 0; i < 5; i++) {
            EquationSystem es(var2idx.size(), var2idx.size());
            build_equations(es, yaxis, iter);
            solve_equations(es, yaxis);
        }
        // Snap positions to grid
        for (int idx : var2idx) {
            auto &data = insts.at(idx);
            if (yaxis) {
                data.row = std::max<int>(0, std::min<int>(rows - 1, int(data.solver_y + 0.5)));
            } else {
                data.col =
                        std::max<int>(data.flip_x ? (data.width - 1) : 0,
                                      std::min<int>(cols - (data.flip_x ? 1 : data.width), int(data.solver_x + 0.5)));
            }
        }
    }

    void do_solve(int iter)
    {
        build_solve_direction(false /* x */, iter);
        build_solve_direction(true /* y */, iter);
    }

    void init()
    {
        ModuleAccess acc(netlist, mod);
        grid = mod.grid;
        auto size = acc.getType().size;
        rows = size.height / grid.height;
        cols = size.width / grid.width;
        setup_cells();
        log_info("Starting placement of %d cells.\n", int(var2idx.size()));
        log_info("   %d rows; %d cols\n", rows, cols);
    }

    dist_t net_hpwl(const Net &net)
    {
        Box net_box = Box();
        int pin_count = 0;
        for (auto &pin : net.ext_pins) {
            net_box.extend(pin);
            ++pin_count;
        }
        for (auto &port : net.inst_ports) {
            auto &data = inst_data(port.inst);
            net_box.extend(Point(data.col * grid.width, data.row * grid.height));
            ++pin_count;
        }
        return (pin_count < 2) ? 0 : ((net_box.x1 - net_box.x0) + (net_box.y1 - net_box.y0));
    }

    int64_t hpwl()
    {
        int64_t total_hpwl = 0;
        for (auto &net : mod.nets)
            total_hpwl += int64_t(net_hpwl(*net));
        return total_hpwl;
    }

    void run()
    {
        init(); // Initial place
        log_info("Starting random place HPWL: %.2fum\n", double(hpwl()) / ctx.unit_per_um);
        do_solve(-1);
        log_info("Initial solved HPWL: %.2fum\n", double(hpwl()) / ctx.unit_per_um);
    }
};
} // namespace

void global_placer(Context &ctx, Module &mod)
{
    GlobalPlacer placer(ctx, mod);
    placer.run();
}

NPNR_NAMESPACE_END
