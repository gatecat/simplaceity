#include "global_placer.h"
#include "netlist.h"
#include "netlist_access.h"
#include "eqn_system.h"

NPNR_NAMESPACE_BEGIN

namespace {
struct GlobalPlacer {
    GlobalPlacer(Context &ctx, Module &mod) : ctx(ctx), netlist(ctx.netlist()), mod(mod) {};
    Context &ctx;
    Netlist &netlist;
    Module &mod;

    Size grid;
    int rows;
    int cols;

    struct InstData {
        int row, col;
        index_t var_idx;
        double solver_x, solver_y;
        int width, height;
        bool fixed = false;
        bool flip_x = false;
        Orientation get_orient() const {
            // Odd rows are flipped in y
            return (row % 1) ? (flip_x ? Orientation::FN : Orientation::S) : (flip_x ? Orientation::FS : Orientation::N);
        }
    };

    std::vector<InstData> insts;
    std::vector<index_t> var2idx;

    dist_t row_pos(int row) {
        return ((row + 1) / 2) * (2 * grid.height);
    }

    void setup_cells() {
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

    InstData &inst_data(store_index<CellInst> inst) {
        return insts.at(inst.idx());
    }

    const float alpha = 0.1;

    void build_equations(EquationSystem &es, bool yaxis, int iter = -1) {
        // Return the x or y position of a cell, depending on ydir
        auto solver_pos = [&](store_index<CellInst> inst) { return yaxis ? inst_data(inst).solver_y : inst_data(inst).solver_x; };
        auto legaliser_pos = [&](store_index<CellInst> inst) { return yaxis ? inst_data(inst).row : inst_data(inst).col; };

        // Find the bounds of the net in this axis, and the ports that correspond to these bounds
        PortRef lbport{}, ubport{};
        double lbpos = std::numeric_limits<double>::max(), ubpos = std::numeric_limits<double>::lowest();

        for (auto &net : mod.nets) {
            if (net->inst_ports.empty())
                continue;
            // Find the bounding ports/pins of the net
            for (auto pin : net->ext_pins) {
                lbpos = std::min(lbpos, double(pin.x) / grid.width);
                ubpos = std::max(ubpos, double(pin.y) / grid.height);
            }
            for (auto port : net->inst_ports) {
                double pos = solver_pos(port.inst);
                if (pos < lbpos) {
                    lbpos = pos;
                    lbport = port;
                }
                if (pos > ubpos) {
                    ubpos = pos;
                    ubport = port;
                }
            }
        }

        es.reset();
    }

    void init() {
        ModuleAccess acc(netlist, mod);
        grid = mod.grid;
        auto size = acc.getType().size;
        rows = size.height / grid.height;
        cols = size.width / grid.width;
        setup_cells();
    }

    void run() {

    }
};
}

void global_placer(Context &ctx, Module &mod) {
    GlobalPlacer placer(ctx, mod);
    placer.run();
}

NPNR_NAMESPACE_END
