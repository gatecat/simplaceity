#include "detail_placer.h"
#include "log.h"
#include "netlist.h"
#include "netlist_access.h"

#include <queue>

NPNR_NAMESPACE_BEGIN

namespace {
struct DetailPlacer
{
    DetailPlacer(Context &ctx, Module &mod) : ctx(ctx), netlist(ctx.netlist()), mod(mod){};
    Context &ctx;
    Netlist &netlist;
    Module &mod;

    Size grid;
    int rows;
    int cols;

    struct InstData
    {
        int row, col;
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
    std::vector<index_t> moveable;

    dist_t row_pos(int row) { return ((row + 1) / 2) * (2 * grid.height); }

    Point cell_loc(const InstData &data) { return Point(data.col * grid.width, row_pos(data.row)); }
    Point port_loc(PortRef port)
    {
        auto &data = insts.at(port.inst.idx());
        auto &port_data = netlist.cell_types[mod.insts[port.inst].type].ports[port.port];
        Point origin = cell_loc(data);
        return origin + port_data.offset.transform(data.get_orient());
    }

    std::pair<int, int> cell_hbounds(const InstData &data, int base_col)
    {
        int x0 = data.col - (data.flip_x ? (data.width - 1) : 0);
        int x1 = data.col + (data.flip_x ? 0 : (data.width + 1));
        return {x0, x1};
    }

    array2d<index_t> loc2inst;
    void stamp_inst(index_t inst)
    {
        auto &data = insts.at(inst);
        auto [x0, x1] = cell_hbounds(data, data.col);
        for (int x = x0; x <= x1; x++) {
            NPNR_ASSERT(loc2inst.at(x, data.row) == -1);
            loc2inst.at(x, data.row) = inst;
        }
    }
    void unstamp_inst(index_t inst)
    {
        auto &data = insts.at(inst);
        auto [x0, x1] = cell_hbounds(data, data.col);
        for (int x = x0; x <= x1; x++) {
            NPNR_ASSERT(loc2inst.at(x, data.row) == inst);
            loc2inst.at(x, data.row) = -1;
        }
    }
    bool check_placement(index_t inst, int new_row, int new_col, pool<index_t> *ripup = nullptr)
    {
        if (new_row < 0 || new_row >= rows)
            return false;
        auto &data = insts.at(inst);
        auto [x0, x1] = cell_hbounds(data, new_col);
        if (x0 < 0 || x1 >= cols)
            return false;
        for (int x = x0; x <= x1; x++) {
            index_t placed = loc2inst.at(x, new_row);
            if (placed != -1) {
                if (ripup)
                    ripup->insert(placed);
                else
                    return false;
            }
        }
        return true;
    }

    void run() {}
};
} // namespace

void detail_placer(Context &ctx, Module &mod)
{
    DetailPlacer placer(ctx, mod);
    placer.run();
}

NPNR_NAMESPACE_END
