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

    // For fast incremental HPWL updates
    struct HPWLBox
    {
        Box box;
        // Number of pins on each extremity
        index_t nx0 = 0, ny0 = 0, nx1 = 0, ny1 = 0;
        void add_pin(Point p)
        {
            if (p.x == box.x0)
                ++nx0;
            else if (p.x < box.x0) {
                box.x0 = p.x;
                nx0 = 1;
            }
            if (p.y == box.y0)
                ++ny0;
            else if (p.y < box.y0) {
                box.y0 = p.y;
                ny0 = 1;
            }
            if (p.x == box.x1)
                ++nx1;
            else if (p.x > box.x1) {
                box.x1 = p.x;
                nx1 = 1;
            }
            if (p.y == box.y1)
                ++ny1;
            else if (p.y > box.y1) {
                box.y1 = p.y;
                ny1 = 1;
            }
        }
    };

    struct IncrementalMoveChange
    {
        enum BoundChangeType
        {
            NO_CHANGE,
            CELL_MOVED_INWARDS,
            CELL_MOVED_OUTWARDS,
            FULL_RECOMPUTE
        };
        struct Axis
        {
            std::vector<store_index<Net>> bounds_changed_nets;
            std::vector<BoundChangeType> already_bounds_changed;
        };
        std::vector<HPWLBox> new_net_bounds;
        std::array<Axis, 2> axes;
    };

    bool ignore_net(store_index<Net> net)
    {
        return false; // TODO
    }

    void update_move_change(IncrementalMoveChange &mc, index_t inst, CellPlacement old_plc)
    {
        CellInstAccess access(netlist, mod, mod.insts[store_index<CellInst>(inst)]);
        for (auto [port_data, port_inst] : access.ports()) {
            auto net = port_inst.net;
            if (!net || ignore_net(net))
                continue;
            HPWLBox &curr_bounds = mc.new_net_bounds.at(net.idx());
            Point old_loc = old_plc.loc + port_data.offset.transform(old_plc.orient);
            Point new_loc = port_loc(PortRef{store_index<CellInst>(inst), port_data.index});
            // For X and Y axes
            for (bool y : {false, true}) {
                auto &axis = mc.axes.at(y ? 1 : 0);
                dist_t old = y ? old_loc.y : old_loc.x;
                dist_t curr = y ? new_loc.y : new_loc.x;
                dist_t &b0 = (y ? curr_bounds.box.y0 : curr_bounds.box.x0),
                       &b1 = (y ? curr_bounds.box.y1 : curr_bounds.box.x1);
                index_t &n0 = (y ? curr_bounds.ny0 : curr_bounds.nx0), &n1 = (y ? curr_bounds.ny1 : curr_bounds.nx1);
                auto &change = axis.already_bounds_changed.at(net.idx());
                // x0/y0 bound
                if (curr < b0) {
                    // Further out than bound
                    b0 = curr;
                    n0 = 1;
                    if (change == IncrementalMoveChange::NO_CHANGE) {
                        change = IncrementalMoveChange::CELL_MOVED_OUTWARDS;
                        axis.bounds_changed_nets.push_back(net);
                    }
                } else if (curr == b0 && old > b0) {
                    // Moved onto bound
                    ++n0;
                    if (change == IncrementalMoveChange::NO_CHANGE) {
                        change = IncrementalMoveChange::CELL_MOVED_OUTWARDS;
                        axis.bounds_changed_nets.push_back(net);
                    }
                } else if (old == b0 && curr > b0) {
                    // Moved inside of bound
                    if (change == IncrementalMoveChange::NO_CHANGE)
                        axis.bounds_changed_nets.push_back(net);
                    if (n0 == 1) {
                        // Bound has now changed
                        change = IncrementalMoveChange::FULL_RECOMPUTE;
                    } else {
                        // Bound has fewer pins on it
                        --n0;
                        if (change == IncrementalMoveChange::NO_CHANGE)
                            change = IncrementalMoveChange::CELL_MOVED_INWARDS;
                    }
                }
                // x1/y1 bound
                if (curr > b1) {
                    // Further out than bound
                    b1 = curr;
                    n1 = 1;
                    if (change == IncrementalMoveChange::NO_CHANGE) {
                        change = IncrementalMoveChange::CELL_MOVED_OUTWARDS;
                        axis.bounds_changed_nets.push_back(net);
                    }
                } else if (curr == b1 && old < b1) {
                    // Moved onto bound
                    ++n1;
                    if (change == IncrementalMoveChange::NO_CHANGE) {
                        change = IncrementalMoveChange::CELL_MOVED_OUTWARDS;
                        axis.bounds_changed_nets.push_back(net);
                    }
                } else if (old == b1 && curr < b1) {
                    // Moved inside of bound
                    if (change == IncrementalMoveChange::NO_CHANGE)
                        axis.bounds_changed_nets.push_back(net);
                    if (n1 == 1) {
                        // Bound has now changed
                        change = IncrementalMoveChange::FULL_RECOMPUTE;
                    } else {
                        // Bound has fewer pins on it
                        --n1;
                        if (change == IncrementalMoveChange::NO_CHANGE)
                            change = IncrementalMoveChange::CELL_MOVED_INWARDS;
                    }
                }
            }
        }
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
