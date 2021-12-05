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
        int64_t hpwl() const { return std::max((box.x1 - box.x0), 0) + std::max((box.y1 - box.y0), 0); }
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
        int64_t wirelen_delta = 0;
    };
    std::vector<HPWLBox> net_bounds;
    int64_t curr_wirelen = 0;

    bool ignore_net(store_index<Net> net)
    {
        return false; // TODO
    }

    HPWLBox get_net_box(const Net &net)
    {
        HPWLBox result{};
        for (auto p : net.ext_pins)
            result.add_pin(p);
        for (auto p : net.inst_ports)
            result.add_pin(port_loc(p));
        return result;
    }

    CellPlacement get_placement(const InstData &inst)
    {
        return CellPlacement{.loc = Point(inst.col * grid.width, row_pos(inst.row)), .orient = inst.get_orient()};
    }

    void init_move_change(IncrementalMoveChange &mc)
    {
        mc.new_net_bounds = net_bounds;
        mc.wirelen_delta = 0;
        for (int i = 0; i < 2; i++) {
            mc.axes.at(i).already_bounds_changed.resize(net_bounds.size(), IncrementalMoveChange::NO_CHANGE);
        }
    }

    void reset_move_change(IncrementalMoveChange &mc)
    {
        for (int i = 0; i < 2; i++) {
            auto &axis = mc.axes.at(i);
            for (auto bc : axis.bounds_changed_nets) {
                axis.already_bounds_changed.at(bc.idx()) = IncrementalMoveChange::NO_CHANGE;
                mc.new_net_bounds.at(bc.idx()) = net_bounds.at(bc.idx());
            }
            axis.bounds_changed_nets.clear();
        }
        mc.wirelen_delta = 0;
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

    void compute_cost_changes(IncrementalMoveChange &mc)
    {
        for (int axis = 0; axis < 2; axis++) {
            for (auto net : mc.axes.at(axis).bounds_changed_nets)
                if (mc.axes.at(axis).already_bounds_changed.at(net.idx()) == IncrementalMoveChange::FULL_RECOMPUTE &&
                    (axis == 0 || mc.axes.at(axis - 1).already_bounds_changed.at(net.idx()) !=
                                          IncrementalMoveChange::FULL_RECOMPUTE))
                    mc.new_net_bounds.at(net.idx()) = get_net_box(mod.nets[net]);
        }
        mc.wirelen_delta = 0;
        for (auto net : mc.axes.at(0).bounds_changed_nets)
            mc.wirelen_delta += mc.new_net_bounds.at(net.idx()).hpwl() - net_bounds.at(net.idx()).hpwl();
        for (auto net : mc.axes.at(1).bounds_changed_nets)
            if (mc.axes.at(0).already_bounds_changed.at(net.idx()) == IncrementalMoveChange::NO_CHANGE)
                mc.wirelen_delta += mc.new_net_bounds.at(net.idx()).hpwl() - net_bounds.at(net.idx()).hpwl();
    }

    void commit_cost_changes(IncrementalMoveChange &mc)
    {
        for (int i = 0; i < 2; i++) {
            for (auto net : mc.axes.at(i).bounds_changed_nets)
                net_bounds.at(net.idx()) = mc.new_net_bounds.at(net.idx());
        }
        curr_wirelen += mc.wirelen_delta;
    }

    IncrementalMoveChange move_change;

    void init()
    {
        // Init insts
        insts.resize(mod.insts.size());
        for (auto &inst : mod.insts) {
            auto &data = insts.at(inst->index.idx());
            auto &type = netlist.cell_types[inst->type];
            NPNR_ASSERT((type.size.width % grid.width) == 0);
            data.width = type.size.width / grid.width;
            NPNR_ASSERT((type.size.height % grid.height) == 0);
            data.height = type.size.height / grid.height;
            data.fixed = inst->fixed;
            const CellPlacement &plc = inst->placement.value();
            if (!data.fixed) {
                // Assert on grid
                NPNR_ASSERT((plc.loc.x % grid.width) == 0);
                NPNR_ASSERT((plc.loc.y % (2 * grid.height)) == 0);
            }
            data.col = plc.loc.x / grid.width;
            data.row = (inst->placement->loc.y / (2 * grid.height)) * 2;
            if (inst->placement->orient == Orientation::FN || inst->placement->orient == Orientation::S)
                data.row -= 1;
            if (inst->placement->orient == Orientation::FN || inst->placement->orient == Orientation::FS)
                data.flip_x = true;
            else
                data.flip_x = false;
        }
        // Init nets
        net_bounds.resize(mod.nets.size());
        curr_wirelen = 0;
        for (auto &net : mod.nets) {
            net_bounds.at(net->index.idx()) = get_net_box(*net);
            curr_wirelen += net_bounds.at(net->index.idx()).hpwl();
        }
        init_move_change(move_change);
    }

    void run() { init(); }
};
} // namespace

void detail_placer(Context &ctx, Module &mod)
{
    DetailPlacer placer(ctx, mod);
    placer.run();
}

NPNR_NAMESPACE_END
