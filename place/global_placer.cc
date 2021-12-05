#include "global_placer.h"
#include "eqn_system.h"
#include "log.h"
#include "netlist.h"
#include "netlist_access.h"

#include <queue>

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
            return (row % 2) ? (flip_x ? Orientation::FN : Orientation::S)
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

    // Solver
    const float alpha = 0.02;

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

        for (auto &net : mod.nets) {
            if (net->type == NetType::POWER || net->type == NetType::GROUND || net->type == NetType::CLOCK)
                continue;
            if ((index_t(net->inst_ports.size()) + index_t(net->ext_pins.size())) < 2)
                continue;
            // Find the bounds of the net in this axis, and the ports that correspond to these bounds
            PortOrPin lbport{{}, std::numeric_limits<double>::max()}, ubport{{}, std::numeric_limits<double>::lowest()};
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
            auto &data = insts.at(var2idx.at(i));
            (yaxis ? data.solver_y : data.solver_x) = vals.at(i);
        }
    }

    void build_solve_direction(bool yaxis, int iter, int inner_iters = 5)
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
        for (int i = 0; i < inner_iters; i++) {
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

    void do_solve(int iter, int inner_iters = 5)
    {
        build_solve_direction(false /* x */, iter, inner_iters);
        build_solve_direction(true /* y */, iter, inner_iters);
    }

    // Spreader
    int bin_width, bin_height;
    int bin_cols, bin_rows;
    array2d<double> avail_area, used_area, fixed_area;
    array2d<pool<index_t>> placed_instances;

    int inst_bin_x(const InstData &inst) { return std::max(0, std::min(bin_cols - 1, inst.col / bin_width)); }
    int inst_bin_y(const InstData &inst) { return std::max(0, std::min(bin_rows - 1, inst.row / bin_height)); }

    void setup_spread()
    {
        // Physically square bins
        bin_height = std::max<int>(1, (grid.width / grid.height));
        bin_width = std::max<int>(1, (grid.height / grid.width));
        bin_rows = (rows + (bin_height - 1)) / bin_height;
        bin_cols = (cols + (bin_width - 1)) / bin_width;
        avail_area.reset(bin_cols, bin_rows, 0);
        used_area.reset(bin_cols, bin_rows, 0);
        placed_instances.reset(bin_cols, bin_rows, pool<int>{});
        // Bin capacity
        for (int r = 0; r < rows; r++)
            for (int c = 0; c < cols; c++)
                avail_area.at(c / bin_width, r / bin_height) += 1.0;
        // Initial to bin
        for (auto &inst : mod.insts) {
            auto &data = insts.at(inst->index.idx());
            int bin_x = inst_bin_x(data);
            int bin_y = inst_bin_y(data);
            if (data.var_idx != -1) {
                // Only include cells we can actually move
                placed_instances.at(bin_x, bin_y).insert(inst->index.idx());
                used_area.at(bin_x, bin_y) += (data.width * data.height);
            } else {
                // TODO: large macros that cross bin boundaries
                avail_area.at(bin_x, bin_y) = std::max(0.0, avail_area.at(bin_x, bin_y) - (data.width * data.height));
            }
        }
    }

    const double beta = 0.8;

    void find_overfull_bins(pool<std::pair<int, int>> &result)
    {
        result.clear();
        for (auto used : used_area) {
            double avail = avail_area.at(used.x, used.y);
            if (used.value >= beta * avail)
                result.emplace(used.x, used.y);
        }
    }

    struct ExpandedBin
    {
        int x0, y0, x1, y1;
        double used, avail;
    };

    ExpandedBin expand_bin(int x, int y)
    {
        ExpandedBin result;
        result.x0 = result.x1 = x;
        result.y0 = result.y1 = y;
        result.used = used_area.at(x, y);
        result.avail = avail_area.at(x, y);
        bool did_something = true;
        auto is_done = [&]() { return result.used < beta * result.avail; };
        auto expand = [&](int x, int y) {
            result.used += used_area.at(x, y);
            result.avail += avail_area.at(x, y);
        };
        while (did_something && !is_done()) {
            did_something = false;
            // x+ direction
            if (result.x1 < (bin_cols - 1)) {
                ++result.x1;
                for (int y = result.y0; y <= result.y1; y++)
                    expand(result.x1, y);
                did_something = true;
                if (is_done())
                    break;
            }
            // y+ direction
            if (result.y1 < (bin_rows - 1)) {
                ++result.y1;
                for (int x = result.x0; x <= result.x1; x++)
                    expand(x, result.y1);
                did_something = true;
                if (is_done())
                    break;
            }
            // x- direction
            if (result.x0 > 0) {
                --result.x0;
                for (int y = result.y0; y <= result.y1; y++)
                    expand(result.x0, y);
                did_something = true;
                if (is_done())
                    break;
            }
            // y- direction
            if (result.y0 > 0) {
                --result.y0;
                for (int x = result.x0; x <= result.x1; x++)
                    expand(x, result.y0);
                did_something = true;
                if (is_done())
                    break;
            }
        }
        return result;
    }

    struct CutJob
    {
        int x0, y0, x1, y1;
        bool yaxis = false;
    };

    void spread_update_inst(int inst)
    {
        auto &data = insts.at(inst);
        // Compute new clamped position
        int new_row = std::max<int>(0, std::min<int>(rows - 1, int(data.solver_y + 0.5)));
        int new_col = std::max<int>(data.flip_x ? (data.width - 1) : 0,
                                    std::min<int>(cols - (data.flip_x ? 1 : data.width), int(data.solver_x + 0.5)));
        // Incrementally update bin structures
        int old_bin_x = inst_bin_x(data), old_bin_y = inst_bin_y(data);
        used_area.at(old_bin_x, old_bin_y) -= (data.height * data.width);
        data.col = new_col;
        data.row = new_row;
        placed_instances.at(old_bin_x, old_bin_y).erase(inst);
        int new_bin_x = inst_bin_x(data), new_bin_y = inst_bin_y(data);
        used_area.at(new_bin_x, new_bin_y) += (data.height * data.width);
        placed_instances.at(new_bin_x, new_bin_y).insert(inst);
        // if (old_bin_x != new_bin_x || old_bin_y != new_bin_y)
        //    log("%d moved (%d, %d) -> (%d, %d)\n", inst, old_bin_x, old_bin_y, new_bin_x, new_bin_y);
    }

    void cut_region(CutJob job, std::queue<CutJob> &out)
    {
        std::vector<int> job_insts;
        int job_start = job.yaxis ? job.y0 : job.x0;
        int job_end = job.yaxis ? job.y1 : job.x1;
        int job_length = (job_end - job_start) + 1;
        // log("    cutting (%d, %d) (%d, %d) %c\n", job.x0, job.y0, job.x1, job.y1, job.yaxis ? 'y' : 'x');
        if (job.y0 == job.y1 && job.yaxis) {
            if (job.x0 == job.x1)
                return; // nothing to do
            job.yaxis = false;
            out.push(job);
            return;
        }
        if (job.x0 == job.x1 && !job.yaxis) {
            if (job.y0 == job.y1)
                return; // nothing to do
            job.yaxis = true;
            out.push(job);
            return;
        }
        double total_avail = 0, total_used = 0;
        std::vector<double> slither_area;
        slither_area.resize(job_length);
        for (int y = job.y0; y <= job.y1; y++) {
            for (int x = job.x0; x <= job.x1; x++) {
                for (auto inst : placed_instances.at(x, y))
                    job_insts.push_back(inst);
                total_avail += avail_area.at(x, y);
                total_used += used_area.at(x, y);
                slither_area.at((job.yaxis ? y : x) - job_start) += avail_area.at(x, y);
            }
        }
        // log("        %d insts\n", int(job_insts.size()));
        if (job_insts.size() <= 1) {
            return;
        }
        std::stable_sort(job_insts.begin(), job_insts.end(), [&](int a, int b) {
            return job.yaxis ? (insts.at(a).solver_y < insts.at(b).solver_y)
                             : (insts.at(a).solver_x < insts.at(b).solver_x);
        });
        // Find midpoint by available area
        double acc_avail = 0;
        int area_midpoint = job_start;
        // Make sure we always leave at least one slice on the right side of the cut; hence '<' not '<='
        for (; area_midpoint < (job_end - 1); area_midpoint++) {
            acc_avail += slither_area.at(area_midpoint - job_start);
            if (acc_avail >= (total_avail / 2))
                break;
        }
        // Left and right available areas
        double left_avail = acc_avail;
        double right_avail = total_avail - acc_avail;
        // Find cut of cells area that minimises utilisation difference
        double best_deltaU = std::numeric_limits<double>::max();
        int best_pivot = -1;
        double left_used = total_used;
        double right_used = 0;
        for (int pivot = 0; pivot < int(job_insts.size()); pivot++) {
            double deltaU =
                    std::abs((left_used / std::max(1.0, left_avail)) - (right_used / std::max(1.0, right_avail)));
            if (deltaU < best_deltaU) {
                best_deltaU = deltaU;
                best_pivot = pivot;
            }
            auto &pivot_data = insts.at(job_insts.at(pivot));
            int pivot_area = pivot_data.width * pivot_data.height;
            left_used -= pivot_area;
            right_used += pivot_area;
        }
        // Split regions into bins, and then spread insts by linear interpolation within those bins
        auto spread_binlerp = [&](int insts_start, int insts_end, double area_l, double area_r) {
            // Scale to original grid coordinates
            area_l *= (job.yaxis ? bin_height : bin_width);
            area_r *= (job.yaxis ? bin_height : bin_width);

            int N = insts_end - insts_start;
            auto pos_ref = [&](int i) -> double & {
                return job.yaxis ? std::ref(insts.at(job_insts.at(i)).solver_y)
                                 : std::ref(insts.at(job_insts.at(i)).solver_x);
            };
            if (N <= 2) {
                for (int i = insts_start; i < insts_end; i++) {
                    pos_ref(i) = area_l + i * ((area_r - area_l) / N);
                }
                return;
            }
            // Split region into up to 10 (K) bins
            int K = std::min<int>(N, 10);
            std::vector<std::pair<int, double>> bin_bounds; // [(inst start, area start)]
            bin_bounds.emplace_back(insts_start, area_l);
            for (int i = 1; i < K; i++)
                bin_bounds.emplace_back(insts_start + (N * i) / K, area_l + ((area_r - area_l + 0.99) * i) / K);
            bin_bounds.emplace_back(insts_end, area_r + 0.99);
            for (int i = 0; i < K; i++) {
                auto &bl = bin_bounds.at(i), br = bin_bounds.at(i + 1);
                double orig_left = pos_ref(bl.first);
                double orig_right = pos_ref(br.first - 1);
                double m = (br.second - bl.second) / std::max(0.00001, orig_right - orig_left);
                for (int j = bl.first; j < br.first; j++) {
                    auto &pos = pos_ref(j);
                    NPNR_ASSERT(pos >= orig_left && pos <= orig_right);
                    pos = bl.second + m * (pos - orig_left);
                }
            }
        };
        // log("        inst cut %c : (0, %d, %d) area cut: (%d, %d, %d) dU %f\n", job.yaxis ? 'y' : 'x', best_pivot,
        //    int(job_insts.size()), job_start, area_midpoint, job_end, best_deltaU);
        spread_binlerp(0, best_pivot, job_start, area_midpoint);
        spread_binlerp(best_pivot, int(job_insts.size()), area_midpoint + 1, job_end);
        // Update inst positions
        for (int inst : job_insts)
            spread_update_inst(inst);
        if (job.yaxis) {
            CutJob top_job{.x0 = job.x0, .y0 = job.y0, .x1 = job.x1, .y1 = area_midpoint, .yaxis = false};
            CutJob bottom_job{.x0 = job.x0, .y0 = area_midpoint + 1, .x1 = job.x1, .y1 = job.y1, .yaxis = false};
            out.push(top_job);
            out.push(bottom_job);
        } else {
            CutJob left_job{.x0 = job.x0, .y0 = job.y0, .x1 = area_midpoint, .y1 = job.y1, .yaxis = true};
            CutJob right_job{.x0 = area_midpoint + 1, .y0 = job.y0, .x1 = job.x1, .y1 = job.y1, .yaxis = true};
            out.push(left_job);
            out.push(right_job);
        }
    }

    void do_expand(ExpandedBin exp)
    {
        // log(" spreading instances in ((%d, %d), (%d, %d))\n", exp.x0, exp.y0, exp.x1, exp.y1);
        std::queue<CutJob> q;
        q.push(CutJob{.x0 = exp.x0, .y0 = exp.y0, .x1 = exp.x1, .y1 = exp.y1});
        while (!q.empty()) {
            auto next = q.front();
            q.pop();
            cut_region(next, q);
        }
    }

    std::vector<ExpandedBin> merge_regions(const std::vector<ExpandedBin> &expanded)
    {
        std::vector<ExpandedBin> result;
        array2d<int> loc2idx(bin_cols, bin_rows, -1);
        pool<int> erased;
        for (auto exp : expanded) {
            pool<int> merge_with;
            int group = int(result.size());
            result.push_back(exp);
            for (int y = exp.y0; y <= exp.y1; y++)
                for (int x = exp.x0; x <= exp.x1; x++) {
                    if (loc2idx.at(x, y) == -1)
                        loc2idx.at(x, y) = group;
                    else
                        merge_with.insert(loc2idx.at(x, y));
                }
            if (!merge_with.empty()) {
                // We found other groups; merge with these
                int base = (*merge_with.begin());
                auto &base_entry = result.at(base);
                bool bounds_changed = false;
                auto do_merge = [&](int group) {
                    if (group == base)
                        return;
                    auto entry = result.at(group);
                    if (entry.x0 < base_entry.x0) {
                        base_entry.x0 = entry.x0;
                        bounds_changed = true;
                    }
                    if (entry.y0 < base_entry.y0) {
                        base_entry.y0 = entry.y0;
                        bounds_changed = true;
                    }
                    if (entry.x1 > base_entry.x1) {
                        base_entry.x1 = entry.x1;
                        bounds_changed = true;
                    }
                    if (entry.y1 > base_entry.y1) {
                        base_entry.y1 = entry.y1;
                        bounds_changed = true;
                    }
                    if (group != int(result.size()) - 1)
                        erased.insert(group);
                };
                for (auto m : merge_with)
                    do_merge(m);
                do_merge(group);
                // Remove temporarily added
                result.pop_back();
                if (bounds_changed) {
                    for (int y = base_entry.y0; y <= base_entry.y1; y++)
                        for (int x = base_entry.x0; x <= base_entry.x1; x++)
                            loc2idx.at(x, y) = base;
                }
            }
        }
        // Update these as we weren't keeping track
        std::vector<ExpandedBin> final_result;
        for (int i = 0; i < int(result.size()); i++) {
            if (erased.count(i))
                continue;
            final_result.push_back(result.at(i));
            auto &r = final_result.back();
            r.avail = 0;
            r.used = 0;
            for (int y = r.y0; y <= r.y1; y++)
                for (int x = r.x0; x <= r.x1; x++) {
                    r.avail += avail_area.at(x, y);
                    r.used += used_area.at(x, y);
                }
        }
        return final_result;
    }

    void do_spread()
    {
        setup_spread();
        pool<std::pair<int, int>> overfull;
        find_overfull_bins(overfull);
        std::vector<ExpandedBin> expanded;
        for (auto bin : overfull)
            expanded.push_back(expand_bin(bin.first, bin.second));
        expanded = merge_regions(expanded);
        for (auto exp : expanded)
            do_expand(exp);
    }

    // Strict legaliser
    std::vector<std::vector<bool>> occupied;
    bool can_legalise_to(const InstData &inst, int row, int col)
    {
        // TODO: multi height instances
        NPNR_ASSERT(inst.height == 1);
        if (row < 0 || row >= rows)
            return false;
        int col0 = inst.flip_x ? (col - (inst.width - 1)) : col;
        int col1 = inst.flip_x ? col : (col + (inst.width - 1));
        for (int x = col0; x <= col1; x++) {
            if (x < 0 || x >= cols)
                return false;
            if (occupied.at(row).at(col))
                return false;
        }
        return true;
    }
    void mark_occupied(const InstData &inst)
    {
        // TODO: multi height instances
        NPNR_ASSERT(inst.height == 1);
        int col0 = inst.flip_x ? (inst.col - (inst.width - 1)) : inst.col;
        int col1 = inst.flip_x ? inst.col : (inst.col + (inst.width - 1));
        for (int x = col0; x <= col1; x++)
            occupied.at(inst.row).at(x) = true;
    }
    void legalise_cell(int inst)
    {
        const CellInst &cell_inst = mod.insts[store_index<CellInst>(inst)];
        InstData &data = insts.at(inst);
        // Already legal
        if (can_legalise_to(data, data.row, data.col)) {
            mark_occupied(data);
            return;
        }
        // Search around cell's location
        int radius = 1;
        int iters = 0;
        int iters_at_radius = 0;
        const int max_iters = 1000000;
        bool done = false;
        while (!done && iters++ < max_iters) {
            ++iters_at_radius;
            if (iters_at_radius > (30 * radius)) {
                radius = std::min(std::max(rows / 2, cols / 2), ((radius * 3) + 1) / 2);
                iters_at_radius = 0;
            }
            int cx, cy;
            do {
                cx = (data.col - radius) + ctx.rng.rng(2 * radius + 1);
                cy = (data.row - radius) + ctx.rng.rng(2 * radius + 1);
            } while (cx < 0 || cx >= cols || cy < 0 || cy >= rows);
            if (can_legalise_to(data, cy, cx)) {
                // Found a free location
                //    TODO: take wirelength into account rather than just finding first free
                data.row = cy;
                data.col = cx;
                mark_occupied(data);
                done = true;
            }
        }
        if (!done)
            log_error("Failed to legalise instance '%s'\n", cell_inst.name.c_str(&ctx));
    }

    void do_legalise()
    {
        // Reset
        occupied.resize(rows);
        for (int y = 0; y < rows; y++) {
            occupied.at(y).resize(cols);
            std::fill(occupied.at(y).begin(), occupied.at(y).end(), false);
        }
        for (auto &inst : insts) {
            if (inst.fixed)
                mark_occupied(inst);
        }
        // Sort remaining cells by area; as bigger means harder to legalise so should be done first
        std::vector<std::pair<int, int>> to_legalise;
        for (int idx : var2idx) {
            to_legalise.emplace_back(idx, insts.at(idx).width * insts.at(idx).height);
        }
        std::stable_sort(to_legalise.begin(), to_legalise.end(), [](auto a, auto b) { return a.second > b.second; });
        for (auto entry : to_legalise) {
            legalise_cell(entry.first);
        }
    }

    void init()
    {
        ModuleAccess acc(netlist, mod);
        grid = mod.grid;
        auto size = acc.getType().size;
        rows = size.height / grid.height;
        cols = size.width / grid.width;
        setup_cells();
        log_info("Starting global placement of %d cells.\n", int(var2idx.size()));
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
        for (auto &net : mod.nets) {
            if (net->type == NetType::POWER || net->type == NetType::GROUND || net->type == NetType::CLOCK)
                continue;
            total_hpwl += int64_t(net_hpwl(*net));
        }
        return total_hpwl;
    }

    void do_iter(int i)
    {
        log_info("Iteration %d:\n", i);
        do_solve(i);
        log_info("    solved HPWL: %.fum\n", double(hpwl()) / ctx.unit_per_um);
        do_spread();
        log_info("    spread HPWL: %.fum\n", double(hpwl()) / ctx.unit_per_um);
        do_legalise();
        log_info("    legalised HPWL: %.fum\n", double(hpwl()) / ctx.unit_per_um);
    }

    void bind()
    {
        for (int idx : var2idx) {
            auto &data = insts.at(idx);
            auto &inst = mod.insts[store_index<CellInst>(idx)];
            inst.placement =
                    CellPlacement{.loc = Point(data.col * grid.width, row_pos(data.row)), .orient = data.get_orient()};
        }
    }

    void run()
    {
        init(); // Initial place
        log_info("Starting random place HPWL: %.fum\n", double(hpwl()) / ctx.unit_per_um);
        for (int i = 0; i < 10; i++) {
            do_solve(-1, 1);
            log_info("    initial solver iter %d HPWL: %.fum\n", i, double(hpwl()) / ctx.unit_per_um);
        }
        for (int i = 0; i < 10; i++) {
            do_iter(i);
        }
        bind();
    }
};
} // namespace

void global_placer(Context &ctx, Module &mod)
{
    GlobalPlacer placer(ctx, mod);
    placer.run();
}

NPNR_NAMESPACE_END
