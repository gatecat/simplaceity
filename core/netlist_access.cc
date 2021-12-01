#include "netlist_access.h"

NPNR_NAMESPACE_BEGIN

const CellType &CellInstAccess::getType() { return netlist.cell_types[inst.type]; }
store_index<CellPort> CellInstAccess::port_idx(IdString name) { return getType().ports[name].index; }

Net *CellInstAccess::getPort(IdString name) { return getPort(port_idx(name)); }
void CellInstAccess::connectPort(IdString name, store_index<Net> net) { return connectPort(port_idx(name), net); }
void CellInstAccess::disconnectPort(IdString name) { return disconnectPort(port_idx(name)); }

Net *CellInstAccess::getPort(store_index<CellPort> index)
{
    auto n = inst.ports[index.idx()].net;
    if (n)
        return &mod.nets[n];
    else
        return nullptr;
}
void CellInstAccess::connectPort(store_index<CellPort> index, store_index<Net> net)
{
    auto &p_type = getType().ports[index];
    auto &p = inst.ports[index.idx()];
    NPNR_ASSERT(!p.net);
    p.net = net;
    if (p.net) {
        auto &n = mod.nets[p.net];
        n.inst_ports.emplace(inst.index, index);
        if (p_type.dir != PortDir::IN) {
            if (n.driver_count == 0) {
                // Singly driven
                n.driver_count = 1;
                n.driver.inst = inst.index;
                n.driver.port = index;
            } else {
                // Multiply driven
                ++n.driver_count;
                n.driver = PortRef{};
            }
        }
    }
}
void CellInstAccess::disconnectPort(store_index<CellPort> index)
{
    auto &p_type = getType().ports[index];
    auto &p = inst.ports[index.idx()];
    if (p.net) {
        auto &n = mod.nets[p.net];
        n.inst_ports.erase(PortRef{inst.index, index});
        if (p_type.dir != PortDir::IN) {
            --n.driver_count;
            if (n.driver_count == 1) {
                // Case where a net has gone from multiply to singly driven
                for (auto net_port : n.inst_ports) {
                    if (netlist.cell_types[mod.insts[net_port.inst].type].ports[net_port.port].dir != PortDir::IN) {
                        n.driver = net_port;
                        break;
                    }
                }
            }
        }
        p.net = store_index<Net>();
    }
}

CellInstAccess ModuleAccess::addInst(IdString name, IdString type)
{
    auto &cell_type = netlist.cell_types[type];
    CellInst &inst = *mod.insts.add(name, cell_type);
    return CellInstAccess(netlist, mod, inst);
}
CellInstAccess ModuleAccess::getInst(IdString name) { return CellInstAccess(netlist, mod, mod.insts[name]); }
Net &ModuleAccess::addNet(IdString name) { return *mod.nets.add_or_get(name); }
Net *ModuleAccess::getNet(IdString name)
{
    if (mod.nets.count(name))
        return &mod.nets[name];
    else
        return nullptr;
}
CellType &ModuleAccess::getType() { return netlist.cell_types[mod.type]; }

ModuleAccess NetlistAccess::createModule(IdString type)
{
    auto &cell_type = netlist.cell_types[type];
    auto &cell = *netlist.modules.add_or_get(type, cell_type.index);
    cell.port_nets.reserve(cell_type.ports.size());
    return ModuleAccess(netlist, cell);
}

NPNR_NAMESPACE_END
