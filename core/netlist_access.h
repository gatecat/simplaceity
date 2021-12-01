#ifndef NETLIST_ACCESS_H
#define NETLIST_ACCESS_H

#include "netlist.h"
#include "preface.h"

NPNR_NAMESPACE_BEGIN

struct InstPortIterator
{
    using port_iter_t = indexed_store<std::unique_ptr<CellPort>>::const_iterator;
    port_iter_t base;
    CellInst &inst;
    InstPortIterator(port_iter_t base, CellInst &inst) : base(base), inst(inst){};
    bool operator==(const InstPortIterator &other) const { return base == other.base; }
    bool operator!=(const InstPortIterator &other) const { return base != other.base; }
    void operator++() { ++base; }
    std::pair<const CellPort &, PortInst &> operator*()
    {
        return std::make_pair(std::ref(**base), std::ref(inst.ports[(*base)->index.idx()]));
    }
};

struct InstPortRange
{
    const CellType &type;
    CellInst &inst;
    InstPortRange(const CellType &type, CellInst &inst) : type(type), inst(inst){};
    InstPortIterator begin() { return {type.ports.begin(), inst}; }
    InstPortIterator end() { return {type.ports.end(), inst}; }
};

struct CellInstAccess
{
    CellInstAccess(Netlist &netlist, Module &mod, CellInst &inst) : netlist(netlist), mod(mod), inst(inst){};
    Netlist &netlist;
    Module &mod;
    CellInst &inst;

    const CellType &getType();

    store_index<CellPort> port_idx(IdString name);

    Net *getPort(IdString name);
    void connectPort(IdString name, store_index<Net> net);
    void disconnectPort(IdString name);

    Net *getPort(store_index<CellPort> index);
    void connectPort(store_index<CellPort> index, store_index<Net> net);
    void disconnectPort(store_index<CellPort> index);

    InstPortRange ports() { return {getType(), inst}; }
};

struct ModuleAccess
{
    ModuleAccess(Netlist &netlist, Module &mod) : netlist(netlist), mod(mod){};
    CellType &getType();

    Netlist &netlist;
    Module &mod;

    CellInstAccess addInst(IdString name, IdString type);
    CellInstAccess getInst(IdString name);
    Net &addNet(IdString name);
    Net *getNet(IdString name);
};

struct NetlistAccess
{
    explicit NetlistAccess(Netlist &netlist) : netlist(netlist){};
    Netlist &netlist;
    ModuleAccess getModule(IdString name) { return {netlist, netlist.modules[name]}; }
    ModuleAccess getModule(store_index<Module> mod) { return {netlist, netlist.modules[mod]}; }
    ModuleAccess createModule(IdString type);
};

NPNR_NAMESPACE_END

#endif
