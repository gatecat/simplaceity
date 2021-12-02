#ifndef NETLIST_H
#define NETLIST_H

#include "idstring.h"
#include "object.h"
#include "phys_types.h"
#include "sso_array.h"

#include <optional>

NPNR_NAMESPACE_BEGIN

struct Module;

enum class PortDir
{
    IN = 0,
    OUT = 1,
    INOUT = 2,
};

enum PortUse
{
    SIGNAL = 0,
    CLOCK = 1,
    GROUND = 2,
    POWER = 3,
    // ...
};

// The 'prototype' of a cell port. Shared between all instances of a cell
struct CellPort : BaseObj<CellPort>
{
    CellPort(IdString name, PortDir dir) : BaseObj(name), dir(dir), use(SIGNAL), offset(0, 0){};
    PortDir dir;
    PortUse use;
    Point offset;
};

// The 'prototype' of a cell
struct CellType : BaseObj<CellType>
{
    CellType(IdString type) : BaseObj(type){};
    bool is_phys_only = false;
    object_store<CellPort> ports;

    Size size;
};

struct CellInst;

struct PortRef
{
    PortRef() = default;
    PortRef(store_index<CellInst> inst, store_index<CellPort> port) : inst(inst), port(port){};
    store_index<CellInst> inst;
    store_index<CellPort> port;
    operator bool() const { return !inst.empty(); }
    bool operator!() const { return inst.empty(); }
    bool operator==(const PortRef &other) const { return inst == other.inst && port == other.port; }
    bool operator!=(const PortRef &other) const { return inst != other.inst || port != other.port; }
    unsigned hash() const { return mkhash_add(inst.hash(), port.hash()); }
};

enum class NetType
{
    SIGNAL,
    CLOCK,
    POWER,
    GROUND,
    // ...
};

struct Net : BaseObj<Net>
{
    Net(IdString name) : BaseObj(name){};

    NetType type;
    IdString owner; // the process that created this Net; so we can reliably undo/redo transformations like CTS
    // Top level ports on the net
    pool<store_index<CellPort>> top_ports;
    // All instance ports on the net
    pool<PortRef> inst_ports;
    // If there is a single driver, this is it (for multiply driven nets 'inst_ports' must be iterated to find drivers)
    PortRef driver{};
    // Number of driving ports attached to the net
    index_t driver_count = 0;
    // Approximate location of external pins on the net
    std::vector<Point> ext_pins;
};

struct PortInst
{
    store_index<Net> net;
};

struct CellPlacement
{
    Point loc;
    Orientation orient;
};

// An instance of a physical; and possibly logical; cell. This includes cells inserted by PnR that affect the netlist
// structure like clock buffers - but not fillers and decap. Undecided where diodes fits
struct CellInst : BaseObj<CellInst, IdString>
{
    CellInst(IdString name, const CellType &type)
            : BaseObj(name), type(type.index), ports(type.ports.size(), PortInst{}){};
    store_index<CellType> type;
    IdString owner; // the process that created this CellInst; so we can reliably undo/redo transformations like CTS
    SSOArray<PortInst, 6> ports; // In the same order as the CellType
    std::optional<CellPlacement> placement;
    bool fixed = false;
};

// The contents of a cell
struct Module : BaseObj<Module>
{
    Module(IdString type_name, store_index<CellType> type_idx) : BaseObj(type_name), type(type_idx){};
    // Index of the corresponding CellType
    store_index<CellType> type;
    // Content of the cell
    object_store<CellInst> insts;
    object_store<Net> nets;
    // Mapping from top-level port indices to nets
    std::vector<store_index<Net>> port_nets;
    // Set of power and ground nets for fast access
    pool<store_index<Net>> supply_nets;
    // Placement grid
    Size grid;
};

struct Netlist
{
    Netlist(Context &ctx) : ctx(ctx){};
    Context &ctx; // for IdStrings etc
    object_store<CellType> cell_types;
    object_store<Module> modules;
};

NPNR_NAMESPACE_END
#endif
