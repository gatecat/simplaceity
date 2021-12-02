#include <pybind11/pybind11.h>

#include "context.h"
#include "netlist.h"
#include "netlist_access.h"

namespace py = pybind11;

NPNR_NAMESPACE_BEGIN

struct PyModule;

struct PyCellType
{
    std::shared_ptr<Context> ctx;
    store_index<CellType> type_idx;
    CellType &type() { return ctx->netlist().cell_types[type_idx]; }
    void setSize(dist_t width, dist_t height) { type().size = Size(width, height); }
    void addPort(const std::string &name, PortDir dir) { type().ports.add(ctx->id(name), dir); }
    void setPortUse(const std::string &name, PortUse use) { type().ports[ctx->id(name)].use = use; }
    void setPortOffset(const std::string &name, dist_t x, dist_t y)
    {
        type().ports[ctx->id(name)].offset = Point(x, y);
    }
    PyModule intoModule();
};

struct PyNet
{
    std::shared_ptr<Context> ctx;
    store_index<Module> mod_idx;
    store_index<Net> net_idx;
    Module &mod() { return ctx->netlist().modules[mod_idx]; }
    Net &net() { return mod().nets[net_idx]; }
    const std::string &getName() { return net().name.str(ctx.get()); }
    void setType(NetType type) { net().type = type; }
    void addExtPin(dist_t x, dist_t y) { net().ext_pins.emplace_back(x, y); }
};

struct PyCellInst
{
    std::shared_ptr<Context> ctx;
    store_index<Module> mod_idx;
    store_index<CellInst> inst_idx;
    Module &mod() { return ctx->netlist().modules[mod_idx]; }
    CellInst &inst() { return mod().insts[inst_idx]; }
    CellInstAccess access() { return {ctx->netlist(), mod(), inst()}; }
    const std::string &getName() { return inst().name.str(ctx.get()); }
    PyCellType getType() { return {ctx, inst().type}; }
    void connectPort(const std::string &port, const std::string &net)
    {
        IdString net_id = ctx->id(net);
        store_index<Net> net_idx;
        if (!mod().nets.count(net_id))
            net_idx = ModuleAccess(ctx->netlist(), mod()).addNet(net_id).index;
        else
            net_idx = mod().nets[net_id].index;
        access().connectPort(ctx->id(port), net_idx);
    }
    void disconnectPort(const std::string &port) { access().disconnectPort(ctx->id(port)); }
    void setFixed(Point p, Orientation orient)
    {
        inst().fixed = true;
        inst().placement = CellPlacement{p, orient};
    }
    py::object getLoc() { return inst().placement ? py::cast(inst().placement->loc) : py::none(); }
    py::object getOrient() { return inst().placement ? py::cast(inst().placement->orient) : py::none(); }
};

struct PyModule
{
    std::shared_ptr<Context> ctx;
    store_index<Module> mod_idx;
    Module &mod() { return ctx->netlist().modules[mod_idx]; }
    ModuleAccess access() { return ModuleAccess(ctx->netlist(), mod()); }
    PyNet addNet(const std::string &name) { return PyNet{ctx, mod_idx, access().addNet(ctx->id(name)).index}; }
    PyNet getNet(const std::string &name) { return PyNet{ctx, mod_idx, mod().nets[ctx->id(name)].index}; }
    PyCellInst addInst(const std::string &name, const std::string &type)
    {
        return PyCellInst{ctx, mod_idx, access().addInst(ctx->id(name), ctx->id(type)).inst.index};
    }
    PyCellInst getInst(const std::string &name) { return PyCellInst{ctx, mod_idx, mod().insts[ctx->id(name)].index}; }
};

PyModule PyCellType::intoModule()
{
    IdString name = type().name;
    if (ctx->netlist().modules.count(name))
        return PyModule{ctx, ctx->netlist().modules[name].index};
    else
        return PyModule{ctx, ctx->netlist().modules.add(name, type_idx)->index};
}

struct PyPlcContext
{
    PyPlcContext() : ctx(std::make_shared<Context>()){};
    std::shared_ptr<Context> ctx;
    PyCellType addCellType(const std::string &name)
    {
        return {ctx, ctx->netlist().cell_types.add(ctx->id(name))->index};
    }
    PyCellType getCellType(const std::string &name) { return {ctx, ctx->netlist().cell_types[ctx->id(name)].index}; }
};

NPNR_NAMESPACE_END

USING_NPNR_NAMESPACE;

PYBIND11_MODULE(simplaceity, m)
{
    m.doc() = "simplaceity placer";

    py::class_<Point>(m, "Point")
            .def(py::init<>())
            .def(py::init<dist_t, dist_t>())
            .def_readwrite("x", &Point::x)
            .def_readwrite("y", &Point::y);

    py::class_<Orientation>(m, "Orientation")
            .def(py::init<>())
            .def_readwrite("dir", &Orientation::dir)
            .def_readwrite("mirror_x", &Orientation::mirror_x)
            .def_readwrite("mirror_y", &Orientation::mirror_y);

    py::class_<PyCellType>(m, "PyCellType")
            .def("setSize", &PyCellType::setSize)
            .def("addPort", &PyCellType::addPort)
            .def("setPortOffset", &PyCellType::setPortOffset)
            .def("setPortUse", &PyCellType::setPortUse)
            .def("intoModule", &PyCellType::intoModule);

    py::class_<PyNet>(m, "PyNet")
            .def("getName", &PyNet::getName)
            .def("setType", &PyNet::setType)
            .def("addExtPin", &PyNet::addExtPin);

    py::class_<PyCellInst>(m, "PyCellInst")
            .def("getName", &PyCellInst::getName)
            .def("getType", &PyCellInst::getType)
            .def("connectPort", &PyCellInst::connectPort)
            .def("disconnectPort", &PyCellInst::disconnectPort)
            .def("setFixed", &PyCellInst::setFixed)
            .def("getLoc", &PyCellInst::getLoc)
            .def("getOrient", &PyCellInst::getOrient);

    py::class_<PyModule>(m, "PyModule")
            .def("addNet", &PyModule::addNet)
            .def("getNet", &PyModule::getNet)
            .def("addInst", &PyModule::addInst)
            .def("getInst", &PyModule::getInst);

    py::class_<PyPlcContext>(m, "PyContext")
            .def(py::init<>())
            .def("addCellType", &PyPlcContext::addCellType)
            .def("getCellType", &PyPlcContext::getCellType);

    py::enum_<PortDir>(m, "PortDir").value("IN", PortDir::IN).value("OUT", PortDir::OUT).value("INOUT", PortDir::INOUT);

    py::enum_<PortUse>(m, "PortUse")
            .value("SIGNAL", PortUse::SIGNAL)
            .value("GROUND", PortUse::GROUND)
            .value("POWER", PortUse::POWER)
            .value("CLOCK", PortUse::CLOCK);
}
