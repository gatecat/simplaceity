import os, sys
import CRL, Hurricane as Hur, Katana, Etesian, Anabatic, Cfg
from Hurricane import DataBase, Transformation, Box, Instance
from helpers import u, l, setNdaTopDir
from helpers.overlay import CfgCache, UpdateSession

ndadir = "../../open_pdk/C4M.Sky130/libs.tech/coriolis/techno"
setNdaTopDir(ndadir)

from NDA.node130 import sky130 as tech

tech.setup()
tech.FlexLib_setup()

db = DataBase.getDB()
flexlib = db.getRootLibrary().getLibrary("FlexLib")

from simplaceity import *
ctx = PyContext()

ctx.setUnits(u(1))

def conv_dir(d):
    if d == Hur.Net.Direction.IN: return PortDir.IN
    elif d == Hur.Net.Direction.OUT: return PortDir.OUT
    elif d == Hur.Net.Direction.INOUT: return PortDir.INOUT
    else: assert False, d

def conv_use(t):
    if t == Hur.Net.Type.CLOCK: return PortUse.CLOCK
    if t == Hur.Net.Type.GROUND: return PortUse.GROUND
    if t == Hur.Net.Type.POWER: return PortUse.POWER
    else: return PortUse.SIGNAL

def conv_orient(o):
    if o == Orientation.N: return Transformation.Orientation.ID
    if o == Orientation.S: return Transformation.Orientation.R2
    if o == Orientation.FN: return Transformation.Orientation.MY
    if o == Orientation.FS: return Transformation.Orientation.MX
    else: assert False, o

for lib_cell in flexlib.getCells():
    c = ctx.addCellType(lib_cell.getName())
    bb = lib_cell.getAbutmentBox()
    c.setSize(bb.getXMax(), bb.getYMax())
    for net in lib_cell.getNets():
        if not net.isExternal():
            continue
        p = c.addPort(net.getName(), conv_dir(net.getDirection()))
        for net_pad in net.getExternalComponents():
            # TODO: case of multiple pins
            c.setPortOffset(net.getName(), net_pad.getX(), net_pad.getY())
            break
        c.setPortUse(net.getName(), conv_use(net.getType()))

top_cell = CRL.Blif.load("picosoc_top")

top_type = ctx.addCellType(top_cell.getName())
ext_count = 0
for net in top_cell.getNets():
    if not net.isExternal():
        continue
    if net.getDirection() == Hur.Net.Direction.UNDEFINED:
        continue
    p = top_type.addPort(net.getName(), conv_dir(net.getDirection()))
    ext_count += 1

top_type.setSize(u(4800), u(4800))

m = top_type.intoModule()

for net in top_cell.getNets():
    m.addNet(net.getName())

placed_insts = []

for inst in top_cell.getInstances():
    i = m.addInst(inst.getName(), inst.getMasterCell().getName())
    for plug in inst.getConnectedPlugs():
        i.connectPort(plug.getMasterNet().getName(), plug.getNet().getName())
    placed_insts.append(i)

m.setGrid(u(1.2), u(12))

pin_idx = 0
for net in top_cell.getNets():
    if not net.isExternal():
        continue
    if net.getDirection() == Hur.Net.Direction.UNDEFINED:
        continue
    # TODO: real pin locations
    if pin_idx < ext_count//2:
        m.getNet(net.getName()).addExtPin(0, u(2.4) * pin_idx)
    else:
        m.getNet(net.getName()).addExtPin(u(4800), u(2.4) * (pin_idx - ext_count//2))
    pin_idx += 1

m.getNet("vss").setType(NetType.GROUND)
m.getNet("vdd").setType(NetType.POWER)
m.getNet("io_in(0)").setType(NetType.CLOCK)

m.globalPlace()
m.detailPlace()

with UpdateSession():
    for i in placed_insts:
        cor_inst = top_cell.getInstance(i.getName())
        loc = i.getLoc()
        orient = i.getOrient()
        cor_inst.setTransformation(Transformation(loc.x, loc.y, conv_orient(orient)))
        cor_inst.setPlacementStatus(Instance.PlacementStatus.FIXED)

CRL.Gds.save(top_cell)
