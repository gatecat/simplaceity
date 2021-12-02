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
for net in top_cell.getNets():
    if not net.isExternal():
        continue
    if net.getDirection() == Hur.Net.Direction.UNDEFINED:
        continue
    p = top_type.addPort(net.getName(), conv_dir(net.getDirection()))

m = top_type.intoModule()

for net in top_cell.getNets():
    m.addNet(net.getName())
for inst in top_cell.getInstances():
    i = m.addInst(inst.getName(), inst.getMasterCell().getName())
    for plug in inst.getConnectedPlugs():
        i.connectPort(plug.getMasterNet().getName(), plug.getNet().getName())


