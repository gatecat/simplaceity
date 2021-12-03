#ifndef PLACER_H
#define PLACER_H

#include "context.h"
#include "netlist.h"

NPNR_NAMESPACE_BEGIN

void global_placer(Context &ctx, Module &mod);

NPNR_NAMESPACE_END

#endif
