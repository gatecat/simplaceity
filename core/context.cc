#include "context.h"
#include "netlist.h"

NPNR_NAMESPACE_BEGIN

Context::Context()
{
    idstring_str_to_idx = new std::unordered_map<std::string, int>();
    idstring_idx_to_str = new std::vector<const std::string *>();
    IdString::initialize_add(this, "", 0); // null string is an empty ID
    _netlist = std::make_unique<Netlist>(*this);
}

Context::~Context()
{
    delete idstring_str_to_idx;
    delete idstring_idx_to_str;
}

NPNR_NAMESPACE_END
