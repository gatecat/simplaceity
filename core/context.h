#ifndef CONTEXT_H
#define CONTEXT_H
#include "idstring.h"
#include "preface.h"

#include <unordered_map>
#include <vector>

NPNR_NAMESPACE_BEGIN

struct Context
{
    Context();
    ~Context();

    // ID String database.
    mutable std::unordered_map<std::string, int> *idstring_str_to_idx;
    mutable std::vector<const std::string *> *idstring_idx_to_str;
    IdString id(const std::string &s) { return IdString(this, s); };
};

NPNR_NAMESPACE_END
#endif
