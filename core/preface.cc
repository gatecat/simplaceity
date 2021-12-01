#include "preface.h"
#include "log.h"

NPNR_NAMESPACE_BEGIN

assertion_failure::assertion_failure(std::string msg, std::string expr_str, std::string filename, int line)
        : runtime_error("Assertion failure: " + msg + " (" + filename + ":" + std::to_string(line) + ")"), msg(msg),
          expr_str(expr_str), filename(filename), line(line)
{
    log_flush();
}

NPNR_NAMESPACE_END
