
#ifndef PREFACE_H
#define PREFACE_H

#include <stdexcept>
#include <stdint.h>
#include <string>

#ifndef NPNR_NAMESPACE
#define NPNR_NAMESPACE Simplaceity
#endif

#ifdef NPNR_NAMESPACE
#define NPNR_NAMESPACE_PREFIX NPNR_NAMESPACE::
#define NPNR_NAMESPACE_BEGIN namespace NPNR_NAMESPACE {
#define NPNR_NAMESPACE_END }
#define USING_NPNR_NAMESPACE using namespace NPNR_NAMESPACE;
#else
#define NPNR_NAMESPACE_PREFIX
#define NPNR_NAMESPACE_BEGIN
#define NPNR_NAMESPACE_END
#define USING_NPNR_NAMESPACE
#endif

#if defined(__GNUC__) || defined(__clang__)
#define NPNR_ATTRIBUTE(...) __attribute__((__VA_ARGS__))
#define NPNR_NORETURN __attribute__((noreturn))
#define NPNR_DEPRECATED __attribute__((deprecated))
#define NPNR_PACKED_STRUCT(...) __VA_ARGS__ __attribute__((packed))
#define NPNR_ALWAYS_INLINE NPNR_ATTRIBUTE(__always_inline__)
#elif defined(_MSC_VER)
#define NPNR_ATTRIBUTE(...)
#define NPNR_NORETURN __declspec(noreturn)
#define NPNR_DEPRECATED __declspec(deprecated)
#define NPNR_PACKED_STRUCT(...) __pragma(pack(push, 1)) __VA_ARGS__ __pragma(pack(pop))
#define NPNR_ALWAYS_INLINE
#else
#define NPNR_ATTRIBUTE(...)
#define NPNR_NORETURN
#define NPNR_DEPRECATED
#define NPNR_PACKED_STRUCT(...) __VA_ARGS__
#define NPNR_ALWAYS_INLINE
#endif

NPNR_NAMESPACE_BEGIN

#ifdef NPNR64
typedef int64_t index_t;
typedef int64_t dist_t;
#else
typedef int32_t index_t;
typedef int32_t dist_t;
#endif

class assertion_failure : public std::runtime_error
{
  public:
    assertion_failure(std::string msg, std::string expr_str, std::string filename, int line);

    std::string msg;
    std::string expr_str;
    std::string filename;
    int line;
};

NPNR_NORETURN
inline void assert_fail_impl(const char *message, const char *expr_str, const char *filename, int line)
{
    throw assertion_failure(message, expr_str, filename, line);
}

NPNR_NORETURN
inline void assert_fail_impl_str(std::string message, const char *expr_str, const char *filename, int line)
{
    throw assertion_failure(message, expr_str, filename, line);
}

#define NPNR_ASSERT(cond) (!(cond) ? assert_fail_impl(#cond, #cond, __FILE__, __LINE__) : (void)true)
#define NPNR_ASSERT_MSG(cond, msg) (!(cond) ? assert_fail_impl(msg, #cond, __FILE__, __LINE__) : (void)true)
#define NPNR_ASSERT_FALSE(msg) (assert_fail_impl(msg, "false", __FILE__, __LINE__))
#define NPNR_ASSERT_FALSE_STR(msg) (assert_fail_impl_str(msg, "false", __FILE__, __LINE__))

#define NPNR_STRINGIFY_MACRO(x) NPNR_STRINGIFY(x)
#define NPNR_STRINGIFY(x) #x

NPNR_NAMESPACE_END

#endif