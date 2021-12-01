
#ifndef SSO_ARRAY_H
#define SSO_ARRAY_H

#include <cstdint>

#include "preface.h"

NPNR_NAMESPACE_BEGIN

// An small size optimised array that is statically allocated when the size is N or less; heap allocated otherwise
template <typename T, std::size_t N> class SSOArray
{
  private:
    union
    {
        T data_static[N];
        T *data_heap;
    };
    std::size_t m_size;
    inline bool is_heap() const { return (m_size > N); }
    void alloc()
    {
        if (is_heap()) {
            data_heap = new T[m_size];
        }
    }

  public:
    T *data() { return is_heap() ? data_heap : data_static; }
    const T *data() const { return is_heap() ? data_heap : data_static; }
    std::size_t size() const { return m_size; }

    T *begin() { return data(); }
    T *end() { return data() + m_size; }
    const T *begin() const { return data(); }
    const T *end() const { return data() + m_size; }

    SSOArray() : m_size(0){};

    SSOArray(std::size_t size, const T &init = T()) : m_size(size)
    {
        alloc();
        std::fill(begin(), end(), init);
    }

    SSOArray(const SSOArray &other) : m_size(other.size())
    {
        alloc();
        std::copy(other.begin(), other.end(), begin());
    }

    template <typename Tother> SSOArray(const Tother &other) : m_size(other.size())
    {
        alloc();
        std::copy(other.begin(), other.end(), begin());
    }

    ~SSOArray()
    {
        if (is_heap()) {
            delete[] data_heap;
        }
    }

    bool operator==(const SSOArray &other) const
    {
        if (size() != other.size())
            return false;
        return std::equal(begin(), end(), other.begin());
    }
    bool operator!=(const SSOArray &other) const
    {
        if (size() != other.size())
            return true;
        return !std::equal(begin(), end(), other.begin());
    }
    T &operator[](std::size_t idx)
    {
        NPNR_ASSERT(idx < m_size);
        return data()[idx];
    }
    const T &operator[](std::size_t idx) const
    {
        NPNR_ASSERT(idx < m_size);
        return data()[idx];
    }
};

NPNR_NAMESPACE_END

#endif
