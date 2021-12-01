
#ifndef NPNR_CONTAINERS_H
#define NPNR_CONTAINERS_H

#include <algorithm>
#include <limits>
#include <vector>

#include "preface.h"

NPNR_NAMESPACE_BEGIN

template <typename T> struct store_index
{
    index_t m_index = -1;
    store_index() = default;
    explicit store_index(index_t index) : m_index(index){};
    index_t idx() const { return m_index; }
    void set(index_t index) { m_index = index; }
    bool empty() const { return m_index == -1; }
    bool operator<=>(const store_index<T> &other) const = default;
    unsigned int hash() const { return m_index; }

    operator bool() const { return !empty(); }
    bool operator!() const { return empty(); }
};

// "Slotted" indexed object store
template <typename T> class indexed_store
{
  private:
    // This should move to using std::optional at some point
    class slot
    {
      private:
        bool active;
        alignas(T) unsigned char storage[sizeof(T)];
        index_t next_free;
        inline T &obj() { return reinterpret_cast<T &>(storage); }
        inline const T &obj() const { return reinterpret_cast<const T &>(storage); }
        friend class indexed_store<T>;

      public:
        slot() : active(false), next_free(std::numeric_limits<index_t>::max()){};
        slot(slot &&other) : active(other.active), next_free(other.next_free)
        {
            if (active)
                ::new (static_cast<void *>(&storage)) T(std::move(other.obj()));
        };

        template <class... Args> void create(Args &&...args)
        {
            NPNR_ASSERT(!active);
            active = true;
            ::new (static_cast<void *>(&storage)) T(std::forward<Args &&>(args)...);
        }
        bool empty() const { return !active; }
        T &get()
        {
            NPNR_ASSERT(active);
            return reinterpret_cast<T &>(storage);
        }
        const T &get() const
        {
            NPNR_ASSERT(active);
            return reinterpret_cast<const T &>(storage);
        }
        void free(index_t first_free)
        {
            NPNR_ASSERT(active);
            obj().~T();
            active = false;
            next_free = first_free;
        }
        ~slot()
        {
            if (active)
                obj().~T();
        }
    };

    std::vector<slot> slots;
    index_t first_free = 0;

  public:
    // Create a new entry and return its index
    template <class... Args> store_index<T> add(Args &&...args)
    {
        if (first_free == index_t(slots.size())) {
            slots.emplace_back();
            slots.back().create(std::forward<Args &&>(args)...);
            ++first_free;
            return store_index<T>(index_t(slots.size()) - 1);
        } else {
            index_t idx = first_free;
            auto &slot = slots.at(idx);
            first_free = slot.next_free;
            slot.create(std::forward<Args &&>(args)...);
            return store_index<T>(idx);
        }
    }

    // Remove an entry at an index
    void remove(store_index<T> idx)
    {
        slots.at(idx.m_index).free(first_free);
        first_free = idx.m_index;
    }

    // Reserve a certain amount of space
    void reserve(index_t size) { slots.reserve(size); }

    // Check if an index exists
    index_t count(store_index<T> idx)
    {
        if (idx.m_index < 0 || idx.m_index >= index_t(slots.size()))
            return 0;
        return slots.at(idx.m_index).empty() ? 0 : 1;
    }

    // Get an item by index
    T &at(store_index<T> idx) { return slots.at(idx.m_index).get(); }
    const T &at(store_index<T> idx) const { return slots.at(idx.m_index).get(); }
    T &operator[](store_index<T> idx) { return slots.at(idx.m_index).get(); }
    const T &operator[](store_index<T> idx) const { return slots.at(idx.m_index).get(); }

    // Total size of the container
    index_t size() const { return index_t(slots.size()); }

    // Iterate over items
    class iterator
    {
      private:
        indexed_store *base;
        index_t index = 0;

      public:
        iterator(indexed_store *base, index_t index) : base(base), index(index){};
        inline bool operator!=(const iterator &other) const { return other.index != index; }
        inline bool operator==(const iterator &other) const { return other.index == index; }
        inline iterator operator++()
        {
            // skip over unused slots
            do {
                index++;
            } while (index < index_t(base->slots.size()) && !base->slots.at(index).active);
            return *this;
        }
        inline iterator operator++(int)
        {
            iterator prior(*this);
            do {
                index++;
            } while (index < index_t(base->slots.size()) && !base->slots.at(index).active);
            return prior;
        }
        T &operator*() { return base->at(store_index<T>(index)); }
        template <typename It, typename S> friend class enumerated_iterator;
    };
    iterator begin() { return iterator{this, 0}; }
    iterator end() { return iterator{this, index_t(slots.size())}; }

    class const_iterator
    {
      private:
        const indexed_store *base;
        index_t index = 0;

      public:
        const_iterator(const indexed_store *base, index_t index) : base(base), index(index){};
        inline bool operator!=(const const_iterator &other) const { return other.index != index; }
        inline bool operator==(const const_iterator &other) const { return other.index == index; }
        inline const_iterator operator++()
        {
            // skip over unused slots
            do {
                index++;
            } while (index < index_t(base->slots.size()) && !base->slots.at(index).active);
            return *this;
        }
        inline const_iterator operator++(int)
        {
            iterator prior(*this);
            do {
                index++;
            } while (index < index_t(base->slots.size()) && !base->slots.at(index).active);
            return prior;
        }
        const T &operator*() { return base->at(store_index<T>(index)); }
        template <typename It, typename S> friend class enumerated_iterator;
    };
    const_iterator begin() const { return iterator{this, 0}; }
    const_iterator end() const { return iterator{this, index_t(slots.size())}; }

    template <typename S> struct enumerated_item
    {
        enumerated_item(index_t index, T &value) : index(index), value(value){};
        store_index<std::remove_const<S>> index;
        S &value;
    };

    template <typename It, typename S> class enumerated_iterator
    {
      private:
        It base;

      public:
        enumerated_iterator(const It &base) : base(base){};
        inline bool operator!=(const enumerated_iterator<It, S> &other) const { return other.base != base; }
        inline bool operator==(const enumerated_iterator<It, S> &other) const { return other.base == base; }
        inline enumerated_iterator<It, S> operator++()
        {
            ++base;
            return *this;
        }
        inline enumerated_iterator<It, S> operator++(int)
        {
            iterator prior(*this);
            ++base;
            return prior;
        }
        enumerated_item<S> operator*() { return enumerated_item{base.index, *base}; }
    };

    template <typename It, typename S> struct enumerated_range
    {
        enumerated_range(const It &begin, const It &end) : m_begin(begin), m_end(end){};
        enumerated_iterator<It, S> m_begin, m_end;
        enumerated_iterator<It, S> begin() { return m_begin; }
        enumerated_iterator<It, S> end() { return m_end; }
    };

    enumerated_range<iterator, T> enumerate() { return enumerated_range{begin(), end()}; }
    enumerated_range<const_iterator, const T> enumerate() const { return enumerated_range{begin(), end()}; }
};

template <typename T> class array2d
{
  public:
    array2d() : m_width(0), m_height(0), m_size(0), data(nullptr){};
    array2d(int width, int height) : m_width(width), m_height(height), m_size(index_t(width) * index_t(height))
    {
        data = new T[size()]();
    }
    array2d(int width, int height, const T &init)
            : m_width(width), m_height(height), m_size(index_t(width) * index_t(height))
    {
        data = new T[size()];
        std::fill(data, data + size(), init);
    }
    void reset(int new_width, int new_height, const T &init = {})
    {
        index_t new_size = index_t(new_width) * index_t(new_height);
        if (new_size > m_size) {
            delete[] data;
            m_size = new_size;
            data = new T[m_size];
        }
        m_width = new_width;
        m_height = new_height;
        std::fill(data, data + size(), init);
    }

    int width() const { return m_width; }
    int height() const { return m_height; }
    index_t size() const { return index_t(m_width) * index_t(m_height); }
    T &at(int x, int y)
    {
        NPNR_ASSERT(x >= 0 && x < m_width);
        NPNR_ASSERT(y >= 0 && y < m_height);
        return data[y * index_t(m_width) + x];
    }
    const T &at(int x, int y) const
    {
        NPNR_ASSERT(x >= 0 && x < m_width);
        NPNR_ASSERT(y >= 0 && y < m_height);
        return data[y * index_t(m_width) + x];
    }
    ~array2d() { delete[] data; }
    struct entry
    {
        entry(int x, int y, T &value) : x(x), y(y), value(value){};
        int x, y;
        T &value;
    };
    struct iterator
    {
      public:
        entry operator*() { return {x, y, base->at(x, y)}; }
        inline iterator operator++()
        {
            ++x;
            if (x >= base->width()) {
                x = 0;
                ++y;
            }
            return *this;
        }
        inline iterator operator++(int)
        {
            iterator prior(x, y, base);
            ++x;
            if (x >= base->width()) {
                x = 0;
                ++y;
            }
            return prior;
        }
        inline bool operator!=(const iterator &other) const { return other.x != x || other.y != y; }
        inline bool operator==(const iterator &other) const { return other.x == x && other.y == y; }

      private:
        iterator(int x, int y, array2d<T> &base) : x(x), y(y), base(&base){};
        int x, y;
        array2d<T> *base;
        friend class array2d;
    };
    iterator begin() { return {0, 0, *this}; }
    iterator end() { return {0, m_height, *this}; }

  private:
    int m_width, m_height;
    index_t m_size;
    T *data;
};

NPNR_NAMESPACE_END

#endif
