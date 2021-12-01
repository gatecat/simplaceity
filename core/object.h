#ifndef OBJECT_H
#define OBJECT_H

#include "containers.h"
#include "hashlib.h"
#include "idstring.h"
#include "preface.h"

#include <memory>

NPNR_NAMESPACE_BEGIN

template <typename T, typename Name = IdString> struct BaseObj
{
    using name_t = Name;
    BaseObj(Name name) : name(name){};

    Name name;
    store_index<T> index;
};

template <typename T> struct object_store
{
    dict<typename T::name_t, store_index<std::unique_ptr<T>>> name2idx;
    indexed_store<std::unique_ptr<T>> objects;

    template <class... Args> T *add(Args &&...args)
    {
        auto uptr = std::make_unique<T>(std::forward<Args>(args)...);
        T *ptr = uptr.get();
        ptr->index.set(objects.add(std::move(uptr)).idx());
        NPNR_ASSERT(!name2idx.count(ptr->name));
        name2idx[ptr->name].set(ptr->index.idx());
        return ptr;
    };

    template <class... Args> T *add_or_get(Args &&...args)
    {
        auto uptr = std::make_unique<T>(std::forward<Args>(args)...);
        if (name2idx.count(uptr->name)) {
            return objects.at(name2idx.at(uptr->name)).get();
        }
        T *ptr = uptr.get();
        ptr->index.set(objects.add(std::move(uptr)).idx());
        NPNR_ASSERT(!name2idx.count(ptr->name));
        name2idx[ptr->name].set(ptr->index.idx());
        return ptr;
    };

    void remove(store_index<T> index)
    {
        name2idx.erase(objects.at(index)->name);
        objects.remove(index);
    }
    void remove(typename T::name_t name)
    {
        objects.remove(name2idx.at(name));
        name2idx.erase(name);
    }

    T &operator[](store_index<T> index) { return *objects.at(store_index<std::unique_ptr<T>>(index.idx())); }
    const T &operator[](store_index<T> index) const
    {
        return *objects.at(store_index<std::unique_ptr<T>>(index.idx()));
    }
    T &operator[](typename T::name_t name) { return *objects.at(name2idx.at(name)); }
    const T &operator[](typename T::name_t name) const { return *objects.at(name2idx.at(name)); }

    int count(typename T::name_t name) const { return name2idx.count(name); }
    index_t size() const { return objects.size(); }

    indexed_store<std::unique_ptr<T>>::iterator begin() { return objects.begin(); }
    indexed_store<std::unique_ptr<T>>::iterator end() { return objects.end(); }
    indexed_store<std::unique_ptr<T>>::const_iterator begin() const { return objects.begin(); }
    indexed_store<std::unique_ptr<T>>::const_iterator end() const { return objects.end(); }
};

NPNR_NAMESPACE_END
#endif
