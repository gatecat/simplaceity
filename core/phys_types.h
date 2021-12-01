#ifndef PHYS_TYPES_H
#define PHYS_TYPES_H

#include "preface.h"

#include <cmath>
#include <limits>

NPNR_NAMESPACE_BEGIN

struct Point
{
    Point() : x(0), y(0){};
    Point(dist_t x, dist_t y) : x(x), y(y){};
    auto operator<=>(const Point &other) const = default;

    Point operator+(const Point &other) const { return {x + other.x, y + other.y}; }
    Point operator-(const Point &other) const { return {x - other.x, y - other.y}; }

    dist_t manhattan_distance(const Point &other) const { return std::abs(x - other.x) + std::abs(y - other.y); }

    dist_t x, y;
};

struct Box
{
    Box()
            : x0(std::numeric_limits<dist_t>::max()), y0(std::numeric_limits<dist_t>::max()),
              x1(std::numeric_limits<dist_t>::min()), y1(std::numeric_limits<dist_t>::min()){};
    Box(Point p) : x0(p.x), y0(p.y), x1(p.x), y1(p.y){};
    Box(Point p0, Point p1) : x0(p0.x), y0(p0.y), x1(p1.x), y1(p1.y){};
    Box(dist_t x0, dist_t y0, dist_t x1, dist_t y1) : x0(x0), y0(y0), x1(x1), y1(y1){};

    bool contains(Point p, bool inclusive = true) const
    {
        return inclusive ? (p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1)
                         : (p.x > x0 && p.x < x1 && p.y > y0 && p.y < y1);
    }

    void extend(Point p)
    {
        x0 = std::min(x0, p.x);
        y0 = std::min(y0, p.y);
        x1 = std::max(x1, p.x);
        y1 = std::max(y1, p.y);
    }

    void extend(Box b)
    {
        x0 = std::min(x0, b.x0);
        y0 = std::min(y0, b.y0);
        x1 = std::max(x1, b.x0);
        y1 = std::max(y1, b.y0);
    }

    dist_t area() const { return std::max(x1 - x0, dist_t(0)) * std::max(y1 - y0, dist_t(0)); }

    Box rotate_90() const { return Box(y0, x0, y1, x1); }

    dist_t x0, y0, x1, y1;
};

inline Box operator+(const Point &p, const Box &b) { return Box(b.x0 + p.x, b.y0 + p.y, b.x1 + p.x, b.y1 + p.y); }

struct Size
{
    dist_t width, height;
};

enum class Axis : uint8_t
{
    H,
    V,
    UNKNOWN
};

enum class Dir : uint8_t
{
    NORTH,
    SOUTH,
    EAST,
    WEST,
};

struct Orientation
{
    Dir dir;
    bool mirror_x : 1;
    bool mirror_y : 1;
};

NPNR_NAMESPACE_END

#endif
