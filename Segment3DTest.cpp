#include "Segment3D.hpp"
#include <cassert>

void testIntersection(const Segment3D& s1, const Segment3D& s2, bool shouldIntersect,
                      const Vector3D& expectedPoint = Vector3D(0,0,0)) {
    auto result = s1.Intersect(s2);

    if (shouldIntersect) {
        assert(result.has_value());
        assert((result.value() - expectedPoint).lengthSquared() < EPS * EPS);
    } else {
        assert(!result.has_value());
    }
}

int main() {
    {
        Segment3D s1({0, 0, 0}, {1, 1, 1});
        Segment3D s2({0, 1, 0}, {1, 0, 1});
        testIntersection(s1, s2, true, Vector3D(0.5, 0.5, 0.5));
    }

    {
        Segment3D s1({0, 0, 0}, {1, 0, 0});
        Segment3D s2({2, 0, 0}, {3, 0, 0});
        testIntersection(s1, s2, false);
    }

    {
        Segment3D s1({0, 0, 0}, {2, 0, 0});
        Segment3D s2({1, 0, 0}, {3, 0, 0});
        testIntersection(s1, s2, true, Vector3D(1.5, 0, 0));
    }

    {
        Segment3D point({1, 1, 0}, {1, 1, 0});
        Segment3D segment({0, 0, 0}, {2, 2, 0});
        testIntersection(point, segment, true, Vector3D(1, 1, 0));
    }

    {
        Segment3D point({3, 3, 0}, {3, 3, 0});
        Segment3D segment({0, 0, 0}, {2, 2, 0});
        testIntersection(point, segment, false);
    }

    return 0;
}