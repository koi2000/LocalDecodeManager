#ifndef POINT_H
#define POINT_H
#include <assert.h>
#include <stdexcept>
namespace MCGAL {
class Point {
  public:
    Point() {
        v[0] = 0.0;
        v[1] = 0.0;
        v[2] = 0.0;
    }

    Point(float x, float y, float z) {
        v[0] = x;
        v[1] = y;
        v[2] = z;
    }

    Point(float x, float y, float z, int id) {
        v[0] = x;
        v[1] = y;
        v[2] = z;
        this->id = id;
    }

    Point(Point* pt) {
        assert(pt);
        for (int i = 0; i < 3; i++) {
            v[i] = pt->v[i];
        }
    };

    float x() const {
        return v[0];
    }

    float y() const {
        return v[1];
    }

    float z() const {
        return v[2];
    }

    float& operator[](int index) {
        if (index >= 0 && index < 3) {
            return v[index];
        } else {
            throw std::out_of_range("Index out of range");
        }
    }

  public:
    float v[3];
    int id;
};
}  // namespace MCGAL
#endif