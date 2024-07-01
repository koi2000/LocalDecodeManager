#ifndef POINT_H
#define POINT_H
#include <assert.h>
#include <stdexcept>
namespace MCGAL {
class Vector;

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

    // 两个点减法返回一个向量
    Point operator-(const Point& p) const {
        return Point(v[0] - p.x(), v[1] - p.y(), v[2] - p.z());
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

// 定义向量类
class Vector : public Point {
  public:
    double x, y, z;

    Vector(double x, double y, double z) : x(x), y(y), z(z) {}
    
    Vector(Point p) : x(p.x()), y(p.y()), z(p.z()) {}

    // 向量减法
    Vector operator-(const Point& p) const {
        return Vector(x - p.x(), y - p.y(), z - p.z());
    }

    // 向量叉积
    Vector cross(const Vector& v) const {
        return Vector(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    // 向量点积
    double dot(const Vector& v) const {
        return x * v.x + y * v.y + z * v.z;
    }
};

}  // namespace MCGAL
#endif