#include "ContextPool.h"
#include "Global.h"
#include <iostream>

int main() {
    MCGAL::Vertex* v = MCGAL::contextPool.allocateVertexFromPool();
    std::cout << v->poolId << std::endl;
    return 0;
}