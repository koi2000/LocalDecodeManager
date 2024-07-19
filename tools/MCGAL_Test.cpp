#include "ContextPool.h"
#include "Global.h"
#include "core.h"
#include <iostream>

int main() {
    MCGAL::Mesh m;
    m.loadOFF("/home/koi/mastercode/LocalDecodeManager/static/untitled.off");
    m.edge_collapse(m.faces[0]->halfedges[0]);
    m.dumpto("/home/koi/mastercode/LocalDecodeManager/static/collapse.off");
    return 0;
}