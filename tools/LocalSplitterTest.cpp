#include "../src/include/LocalSplitter.h"

int main() {
    LocalSplitter splitter = LocalSplitter("/home/koi/mastercode/LocalDecodeManager/static/untitled.off");
    splitter.split();
    for (size_t i = 0; i < 10; i++) {
        std::string path = "./submesh/group" + std::to_string(i) + ".off";
        splitter.dumpSubMesh(path, i);
    }
    return 0;
}