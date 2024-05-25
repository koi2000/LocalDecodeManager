#include "../src/include/LocalEncoder.h"

int main() {
    LocalEncoder encoder = LocalEncoder("/home/koi/mastercode/LocalDecodeManager/static/untitled.off");
    encoder.encode();
    encoder.dumpToFile("./bunny.loc");
    return 0;
}