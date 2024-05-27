#include "../src/include/LocalDecoder.h"

int main() {
    LocalDecoder decoder = LocalDecoder("./bunny.loc");
    decoder.dumpToOFF("./res1.off");
    // decoder.decodeOp(0, 2);
    decoder.decodeOp(1, 2);
    decoder.dumpToOFF("./res2.off");
    return 0;
}