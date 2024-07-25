#include "../src/include/PartialDecoder.h"

int main() {
    PartialDecoder decoder = PartialDecoder("./bunny.loc");
    decoder.decode(3, 3);
    return 0;
}