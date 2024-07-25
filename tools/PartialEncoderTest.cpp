#include "../src/include/PartialEncoder.h"

int main() {
    PartialEncoder encoder = PartialEncoder("/home/koi/mastercode/LocalDecodeManager/static/untitled.off");
    encoder.encode(3);
    return 0;
}