#include "../src/include/PartialEncoder.h"

int main() {
    PartialEncoder encoder = PartialEncoder("/home/koi/mastercode/LocalDecodeManager/static/untitled.off");
    encoder.encode(1, 2);
    return 0;
}