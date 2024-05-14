#ifndef LOCALDECODEMANAGER_H
#define LOCALDECODEMANAGER_H
#include <string>

class LocalDecodeManager {
public:
    // 直接读取文件
    LocalDecodeManager(std::string filename);

    void readMesh();

    void encode();

    void dumpBuffer();

private:

};

#endif