#ifndef ENCODE_BOUNDARY_MESSAGE_H
#define ENCODE_BOUNDARY_MESSAGE_H
#include "core.h"

constexpr int SCHEMA_FIXED_LENGTH = sizeof(int) * 4 + sizeof(MCGAL::Point);
class EncodeBoundarySchema {
  public:
    EncodeBoundarySchema() {}

    EncodeBoundarySchema(int vid_, MCGAL::Point p_, int groupId1_, int groupId2_, int needMovedSize_, char* needMoved_)
        : vid(vid_), p(p_), groupId1(groupId1_), groupId2(groupId2_), needMoved(needMoved_) {}

    void dumpEncodeBoundarySchema(char* buffer, int& dataOffset);
    void loadEncodeBoundarySchema(char* buffer, int& dataOffset);

    ~EncodeBoundarySchema() {
        // delete needMoved;
    }

    int getVid() const;
    void setVid(int vid);

    MCGAL::Point getP() const;
    void setP(const MCGAL::Point& p);

    int getGroupId1() const;
    void setGroupId1(int groupId1);

    int getGroupId2() const;
    void setGroupId2(int groupId2);

    int getVid1() const;
    void setVid1(int vid1);

    int getVid2() const;
    void setVid2(int vid2);

    int getNeedMovedSize() const;
    void setNeedMovedSize(int needMovedSize);

    char* getNeedMoved() const;
    void setNeedMoved(char* needMoved);

  private:
    // 合并所得点的vid
    int vid;
    // 被删除点的完整信息
    MCGAL::Point p;
    // 两个新产生的边的groupId
    int groupId1;
    int groupId2;
    int vid1;
    int vid2;
    // needMoved的指针大小
    int needMovedSize;
    // 需要移动点的bitmap
    char* needMoved = nullptr;
    std::vector<int> schemaOffset;
};
#endif