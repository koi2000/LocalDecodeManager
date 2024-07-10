#include "./include/EncodeBoundarySchema.h"
#include "BufferUtils.h"

#include "EncodeBoundarySchema.h"

// Getter and Setter for vid
int EncodeBoundarySchema::getVid() const {
    return vid;
}

void EncodeBoundarySchema::setVid(int vid) {
    this->vid = vid;
}

// Getter and Setter for p (MCGAL::Point)
MCGAL::Point EncodeBoundarySchema::getP() const {
    return p;
}

void EncodeBoundarySchema::setP(const MCGAL::Point& p) {
    this->p = p;
}

// Getter and Setter for groupId1
int EncodeBoundarySchema::getGroupId1() const {
    return groupId1;
}

void EncodeBoundarySchema::setGroupId1(int groupId1) {
    this->groupId1 = groupId1;
}

// Getter and Setter for groupId2
int EncodeBoundarySchema::getGroupId2() const {
    return groupId2;
}

void EncodeBoundarySchema::setGroupId2(int groupId2) {
    this->groupId2 = groupId2;
}

// Getter and Setter for groupId1
int EncodeBoundarySchema::getVid1() const {
    return vid1;
}

void EncodeBoundarySchema::setVid1(int vid1) {
    this->vid1 = vid1;
}

// Getter and Setter for groupId2
int EncodeBoundarySchema::getVid2() const {
    return vid2;
}

void EncodeBoundarySchema::setVid2(int vid2) {
    this->vid2 = vid2;
}

// Getter and Setter for needMovedSize
int EncodeBoundarySchema::getNeedMovedSize() const {
    return needMovedSize;
}

void EncodeBoundarySchema::setNeedMovedSize(int needMovedSize) {
    this->needMovedSize = needMovedSize;
}

// Getter and Setter for needMoved
char* EncodeBoundarySchema::getNeedMoved() const {
    return needMoved;
}

void EncodeBoundarySchema::setNeedMoved(char* needMoved) {
    this->needMoved = needMoved;
}

void EncodeBoundarySchema::dumpEncodeBoundarySchema(char* buffer, int& dataOffset) {
    writeInt(buffer, dataOffset, vid);
    writePoint(buffer, dataOffset, p);
    writeInt(buffer, dataOffset, groupId1);
    writeInt(buffer, dataOffset, groupId2);
    writeInt(buffer, dataOffset, vid1);
    writeInt(buffer, dataOffset, vid2);
    writeInt(buffer, dataOffset, needMovedSize);
    writeCharPointer(buffer, dataOffset, needMoved, needMovedSize);
}

void EncodeBoundarySchema::loadEncodeBoundarySchema(char* buffer, int& dataOffset) {
    vid = readInt(buffer, dataOffset);
    p = readPoint(buffer, dataOffset);
    groupId1 = readInt(buffer, dataOffset);
    groupId2 = readInt(buffer, dataOffset);
    vid1 = readInt(buffer, dataOffset);
    vid2 = readInt(buffer, dataOffset);
    needMovedSize = readInt(buffer, dataOffset);
    needMoved = new char[needMovedSize];
    readCharPointer(buffer, dataOffset, needMoved, needMovedSize);
}