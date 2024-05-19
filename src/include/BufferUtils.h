#ifndef BUFFER_UTILS_H
#define BUFFER_UTILS_H
#include "core.h"
#include <assert.h>
#include <stdint.h>
#include <unistd.h>

// Write a given number of bits in a buffer.
static void writeBits(uint32_t data, unsigned i_nbBits, char* p_dest, unsigned& i_bitOffset, size_t& offset) {
    assert(i_nbBits <= 25);

    uint32_t dataToAdd = data << (32 - i_nbBits - i_bitOffset);
    // Swap the integer bytes because the x86 architecture is little endian.
    dataToAdd = __builtin_bswap32(dataToAdd);  // Call a GCC builtin function.

    // Write the data.
    *(uint32_t*)p_dest |= dataToAdd;

    // Update the size and offset.
    offset += (i_bitOffset + i_nbBits) / 8;
    i_bitOffset = (i_bitOffset + i_nbBits) % 8;
}

/**
 * Read a given number of bits in a buffer.
 */
static uint32_t readBits(unsigned i_nbBits, char* p_src, unsigned& i_bitOffset, size_t& offset) {
    assert(i_nbBits <= 25);

    // Build the mask.
    uint32_t mask = 0;
    for (unsigned i = 0; i < 32 - i_bitOffset; ++i)
        mask |= 1 << i;
    // Swap the mask bytes because the x86 architecture is little endian.
    mask = __builtin_bswap32(mask);  // Call a GCC builtin function.

    uint32_t data = *(uint32_t*)p_src & mask;

    // Swap the integer bytes because the x86 architecture is little endian.
    data = __builtin_bswap32(data);  // Call a GCC builtin function.

    data >>= 32 - i_nbBits - i_bitOffset;

    // Update the size and offset.
    offset += (i_bitOffset + i_nbBits) / 8;
    i_bitOffset = (i_bitOffset + i_nbBits) % 8;

    return data;
}

// Write a floating point number in the data buffer.
static void writeFloat(char* buffer, int& dataOffset, float f) {
    *(float*)(buffer + dataOffset) = f;
    dataOffset += sizeof(float);
}

/**
 * Read a floating point number in the data buffer.
 */
static float readFloat(char* buffer, int& dataOffset) {
    float f = *(float*)(buffer + dataOffset);
    dataOffset += sizeof(float);
    return f;
}

// Write a floating point number in the data buffer.
static void writePoint(char* buffer, int& dataOffset, MCGAL::Point& p) {
    writeInt(buffer, dataOffset, p.id);
    for (unsigned i = 0; i < 3; ++i) {
        writeFloat(buffer, dataOffset, p[i]);
    }
}

// Write a floating point number in the data buffer.
static MCGAL::Point readPoint(char* buffer, int& dataOffset) {
    float coord[3];
    int id = readInt(buffer, dataOffset);
    for (unsigned i = 0; i < 3; ++i) {
        coord[i] = readFloat(buffer, dataOffset);
    }
    MCGAL::Point pt(coord[0], coord[1], coord[2], id);
    return pt;
}

/**
 * Read an integer in the data buffer.
 */
static int readInt(char* buffer, int& dataOffset) {
    int i = *(int*)(buffer + dataOffset);
    dataOffset += sizeof(int);
    return i;
}

// Write an integer in the data buffer
static void writeInt(char* buffer, int& dataOffset, int i) {
    *(int*)(buffer + dataOffset) = i;
    dataOffset += sizeof(int);
}

/**
 * Read a 16 bits integer in the data buffer.
 */
static int16_t readInt16(char* buffer, int& dataOffset) {
    int16_t i = *(int16_t*)(buffer + dataOffset);
    dataOffset += sizeof(int16_t);
    return i;
}

// Write a 16 bits integer in the data buffer
static void writeInt16(char* buffer, int& dataOffset, int16_t i) {
    *(int16_t*)(buffer + dataOffset) = i;
    dataOffset += sizeof(int16_t);
}

/**
 * Read a 16 bits integer in the data buffer.
 */
static uint16_t readuInt16(char* buffer, int& dataOffset) {
    uint16_t i = *(uint16_t*)(buffer + dataOffset);
    dataOffset += sizeof(uint16_t);
    return i;
}

// Write a 16 bits integer in the data buffer
static void writeuInt16(char* buffer, int& dataOffset, uint16_t i) {
    *(uint16_t*)(buffer + dataOffset) = i;
    dataOffset += sizeof(uint16_t);
}

/**
 * Read a byte in the data buffer.
 */
static unsigned char readChar(char* buffer, int& dataOffset) {
    unsigned char i = *(unsigned char*)(buffer + dataOffset);
    dataOffset += sizeof(unsigned char);
    return i;
}

// Write a byte in the data buffer
static void writeChar(char* buffer, int& dataOffset, unsigned char i) {
    *(unsigned char*)(buffer + dataOffset) = i;
    dataOffset += sizeof(unsigned char);
}

static float readFloatByOffset(char* buffer, int offset) {
    float f = *(float*)(buffer + offset);
    return f;
}

static int16_t readInt16ByOffset(char* buffer, int offset) {
    int16_t i = *(int16_t*)(buffer + offset);
    return i;
}

static uint16_t readuInt16ByOffset(char* buffer, int offset) {
    uint16_t i = *(uint16_t*)(buffer + offset);
    return i;
}

static int readIntByOffset(char* buffer, int offset) {
    int i = *(int*)(buffer + offset);
    return i;
}

static unsigned char readCharByOffset(char* buffer, int offset) {
    unsigned char i = *(unsigned char*)(buffer + offset);
    return i;
}

static MCGAL::Point readPointByOffset(char* buffer, int offset) {
    float coord[3];
    int id = readIntByOffset(buffer, offset);
    offset += sizeof(int);
    for (unsigned i = 0; i < 3; ++i) {
        coord[i] = readFloatByOffset(buffer, offset);
        offset += sizeof(float);
    }
    MCGAL::Point pt(coord[0], coord[1], coord[2], id);
    return pt;
}

static float readFloatByOffsetRef(char* buffer, int& offset) {
    float f = *(float*)(buffer + offset);
    offset += sizeof(float);
    return f;
}

static int16_t readInt16ByOffsetRef(char* buffer, int& offset) {
    int16_t i = *(int16_t*)(buffer + offset);
    offset += sizeof(int16_t);
    return i;
}

static uint16_t readuInt16ByOffsetRef(char* buffer, int& offset) {
    uint16_t i = *(uint16_t*)(buffer + offset);
    offset += sizeof(uint16_t);
    return i;
}

static int readIntByOffsetRef(char* buffer, int& offset) {
    int i = *(int*)(buffer + offset);
    offset += sizeof(int);
    return i;
}

static unsigned char readCharByOffsetRef(char* buffer, int& offset) {
    unsigned char i = *(unsigned char*)(buffer + offset);
    offset += sizeof(unsigned char);
    return i;
}

static MCGAL::Point readPointByOffsetRef(char* buffer, int& offset) {
    float coord[3];
    int id = readIntByOffset(buffer, offset);
    offset += sizeof(int);
    for (unsigned i = 0; i < 3; ++i) {
        coord[i] = readFloatByOffset(buffer, offset);
        offset += sizeof(float);
    }
    MCGAL::Point pt(coord[0], coord[1], coord[2], id);
    return pt;
}

#endif