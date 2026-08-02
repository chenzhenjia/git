#pragma once

#include <cstdint>
#include <string>
#include <vector>

#ifdef _WIN32
#include <Windows.h>
#endif

/// PLY header parse result
struct PlyHeader {
    uint64_t vertexCount = 0;
    uint64_t dataOffset  = 0;
    int stride       = 0;
    int xOff = -1, yOff = -1, zOff = -1;
    int rOff = -1, gOff = -1, bOff = -1;
    int nxOff = -1, nyOff = -1, nzOff = -1;
    bool isAscii   = false;
    bool hasColor()  const { return rOff >= 0; }
    bool hasNormal() const { return nxOff >= 0; }
};

/// Memory-mapped PLY reader. Supports both binary_little_endian
/// and ascii 1.0 formats via the same readPoint/readPosition API.
class MmapPlyReader {
public:
    MmapPlyReader() = default;
    ~MmapPlyReader();

    MmapPlyReader(const MmapPlyReader&) = delete;
    MmapPlyReader& operator=(const MmapPlyReader&) = delete;

    bool open(const char *path, std::string &error);
    void close();

    const PlyHeader &header()   const { return m_hdr; }
    uint64_t          pointCount() const { return m_hdr.vertexCount; }
    uint64_t          fileSize()   const { return m_fileSize; }

    void readPoint(uint64_t idx, float *xyz, float *rgb) const;
    void readPosition(uint64_t idx, float *xyz) const;

    // helpers for routing
    static bool isAsciiPly(const char *path);
    static uint64_t safeFileSize(const char *path);

private:
    bool parseHeader();
    bool parseAsciiData(std::string &error);

#ifdef _WIN32
    HANDLE        m_hFile    = nullptr;
    HANDLE        m_hMapping = nullptr;
#endif
    const uint8_t *m_data    = nullptr;
    uint64_t       m_fileSize = 0;
    PlyHeader      m_hdr;

    // ASCII fallback: parsed float data in [xyz, rgb] per point format
    std::vector<float> m_parsedData;
};