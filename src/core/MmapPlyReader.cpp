#include <fstream>
#include "MmapPlyReader.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <sstream>

// ===================================================================
//  Static helpers
// ===================================================================
bool MmapPlyReader::isAsciiPly(const char *path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    char buf[256] = {};
    f.read(buf, sizeof(buf) - 1);
    std::string s(buf);
    return s.find("format ascii") != std::string::npos;
}

uint64_t MmapPlyReader::safeFileSize(const char *path) {
#ifdef _WIN32
    WIN32_FILE_ATTRIBUTE_DATA attr;
    if (!GetFileAttributesExA(path, GetFileExInfoStandard, &attr))
        return 0;
    return (static_cast<uint64_t>(attr.nFileSizeHigh) << 32) | attr.nFileSizeLow;
#else
    std::error_code ec;
    return std::filesystem::file_size(path, ec);
#endif
}

// ===================================================================
//  Win32 mmap
// ===================================================================
#ifdef _WIN32
static bool win32Map(const char *path, HANDLE &hFile, HANDLE &hMapping,
                     const uint8_t *&data, uint64_t &fileSize,
                     std::string &error) {
    hFile = CreateFileA(path, GENERIC_READ, FILE_SHARE_READ, nullptr,
                        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (hFile == INVALID_HANDLE_VALUE) {
        error = "Cannot open file: " + std::string(path); return false;
    }
    LARGE_INTEGER li;
    if (!GetFileSizeEx(hFile, &li)) {
        CloseHandle(hFile); error = "Cannot get file size"; return false;
    }
    fileSize = static_cast<uint64_t>(li.QuadPart);
    hMapping = CreateFileMappingW(hFile, nullptr, PAGE_READONLY, 0, 0, nullptr);
    if (!hMapping) { CloseHandle(hFile); error = "CreateFileMapping failed"; return false; }
    data = static_cast<const uint8_t *>(MapViewOfFile(hMapping, FILE_MAP_READ, 0, 0, 0));
    if (!data) { CloseHandle(hMapping); CloseHandle(hFile); error = "MapViewOfFile failed"; return false; }
    return true;
}
#endif

MmapPlyReader::~MmapPlyReader() { close(); }
void MmapPlyReader::close() {
#ifdef _WIN32
    if (m_data) { UnmapViewOfFile(m_data); m_data = nullptr; }
    if (m_hMapping) { CloseHandle(m_hMapping); m_hMapping = nullptr; }
    if (m_hFile) { CloseHandle(m_hFile); m_hFile = nullptr; }
#endif
    m_fileSize = 0;
}

// ===================================================================
//  open() �� entry point: mmap �� parse header �� if ASCII: parse data
// ===================================================================
bool MmapPlyReader::open(const char *path, std::string &error) {
    close();

#ifdef _WIN32
    if (!win32Map(path, m_hFile, m_hMapping, m_data, m_fileSize, error))
        return false;
#else
    error = "Only Windows mmap supported"; return false;
#endif

    if (!parseHeader()) {
        error = "Failed to parse PLY header."; close(); return false;
    }

    // ASCII fallback: parse text into float array
    if (m_hdr.isAscii) {
        if (!parseAsciiData(error)) { close(); return false; }
    }

    return true;
}

// ===================================================================
//  PLY header parser (supports binary_little_endian AND ascii)
// ===================================================================
bool MmapPlyReader::parseHeader() {
    const char *p = reinterpret_cast<const char *>(m_data);
    const uint64_t scan = std::min<uint64_t>(m_fileSize, 65536);

    const char *term = std::strstr(p, "end_header\n");
    if (!term) term = std::strstr(p, "end_header\r\n");
    if (!term) return false;

    // Detect format
    bool isBinary = (std::strstr(p, "binary_little_endian") != nullptr);
    bool isAscii  = (std::strstr(p, "format ascii") != nullptr);
    if (!isBinary && !isAscii) return false;

    m_hdr = PlyHeader{};
    m_hdr.isAscii = isAscii;
    m_hdr.dataOffset = static_cast<uint64_t>(term - p) + std::strlen("end_header\n");

    int currentOffset = 0;
    const char *line = p;
    while (line < term) {
        const char *nl = std::strchr(line, '\n');
        if (!nl) break;
        std::string s(line, nl - line);
        if (!s.empty() && s.back() == '\r') s.pop_back();

        if (s.rfind("element vertex ", 0) == 0) {
            m_hdr.vertexCount = std::stoull(s.substr(15));
        } else if (s.rfind("property ", 0) == 0 && isBinary) {
            // Binary: compute stride/offsets
            std::istringstream iss(s);
            std::string kw, type, name; iss >> kw >> type >> name;
            if (type.empty() || name.empty()) { line = nl + 1; continue; }
            int sz = 0;
            if (type == "float"||type=="float32") sz=4;
            else if (type == "double"||type=="float64") sz=8;
            else if (type == "int"||type=="int32") sz=4;
            else if (type == "uchar"||type=="uint8") sz=1;
            else if (type == "uint"||type=="uint32") sz=4;
            else { line=nl+1; continue; }
            if (name=="x") m_hdr.xOff=currentOffset;
            else if (name=="y") m_hdr.yOff=currentOffset;
            else if (name=="z") m_hdr.zOff=currentOffset;
            else if (name=="red")   m_hdr.rOff=currentOffset;
            else if (name=="green") m_hdr.gOff=currentOffset;
            else if (name=="blue")  m_hdr.bOff=currentOffset;
            else if (name=="nx") m_hdr.nxOff=currentOffset;
            else if (name=="ny") m_hdr.nyOff=currentOffset;
            else if (name=="nz") m_hdr.nzOff=currentOffset;
            currentOffset += sz;
        }
        line = nl + 1;
    }

    if (isBinary) {
        m_hdr.stride = currentOffset;
    } else {
        // ASCII: stride is 6 floats (xyz + rgb normalized)
        m_hdr.stride = 24;
        m_hdr.xOff = 0; m_hdr.yOff = 4; m_hdr.zOff = 8;
        m_hdr.rOff = 12; m_hdr.gOff = 16; m_hdr.bOff = 20;
    }
    return m_hdr.vertexCount > 0;
}

// ===================================================================
//  ASCII data parser: mmap text �� float[6] per point
// ===================================================================
bool MmapPlyReader::parseAsciiData(std::string &error) {
    const uint64_t n = m_hdr.vertexCount;
    if (n == 0) { error = "Zero vertices"; return false; }

    m_parsedData.resize(n * 6); // [x,y,z, r,g,b] per point
    const char *p   = reinterpret_cast<const char *>(m_data) + m_hdr.dataOffset;
    const char *end = reinterpret_cast<const char *>(m_data) + m_fileSize;

    uint64_t idx = 0;
    while (p < end && idx < n) {
        // Skip whitespace
        while (p < end && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) ++p;
        if (p >= end) break;

        char *next = nullptr;
        float x = std::strtof(p, &next); if (next == p) break; p = next;
        float y = std::strtof(p, &next); if (next == p) break; p = next;
        float z = std::strtof(p, &next); if (next == p) break; p = next;
        unsigned long r = std::strtoul(p, &next, 10); if (next == p) break; p = next;
        unsigned long g = std::strtoul(p, &next, 10); if (next == p) break; p = next;
        unsigned long b = std::strtoul(p, &next, 10); if (next == p) break; p = next;

        float *dst = m_parsedData.data() + idx * 6;
        dst[0] = x; dst[1] = y; dst[2] = z;
        dst[3] = r / 255.0f; dst[4] = g / 255.0f; dst[5] = b / 255.0f;
        ++idx;
    }

    if (idx == 0) { error = "Parsed 0 points from ASCII data"; return false; }

    // Adjust vertexCount to actually parsed count
    m_hdr.vertexCount = idx;
    m_parsedData.resize(idx * 6);

    // Redirect m_data to parsed array for readPoint compatibility
    m_data = reinterpret_cast<const uint8_t *>(m_parsedData.data());
    m_hdr.dataOffset = 0;
    m_hdr.stride = 24;
    m_hdr.xOff = 0; m_hdr.yOff = 4; m_hdr.zOff = 8;
    m_hdr.rOff = 12; m_hdr.gOff = 16; m_hdr.bOff = 20;

    return true;
}

// ===================================================================
//  readPosition / readPoint
// ===================================================================
void MmapPlyReader::readPosition(uint64_t idx, float *xyz) const {
    const uint8_t *vp = m_data + m_hdr.dataOffset + idx * m_hdr.stride;
    std::memcpy(&xyz[0], vp + m_hdr.xOff, 4);
    std::memcpy(&xyz[1], vp + m_hdr.yOff, 4);
    std::memcpy(&xyz[2], vp + m_hdr.zOff, 4);
}

void MmapPlyReader::readPoint(uint64_t idx, float *xyz, float *rgb) const {
    readPosition(idx, xyz);
    if (m_hdr.hasColor()) {
        const uint8_t *vp = m_data + m_hdr.dataOffset + idx * m_hdr.stride;
        std::memcpy(&rgb[0], vp + m_hdr.rOff, 4);
        std::memcpy(&rgb[1], vp + m_hdr.gOff, 4);
        std::memcpy(&rgb[2], vp + m_hdr.bOff, 4);
    } else {
        rgb[0] = rgb[1] = rgb[2] = 0.85f;
    }
}