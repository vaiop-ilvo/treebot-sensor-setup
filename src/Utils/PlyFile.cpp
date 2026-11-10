#include <Utils/PlyFile.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>

namespace {
    size_t typeSize(const std::string& type)
    {
        if (type == "char" || type == "uchar" || type == "int8" || type == "uint8") return 1;
        if (type == "short" || type == "ushort" || type == "int16" || type == "uint16") return 2;
        if (type == "int" || type == "uint" || type == "float" || type == "int32" || type == "uint32" || type == "float32") return 4;
        if (type == "double" || type == "float64") return 8;
        return 0;
    }

    double readValue(const char* p, const std::string& type)
    {
        auto get = [p](auto value) { std::memcpy(&value, p, sizeof(value)); return static_cast<double>(value); };
        if (type == "char" || type == "int8") return get(int8_t{});
        if (type == "uchar" || type == "uint8") return get(uint8_t{});
        if (type == "short" || type == "int16") return get(int16_t{});
        if (type == "ushort" || type == "uint16") return get(uint16_t{});
        if (type == "int" || type == "int32") return get(int32_t{});
        if (type == "uint" || type == "uint32") return get(uint32_t{});
        if (type == "float" || type == "float32") return get(float{});
        return get(double{});
    }

    struct Property {
        std::string name;
        std::string type;
        size_t offset = 0;
    };
}

bool writePly(const std::string& fileName, const std::vector<vaiop::QtPoint>& cloud, bool withColor)
{
    std::ofstream out(fileName, std::ios::binary);
    if (!out) {
        return false;
    }
    out << "ply\nformat binary_little_endian 1.0\nelement vertex " << cloud.size()
        << "\nproperty float x\nproperty float y\nproperty float z\n";
    if (withColor) {
        out << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    }
    out << "end_header\n";

    const size_t step = withColor ? 15 : 12;
    std::vector<char> data(cloud.size() * step);
    for (size_t i = 0; i < cloud.size(); ++i) {
        char* p = data.data() + i * step;
        const float xyz[3] = {cloud[i].position.x(), cloud[i].position.y(), cloud[i].position.z()};
        std::memcpy(p, xyz, sizeof(xyz));
        if (withColor) {
            for (int c = 0; c < 3; ++c) {
                p[12 + c] = static_cast<char>(static_cast<uint8_t>(std::clamp(cloud[i].color[c], 0.0f, 1.0f) * 255.0f + 0.5f));
            }
        }
    }
    out.write(data.data(), static_cast<std::streamsize>(data.size()));
    return static_cast<bool>(out);
}

std::shared_ptr<std::vector<vaiop::QtPoint>> readPly(const std::string& fileName)
{
    std::ifstream in(fileName, std::ios::binary);
    std::string line;
    if (!in || !std::getline(in, line) || line.rfind("ply", 0) != 0) {
        return nullptr;
    }

    // Header: only the vertex element is read; it must be the first element (as in PCL's files)
    bool binaryLittleEndian = false;
    bool inVertex = false;
    bool vertexSeen = false;
    size_t vertexCount = 0;
    size_t step = 0;
    std::vector<Property> properties;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        std::istringstream words(line);
        std::string keyword;
        words >> keyword;
        if (keyword == "format") {
            std::string format;
            words >> format;
            binaryLittleEndian = format == "binary_little_endian";
        } else if (keyword == "element") {
            std::string name;
            size_t count = 0;
            words >> name >> count;
            inVertex = name == "vertex" && !vertexSeen;
            if (inVertex) {
                vertexSeen = true;
                vertexCount = count;
            } else if (!vertexSeen) {
                return nullptr;     // Another element before the vertices
            }
        } else if (keyword == "property" && inVertex) {
            Property property;
            words >> property.type >> property.name;
            const size_t size = typeSize(property.type);
            if (size == 0) {
                return nullptr;     // List properties are not supported in the vertices
            }
            property.offset = step;
            step += size;
            properties.push_back(property);
        } else if (keyword == "end_header") {
            break;
        }
    }
    auto find = [&properties](const char* name) -> const Property* {
        for (const Property& p : properties) {
            if (p.name == name) return &p;
        }
        return nullptr;
    };
    const Property* x = find("x");
    const Property* y = find("y");
    const Property* z = find("z");
    const Property* red = find("red");
    const Property* green = find("green");
    const Property* blue = find("blue");
    if (!binaryLittleEndian || !x || !y || !z || step == 0) {
        return nullptr;
    }

    std::vector<char> data(vertexCount * step);
    if (!in.read(data.data(), static_cast<std::streamsize>(data.size()))) {
        return nullptr;
    }
    auto cloud = std::make_shared<std::vector<vaiop::QtPoint>>(vertexCount);
    for (size_t i = 0; i < vertexCount; ++i) {
        const char* p = data.data() + i * step;
        vaiop::QtPoint& point = (*cloud)[i];
        point.position = QVector3D(static_cast<float>(readValue(p + x->offset, x->type)),
                                   static_cast<float>(readValue(p + y->offset, y->type)),
                                   static_cast<float>(readValue(p + z->offset, z->type)));
        if (red && green && blue) {
            point.color = QVector3D(static_cast<float>(readValue(p + red->offset, red->type) / 255.0),
                                    static_cast<float>(readValue(p + green->offset, green->type) / 255.0),
                                    static_cast<float>(readValue(p + blue->offset, blue->type) / 255.0));
        }
    }
    return cloud;
}
