#include <leph_viewer_rhio/TreeValue.hpp>

namespace leph {

TreeValue::TreeValue(
    const std::string& path,
    ValueType type) :
    _name(),
    _path(path),
    _type(type)
{
    size_t index = _path.find_last_of("/");
    if (index != std::string::npos) {
        _name = _path.substr(index+1);
    } else {
        _name = _path;
        _path = "/" + _name;
    }
}

const std::string& TreeValue::name() const
{
    return _name;
}
        
const std::string& TreeValue::path() const
{
    return _path;
}

std::string TreeValue::typeAsStr() const
{
    if (_type == ValueType::BOOL) {
        return "bool";
    }
    if (_type == ValueType::INT) {
        return "int";
    }
    if (_type == ValueType::FLOAT) {
        return "float";
    }
    return "";
}
        
}

