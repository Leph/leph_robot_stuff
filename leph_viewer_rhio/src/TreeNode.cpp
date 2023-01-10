#include <leph_viewer_rhio/TreeNode.hpp>

namespace leph {

TreeNode::TreeNode(
    const std::string& path) :
    _name(),
    _path(path),
    _children(),
    _values(),
    _isCollapsed(true)
{
    size_t index = _path.find_last_of("/");
    if (index != std::string::npos) {
        _name = _path.substr(index+1);
    } else {
        _name = _path;
        _path = "/" + _name;
    }
    if (_path == "/") {
        _isCollapsed = false;
    }
}

const std::string& TreeNode::name() const
{
    return _name;
}
        
const std::string& TreeNode::path() const
{
    return _path;
}

const std::vector<TreeNode>& TreeNode::children() const
{
    return _children;
}
std::vector<TreeNode>& TreeNode::children()
{
    return _children;
}

const std::vector<TreeValue>& TreeNode::values() const
{
    return _values;
}
std::vector<TreeValue>& TreeNode::values()
{
    return _values;
}
        
const bool& TreeNode::isCollapsed() const
{
    return _isCollapsed;
}
bool& TreeNode::isCollapsed()
{
    return _isCollapsed;
}
        
void TreeNode::addChild(const std::string& name)
{
    bool isFound = false;
    for (size_t i=0;i<_children.size();i++) {
        if (name == _children[i].name()) {
            isFound = true;
        }
    }
    if (!isFound) {
        if (_name.length() > 0) {
            _children.push_back(TreeNode(
                _path + "/" + name));
        } else {
            _children.push_back(TreeNode(
                "/" + name));
        }
    }
}

void TreeNode::addValue(
    const std::string& name,
    TreeValue::ValueType type)
{
    bool isFound = false;
    for (size_t i=0;i<_values.size();i++) {
        if (name == _values[i].name()) {
            isFound = true;
        }
    }
    if (!isFound) {
        if (_name.length() > 0) {
            _values.push_back(TreeValue(
                _path + "/" + name, type));
        } else {
            _values.push_back(TreeValue(
                "/" + name, type));
        }
    }
}

void TreeNode::addValueByPath(
    const std::string& path,
    TreeValue::ValueType type)
{
    if (path.size() == 0) {
        return;
    }
    size_t index = path.find_first_of("/");

    //Terminal case
    if (index == std::string::npos) {
        addValue(path, type);
        return;
    } 

    //Root slash prefix case
    if (index == 0) {
        addValueByPath(
            path.substr(1), type);
        return;
    }

    //Recursive case
    std::string nodeName = path.substr(0, index);
    addChild(nodeName);
    for (size_t i=0;i<_children.size();i++) {
        if (_children[i].name() == nodeName) {
            _children[i].addValueByPath(
                path.substr(index+1), type);
            return;
        }
    }
}

}

