#include <leph_viewer_rhio/WinTree.hpp>
#include <sstream>

namespace leph {

/**
 * Number of spaces per indentation 
 * level in the tree display
 */
static constexpr unsigned int INDENT_SPACES = 4;

WinTree::WinTree(
    int sizeRow, int sizeCol,
    int posRow, int posCol,
    TreeNode& tree) :
    WinBase(sizeRow, sizeCol, posRow, posCol),
    _root(&tree),
    _selectedNode(nullptr),
    _selectedValue(nullptr)
{
}
        
void WinTree::draw()
{
    if (!WinBase::needDrawUpdate()) {
        return;
    }

    //Reset selected node or value
    _selectedNode = nullptr;
    _selectedValue = nullptr;

    //Recursively print
    //the RhIO tree to windo buffer
    printTree(*_root, 0);
    
    //Update window content to screen
    WinBase::draw();
}

void WinTree::printTree(
    TreeNode& node, 
    int level)
{
    if (node.path() != "/") {
        bool isSelected = false;
        if (node.isCollapsed()) {
            isSelected = WinBase::print(
                node.name()+"/\n", 
                A_BOLD | COLOR_PAIR(2),
                (level-1)*INDENT_SPACES);
        } else {
            isSelected = WinBase::print(
                node.name()+"/\n", 
                COLOR_PAIR(2),
                (level-1)*INDENT_SPACES);
        }
        if (isSelected) {
            _selectedNode = &node;
        }
    }
    if (!node.isCollapsed()) {
        for (size_t i=0;i<node.children().size();i++) {
            printTree(node.children()[i], level+1);
        }
        for (size_t i=0;i<node.values().size();i++) {
            bool isSelected = WinBase::print(
                node.values()[i].name()+"\n",
                A_NORMAL | COLOR_PAIR(1),
                level*INDENT_SPACES);
            if (isSelected) {
                _selectedValue = &(node.values()[i]);
            }
        }
    }
}

const TreeNode* WinTree::selectedNode() const
{
    return _selectedNode;
}
TreeNode* WinTree::selectedNode()
{
    return _selectedNode;
}
const TreeValue* WinTree::selectedValue() const
{
    return _selectedValue;
}
TreeValue* WinTree::selectedValue()
{
    return _selectedValue;
}

}

