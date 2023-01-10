#ifndef LEPH_VIEWER_RHIO_WINTREE_HPP
#define LEPH_VIEWER_RHIO_WINTREE_HPP

#include <leph_viewer_rhio/WinBase.hpp>
#include <leph_viewer_rhio/TreeNode.hpp>
#include <leph_viewer_rhio/TreeValue.hpp>

namespace leph {

/**
 * WinTree
 *
 * NCurses window for displaying and 
 * selecting a RhIO node or value.
 */
class WinTree : public WinBase
{
    public:
        
        /**
         * Initialization with window
         * size and position on screen as
         * well as an Initialized tree.
         *
         * @param sizeRow Window height size.
         * @param sizeCol Window width size.
         * @param posRow Window Y position on global screen.
         * @param posCol Window X position on global screen.
         * @param tree Root node of an initialized 
         * RhIO local tree.
         */
        WinTree(
            int sizeRow, int sizeCol,
            int posRow, int posCol,
            TreeNode& tree);

        /**
         * Update window content.
         * (see @inherited).
         */
        void draw();

        /**
         * @return the pointers of selected
         * tree node or value
         */
        const TreeNode* selectedNode() const;
        TreeNode* selectedNode();
        const TreeValue* selectedValue() const;
        TreeValue* selectedValue();

    private:

        /**
         * Pointer to the local
         * representation of RhIO tree
         */
        TreeNode* _root;

        /**
         * Recursively print the given
         * tree to the window buffer and
         * manage row (node or value) selection.
         *
         * @param node Currently being printed node.
         * @param level Indentation level.
         */
        void printTree(
            TreeNode& node, 
            int level);

        /**
         * If not null, pointer either to the 
         * selected node or the selected value 
         * in the tree
         */
        TreeNode* _selectedNode;
        TreeValue* _selectedValue;
};

}

#endif

