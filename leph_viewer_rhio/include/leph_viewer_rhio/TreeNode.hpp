#ifndef LEPH_VIEWER_RHIO_TREENODE_HPP
#define LEPH_VIEWER_RHIO_TREENODE_HPP

#include <vector>
#include <string>
#include <leph_viewer_rhio/TreeValue.hpp>

namespace leph {

/**
 * TreeNode
 *
 * Representation of the local information 
 * about a RhIO node in the RhIO tree.
 */
class TreeNode
{
    public:

        /**
         * Initialization.
         *
         * @param path New node absolute path.
         */
        TreeNode(
            const std::string& path);

        /**
         * @return node's name
         */
        const std::string& name() const;

        /**
         * @return absolute node's path
         */
        const std::string& path() const;

        /**
         * @return direct access to node's children
         */
        const std::vector<TreeNode>& children() const;
        std::vector<TreeNode>& children();
        
        /**
         * @return direct access to node's values
         */
        const std::vector<TreeValue>& values() const;
        std::vector<TreeValue>& values();

        /**
         * Read-write direct access to
         * the visualization collapsed  attribute.
         *
         * @return false of the node is expended
         * in the tree visualization.
         */
        const bool& isCollapsed() const;
        bool& isCollapsed();

        /**
         * Add a new child node to the internal
         * container if it does not already exists.
         *
         * @param name The relative node's new name.
         */
        void addChild(const std::string& name);

        /**
         * Add a new value to internal 
         * container if it does not already exists.
         *
         * @param name The relative value's new name. 
         * @param type Type of the created value.
         */
        void addValue(
            const std::string& name,
            TreeValue::ValueType type);
        
        /**
         * Add a new value to internal 
         * container if it does not already exists
         * from its path. 
         * Intermadiate node are created if needed.
         *
         * @param path The relative path of the new 
         * value from this node to be created. 
         * @param type Type of the created value.
         */
        void addValueByPath(
            const std::string& path,
            TreeValue::ValueType type);

    private:

        /**
         * Node name
         */
        std::string _name;

        /**
         * Node full path from root
         */
        std::string _path;

        /**
         * Children nodes
         */
        std::vector<TreeNode> _children;

        /**
         * Contained values
         */
        std::vector<TreeValue> _values;

        /**
         * If false, this node is expended 
         * in the tree visualization
         */
        bool _isCollapsed;
};

}

#endif

