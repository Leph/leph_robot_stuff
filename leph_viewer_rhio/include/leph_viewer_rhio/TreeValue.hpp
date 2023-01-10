#ifndef LEPH_VIEWER_RHIO_TREEVALUE_HPP
#define LEPH_VIEWER_RHIO_TREEVALUE_HPP

#include <vector>
#include <string>

namespace leph {

/**
 * TreeValue
 *
 * Representation of the local information 
 * about a RhIO value inside a RhIO tree.
 */
class TreeValue
{
    public:
        
        /**
         * Internal value type
         */
        enum class ValueType {
            BOOL,
            INT,
            FLOAT,
        };
        
        /**
         * Initialization.
         *
         * @param path New value absolute path.
         * @path type Its internal type.
         */
        TreeValue(
            const std::string& path,
            ValueType type);

        /**
         * @return value's name
         */
        const std::string& name() const;

        /**
         * @return absolute value's path
         */
        const std::string& path() const;

        /**
         * @return the value type as a string.
         */
        std::string typeAsStr() const;

    private:

        /**
         * Value name
         */
        std::string _name;

        /**
         * Value full path from root
         */
        std::string _path;

        /**
         * Value type
         */
        ValueType _type;
};

}

#endif

