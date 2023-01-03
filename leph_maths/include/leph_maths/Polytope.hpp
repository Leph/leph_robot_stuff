#ifndef LEPH_MATHS_POLYTOPE_HPP
#define LEPH_MATHS_POLYTOPE_HPP

#include <vector>
#include <Eigen/Dense>

namespace leph {

/**
 * Polytope
 *
 * Generic class for convex polytope
 * in template dimension.
 * Polytope structure is stored in redundant format:
 * vertices and sides and edges data are kept.
 */
template <int dim>
class Polytope
{
    public:

        //Vector typedef in template defined dimension
        typedef Eigen::Matrix<double, dim, 1> Vector;

        //A side of a Polytope
        struct Side {
            //Index of side vertices in counter clock 
            //wise order (OpenGL standard)
            std::vector<size_t> vertexIndexes;
            //Surface normal pointing toward outside
            Vector normal;
        };

        //An edge of a Polytope
        struct Edge {
            //Index of first end vertex
            size_t vertexIndex1;
            //Index of second end vertex
            size_t vertexIndex2;
        };

        /**
         * Empty constructor
         */
        Polytope() :
            _vertices(),
            _sides(),
            _edges()
        {
        }

        /**
         * @return read access to internal 
         * vertices, sides and edges container
         */
        const std::vector<Vector>& getVertices() const
        {
            return _vertices;
        }
        const std::vector<Side>& getSides() const
        {
            return _sides;
        }
        const std::vector<Edge>& getEdges() const
        {
            return _edges;
        }

        /**
         * Apply a translation vector over 
         * all contained vertices.
         *
         * @param vect Vector to add to 
         * all internal vertices.
         */
        void applyTranslation(const Vector& vect)
        {
            for (size_t i=0;i<_vertices.size();i++) {
                _vertices[i] += vect;
            }
        }

        /**
         * Apply a linear transform over 
         * all contained vertices.
         *
         * @param mat Matrix to be pre-multiply 
         * to all internal vertices.
         */
        void applyTranform(const Eigen::Matrix<double, dim, dim>& mat)
        {
            for (size_t i=0;i<_vertices.size();i++) {
                _vertices[i] = mat*_vertices[i];
            }
            for (size_t i=0;i<_sides.size();i++) {
                _sides[i].normal = mat*_sides[i].normal;
            }
        }

        /**
         * Compute if a point is 
         * inside the polytope.
         * 
         * @param point Generic dimensional 
         * point to be checked.
         * @return true if the given point is
         * inside the self polytope.
         */
        bool checkCollision(const Vector& point) const
        {
            bool isInside = true;
            for (size_t i=0;i<_sides.size();i++) {
                size_t index = _sides[i].vertexIndexes.front();
                Vector vect = point - _vertices[index];
                double proj = _sides[i].normal.dot(vect);
                if (proj > 0.0) {
                    isInside = false;
                }
            }

            return isInside;
        }

        //Generating friend functions
        friend Polytope<3> PolytopeGenerateBox(
            const Eigen::Vector3d& center,
            const Eigen::Matrix3d& orientation,
            double sizeX, double sizeY, double sizeZ);
        friend Polytope<3> PolytopeGenerateUnevenTile(
            const Eigen::Vector3d& center,
            const Eigen::Matrix3d& orientation,
            double baseLength, double baseHeight);

    private:

        //Container for polytope vertices
        std::vector<Vector> _vertices;

        //Container for polytope sides
        std::vector<Side> _sides;

        //Container for polytope edges
        std::vector<Edge> _edges;
};

/**
 * Generate a simple 3d box as a Polytope.
 *
 * @param center Vector 3d of box center.
 * @param orientation Box orientation rotation from origin.
 * @param sizeX Half box length of X axis.
 * @param sizeY Half box length of Y axis.
 * @param sizeZ Half box length of Z axis.
 * @return the initialized Polytope.
 */
inline Polytope<3> PolytopeGenerateBox(
    const Eigen::Vector3d& center,
    const Eigen::Matrix3d& orientation,
    double sizeX, double sizeY, double sizeZ)
{
    //Vertices definition
    Eigen::Vector3d vertexLow0(-sizeX, -sizeY, -sizeZ);
    Eigen::Vector3d vertexLow1(sizeX, -sizeY, -sizeZ);
    Eigen::Vector3d vertexLow2(sizeX, sizeY, -sizeZ);
    Eigen::Vector3d vertexLow3(-sizeX, sizeY, -sizeZ);
    Eigen::Vector3d vertexUp0(-sizeX, -sizeY, sizeZ);
    Eigen::Vector3d vertexUp1(sizeX, -sizeY, sizeZ);
    Eigen::Vector3d vertexUp2(sizeX, sizeY, sizeZ);
    Eigen::Vector3d vertexUp3(-sizeX, sizeY, sizeZ);

    //Polytope generation
    Polytope<3> polytope;
    polytope._vertices.push_back(vertexLow0);
    polytope._vertices.push_back(vertexLow1);
    polytope._vertices.push_back(vertexLow2);
    polytope._vertices.push_back(vertexLow3);
    polytope._vertices.push_back(vertexUp0);
    polytope._vertices.push_back(vertexUp1);
    polytope._vertices.push_back(vertexUp2);
    polytope._vertices.push_back(vertexUp3);
    polytope._sides.push_back(
        {{0, 1, 2, 3}, Eigen::Vector3d(0.0, 0.0, -1.0)});
    polytope._sides.push_back(
        {{4, 5, 6, 7}, Eigen::Vector3d(0.0, 0.0, 1.0)});
    polytope._sides.push_back(
        {{0, 1, 5, 4}, Eigen::Vector3d(0.0, -1.0, 0.0)});
    polytope._sides.push_back(
        {{3, 2, 6, 7}, Eigen::Vector3d(0.0, 1.0, 0.0)});
    polytope._sides.push_back(
        {{1, 2, 6, 5}, Eigen::Vector3d(1.0, 0.0, 0.0)});
    polytope._sides.push_back(
        {{3, 0, 4, 7}, Eigen::Vector3d(-1.0, 0.0, 0.0)});
    polytope._edges.push_back({0, 1});
    polytope._edges.push_back({1, 2});
    polytope._edges.push_back({2, 3});
    polytope._edges.push_back({3, 0});
    polytope._edges.push_back({4, 5});
    polytope._edges.push_back({5, 6});
    polytope._edges.push_back({6, 7});
    polytope._edges.push_back({7, 4});
    polytope._edges.push_back({0, 4});
    polytope._edges.push_back({1, 5});
    polytope._edges.push_back({2, 6});
    polytope._edges.push_back({3, 7});
    polytope.applyTranform(orientation);
    polytope.applyTranslation(center);

    return polytope;
}

/**
 * Generate a 3d uneven tile aligned on world axis 
 * whose top surface center have given pose.
 *
 * @param center Top surface center 
 * in world frame.
 * @param orientation Top surface center orientation 
 * rotation from origin (normal along Z axis).
 * @param baseLength Half length (X and Y) of 
 * the aligned squared base.
 * @param baseHeight Height (Z) of the aligned 
 * squared base.
 * @return the initialized Polytope.
 */
inline Polytope<3> PolytopeGenerateUnevenTile(
    const Eigen::Vector3d& center,
    const Eigen::Matrix3d& orientation,
    double baseLength, double baseHeight)
{
    //Tile base center
    Eigen::Vector3d base(center.x(), center.y(), baseHeight);
    //Compute the top surface normal
    Eigen::Vector3d normal = 
        orientation*Eigen::Vector3d(0.0, 0.0, 1.0);

    //Compute vertices height according
    //to plane equation
    double height0 = center.z() -
        (baseLength*normal.x() + baseLength*normal.y())/normal.z();
    double height1 = center.z() -
        (-baseLength*normal.x() + baseLength*normal.y())/normal.z();
    double height2 = center.z() -
        (-baseLength*normal.x() - baseLength*normal.y())/normal.z();
    double height3 = center.z() -
        (baseLength*normal.x() - baseLength*normal.y())/normal.z();

    //Lower vertices
    Eigen::Vector3d vertexLow0 = 
        base + Eigen::Vector3d(baseLength, baseLength, 0.0);
    Eigen::Vector3d vertexLow1 =
        base + Eigen::Vector3d(-baseLength, baseLength, 0.0);
    Eigen::Vector3d vertexLow2 =
        base + Eigen::Vector3d(-baseLength, -baseLength, 0.0);
    Eigen::Vector3d vertexLow3 =
        base + Eigen::Vector3d(baseLength, -baseLength, 0.0);
    //Upper vertices
    Eigen::Vector3d vertexUp0 = 
        vertexLow0 + Eigen::Vector3d(0.0, 0.0, height0);
    Eigen::Vector3d vertexUp1 =
        vertexLow1 + Eigen::Vector3d(0.0, 0.0, height1);
    Eigen::Vector3d vertexUp2 =
        vertexLow2 + Eigen::Vector3d(0.0, 0.0, height2);
    Eigen::Vector3d vertexUp3 =
        vertexLow3 + Eigen::Vector3d(0.0, 0.0, height3);
    
    //Polytope generation
    Polytope<3> polytope;
    polytope._vertices.push_back(vertexLow0);
    polytope._vertices.push_back(vertexLow1);
    polytope._vertices.push_back(vertexLow2);
    polytope._vertices.push_back(vertexLow3);
    polytope._vertices.push_back(vertexUp0);
    polytope._vertices.push_back(vertexUp1);
    polytope._vertices.push_back(vertexUp2);
    polytope._vertices.push_back(vertexUp3);
    polytope._sides.push_back(
        {{0, 1, 2, 3}, Eigen::Vector3d(0.0, 0.0, -1.0)});
    polytope._sides.push_back(
        {{4, 5, 6, 7}, normal});
    polytope._sides.push_back(
        {{0, 4, 5, 1}, Eigen::Vector3d(0.0, 1.0, 0.0)});
    polytope._sides.push_back(
        {{2, 3, 7, 6}, Eigen::Vector3d(0.0, -1.0, 0.0)});
    polytope._sides.push_back(
        {{0, 4, 7, 3}, Eigen::Vector3d(1.0, 0.0, 0.0)});
    polytope._sides.push_back(
        {{1, 5, 6, 2}, Eigen::Vector3d(-1.0, 0.0, 0.0)});
    polytope._edges.push_back({0, 1});
    polytope._edges.push_back({1, 2});
    polytope._edges.push_back({2, 3});
    polytope._edges.push_back({3, 0});
    polytope._edges.push_back({4, 5});
    polytope._edges.push_back({5, 6});
    polytope._edges.push_back({6, 7});
    polytope._edges.push_back({7, 4});
    polytope._edges.push_back({0, 4});
    polytope._edges.push_back({1, 5});
    polytope._edges.push_back({2, 6});
    polytope._edges.push_back({3, 7});
    
    return polytope;
}

/**
 * Explicit template instantiation
 */
extern template class Polytope<2>;
extern template class Polytope<3>;
extern template class Polytope<6>;

}

#endif

