#include "bitrl/bitrl_types.h"
#include "bitrl/utils/geometry/geom_point.h"
#include "bitrl/utils/geometry/mesh/mesh.h"
#include "bitrl/utils/geometry/mesh/quad_mesh_generation.h"

#include <filesystem>
#include <iostream>
#include <random>
#include <string>

namespace example_13
{
using namespace bitrl;
using utils::geom::GeomPoint;
using utils::geom::Mesh;

} // namespace example_13

int main()
{

    using namespace example_13;

    try
    {

        // build a square grid
        Mesh<2> mesh;
        utils::geom::build_quad_mesh(mesh, 10, 10, GeomPoint<2>({0.0, 0.0}),
                                     GeomPoint<2>({1.0, 1.0}));

        std::cout << "Number of elements: " << mesh.n_elements() << std::endl;
        std::cout << "Number of nodes:    " << mesh.n_nodes() << std::endl;
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }
    catch (...)
    {

        std::cout << "Unknown exception occured" << std::endl;
    }

    return 0;
}
