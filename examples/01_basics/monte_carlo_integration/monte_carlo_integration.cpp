/**
 * This example illustrates a simple example of Monte Carlo
 * iteration using the IterationCounter class
 *
 * */

#include "bitrl/base/bitrl_config.h"
#include "bitrl/base/bitrl_log.h"
#include "bitrl/base/bitrl_types.h"
#include "bitsim/geometry/shapes/circle.h"
#include "bitsim/utils/io/json_file_reader.h"
#include "bitsim/utils/iteration_counter.h"

#include <fstream>
#include <iostream>
#include <random>

namespace intro_example_1
{

using bitrl::real_t;
using bitrl::uint_t;
using bitsim::geometry::Circle;
using bitsim::utils::IterationCounter;
using bitsim::utils::io::JSONFileReader;

const std::string CONFIG = "config.json";

} // namespace intro_example_1

int main()
{

    using namespace intro_example_1;

    try
    {

        BITRL_LOG_INFO << "Running example...";
        BITRL_LOG_INFO << "Reading configuration file...";

        JSONFileReader json_reader(CONFIG);
        json_reader.open();

        const auto R = json_reader.template get_value<real_t>("R");
        const auto N_POINTS = json_reader.template get_value<uint_t>("N_POINTS");
        const auto SEED = json_reader.template get_value<uint_t>("SEED");
        const auto X = json_reader.template get_value<real_t>("X");
        const auto Y = json_reader.template get_value<real_t>("Y");

        // create a circle
        Circle c(R, {X, Y});

        // simple object to control iterations
        IterationCounter counter(N_POINTS);

        // how many points we found in the Circle
        auto points_inside_circle = 0;

        // the box has side 2
        const real_t SQUARE_SIDE = R * 2.0;
        std::uniform_real_distribution dist(0.0, SQUARE_SIDE);
        std::mt19937 gen(SEED);

        BITRL_LOG_INFO << "Starting computation...";
        while (counter.continue_iterations())
        {
            auto x = dist(gen);
            auto y = dist(gen);
            if (c.is_inside(x, y, 1.0e-4))
            {
                points_inside_circle += 1;
            }
        }

        BITRL_LOG_INFO << "Finished computation...";
        auto area = (static_cast<real_t>(points_inside_circle) / static_cast<real_t>(N_POINTS)) *
                    std::pow(SQUARE_SIDE, 2);
        BITRL_LOG_INFO << "Circle area calculated with:" << N_POINTS << " is: " << area;
        BITRL_LOG_INFO << "Circle area: " << c.area();
    }
    catch (std::exception &e)
    {
        BITRL_LOG_ERROR << e.what();
    }
    catch (...)
    {
        BITRL_LOG_ERROR << "Unknown exception occured";
    }

    return 0;
}
