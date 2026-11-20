/**
  * This example illustrates a simple example of Monte Carlo
  * iteration using the IterationCounter class
  *
  * */

#include "bitrl/base/bitrl_config.h"
#include "bitrl/base/bitrl_types.h"
#include "bitsim/utils/io/json_file_reader.h"
#include "bitsim/utils/iteration_counter.h"
#include "bitsim/geometry/shapes/circle.h"

#include <boost/log/trivial.hpp>
#include <iostream>
#include <random>
#include <fstream>

namespace intro_example_1
{

using bitrl::real_t;
using bitrl::uint_t;
using bitsim::utils::IterationCounter;
using bitsim::geometry::Circle;
using bitsim::utils::io::JSONFileReader;

const std::string CONFIG = "config.json";

}

int main() {

    using namespace intro_example_1;

    try{
		
		BOOST_LOG_TRIVIAL(info)<<"Running example...";
        BOOST_LOG_TRIVIAL(info)<<"Reading configuration file...";

        JSONFileReader json_reader(CONFIG);
        json_reader.open();

        const auto R = json_reader.template get_value<real_t>("R");
        const auto N_POINTS =  json_reader.template get_value<uint_t>("N_POINTS");
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
        const real_t SQUARE_SIDE = R*2.0;
        std::uniform_real_distribution dist(0.0,SQUARE_SIDE);
        std::mt19937 gen(SEED);

        BOOST_LOG_TRIVIAL(info)<<"Starting computation...";
        while(counter.continue_iterations()){
            auto x = dist(gen);
            auto y = dist(gen);
            if(c.is_inside(x,y, 1.0e-4)){
              points_inside_circle += 1;
            }
        }

        BOOST_LOG_TRIVIAL(info)<<"Finished computation...";
        auto area = (static_cast<real_t>(points_inside_circle) / static_cast<real_t>(N_POINTS)) * std::pow(SQUARE_SIDE, 2);
        BOOST_LOG_TRIVIAL(info)<<"Circle area calculated with:" <<N_POINTS<<" is: "<<area;
        BOOST_LOG_TRIVIAL(info)<<"Circle area: "<<c.area();
    }
    catch(std::exception& e){
        BOOST_LOG_TRIVIAL(error)<<e.what();
    }
    catch(...){
        BOOST_LOG_TRIVIAL(error)<<"Unknown exception occured";
    }

   return 0;
}


