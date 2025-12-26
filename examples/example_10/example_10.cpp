/*
 * In this example we simulate a quadrotor.
 * The quadrotor data is taken from
 * https://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf
 */

#include "bitrl/bitrl_types.h"
#include "bitrl/dynamics/quadrotor_dynamics.h"
#include "bitrl/utils/io/csv_file_writer.h"
#include "bitrl/utils/unit_converter.h"

#include <any>
#include <array>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace example_10
{

using namespace bitrl::dynamics;
using bitrl::real_t;
using bitrl::RealVec;
using bitrl::uint_t;
using bitrl::utils::io::CSVWriter;

RealVec compute_motor_speed(real_t t)
{

    RealVec w(4);

    if (t <= 0.1)
    {

        w[0] = 500 * t + 625.0;
        w[1] = 500 * t + 625.0;
        w[2] = 500 * t + 625.0;
        w[3] = 500 * t + 625.0;

        return w;
    }
    else if (t > 0.1 && t <= 0.4)
    {

        w[0] = -416.66 * t + 716.66;
        w[1] = -416.66 * t + 716.66;
        w[2] = -416.66 * t + 716.66;
        w[3] = -416.66 * t + 716.66;

        return w;
    }
    else if (t > 0.4 && t <= 0.5)
    {

        w[0] = 750.0 * t + 250.0;
        w[1] = 750.0 * t + 250.0;
        w[2] = 750.0 * t + 250.0;
        w[3] = 750.0 * t + 250.0;
        return w;
    }
    else
    {

        w[0] = 625.0;
        w[1] = 625.0;
        w[2] = 625.0;
        w[3] = 625.0;
        return w;
    }
}
} // namespace example_10

int main()
{

    using namespace example_10;

    QuadrotorDynamicsConfig config;
    config.dt = 0.0001;
    config.mass = 0.468;
    config.l = 0.225;
    config.k_1 = 2.980e-6; // this is the k coeff
    config.k_2 = 1.140e-7; // this is the beta coeff
    config.Jx = 4.856e-3;
    config.Jy = 4.856e-3;
    config.Jz = 8.801e-3;

    // initialize the state variables
    std::array<std::pair<std::string, real_t>, 12> values;

    // initial position in NED
    values[0] = std::make_pair("x", 0.0);
    values[1] = std::make_pair("y", 0.0);
    values[2] = std::make_pair("z", 0.0);

    // initial translation velocities body coords
    values[3] = std::make_pair("u", 0.0);
    values[4] = std::make_pair("v", 0.0);
    values[5] = std::make_pair("w", 0.0);

    // initial rotational velocities
    values[6] = std::make_pair("p", 0.0);
    values[7] = std::make_pair("q", 0.0);
    values[8] = std::make_pair("r", 0.0);

    // Euler angles
    values[9] = std::make_pair("phi", 0.0);
    values[10] = std::make_pair("theta", 0.0);
    values[11] = std::make_pair("psi", 0.0);

    SysState<12> state(std::move(values));
    QuadrotorDynamics dynamics(config, state);

    // the motor velocities rad/sec
    RealVec omegas = RealVec::Zero(4);
    omegas[0] = omegas[1] = omegas[2] = omegas[3] = 625.0;

    CSVWriter csv_writer("state");
    csv_writer.open();

    std::vector<std::string> names = {"t",
                                      "x",
                                      "y",
                                      "z",
                                      "u",
                                      "v",
                                      "w",
                                      "p",
                                      "q",
                                      "r",
                                      "phi",
                                      "theta",
                                      "psi"
                                      "w0",
                                      "w1",
                                      "w2",
                                      "w3"};

    csv_writer.write_column_names(names);

    const real_t T = 2.0;
    real_t time = 0.0;

    std::vector<real_t> row(17, 0.0);
    while (time < T)
    {

        std::cout << "Time: " << time << std::endl;

        auto omega_motor = compute_motor_speed(time);
        dynamics.integrate(omega_motor);

        auto p = dynamics.get_position();
        auto v = dynamics.get_velocity();
        auto omega = dynamics.get_angular_velocity();
        auto euler = dynamics.get_euler_angles();

        row[0] = time;
        row[1] = p[0];
        row[2] = p[1];
        row[3] = -p[2];

        row[4] = v[0];
        row[5] = v[1];
        row[6] = v[2];

        row[7] = omega[0];
        row[8] = omega[1];
        row[9] = omega[2];

        row[10] = bitrl::utils::unit_converter::rad_to_degrees(euler[0]);
        row[11] = bitrl::utils::unit_converter::rad_to_degrees(euler[1]);
        row[12] = bitrl::utils::unit_converter::rad_to_degrees(euler[2]);

        row[13] = omega_motor[0];
        row[14] = omega_motor[1];
        row[15] = omega_motor[2];
        row[16] = omega_motor[3];

        csv_writer.write_row(row);
        std::cout << "Current position: ";
        std::cout << p << std::endl;
        time += config.dt;
    }

    csv_writer.close();

    return 0;
}
