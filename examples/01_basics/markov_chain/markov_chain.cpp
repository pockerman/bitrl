#include "bitrl/base/bitrl_types.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <tuple>
#include <utility>

namespace exe
{

using bitrl::DynMat;
using bitrl::DynVec;
using bitrl::real_t;
using bitrl::uint_t;

// create transition matrix
DynMat<real_t> create_transition_matrix() { return DynMat<real_t>({{0.9, 0.1}, {0.5, 0.5}}); }

// create transition matrix
DynMat<real_t> compute_matrix_power(const DynMat<real_t> &mat, uint_t power)
{

    auto result = mat;

    for (uint i = 0; i < power - 1; ++i)
    {
        result *= mat;
    }

    return result;
}

void print_matrix(const DynMat<real_t> &mat)
{
    std::cout << "[" << mat(0, 0) << " , " << mat(0, 1) << "]" << std::endl;
    std::cout << "[" << mat(1, 0) << " , " << mat(1, 1) << "]" << std::endl;
}

} // namespace exe

int main()
{

    using namespace exe;

    auto transition = create_transition_matrix();

    std::cout << "After 3 steps..." << std::endl;
    // after 3 steps
    auto t_3 = compute_matrix_power(transition, 3);
    print_matrix(t_3);

    std::cout << "After 50 steps..." << std::endl;
    // after 3 steps
    auto t_50 = compute_matrix_power(transition, 50);
    print_matrix(t_50);

    std::cout << "After 100 steps..." << std::endl;

    // after 3 steps
    auto t_100 = compute_matrix_power(transition, 100);
    print_matrix(t_100);

    // initial vector
    auto v1 = DynVec<real_t>(2);
    v1[0] = 1.0;
    v1[1] = 0.0;

    // We can calculate the probability of being
    // in a specific state after k iterations multiplying
    // the initial distribution and the transition matrix: v⋅Tk.

    std::cout << "v_3=" << v1.transpose() * t_3 << std::endl;
    std::cout << "v_50=" << v1.transpose() * t_50 << std::endl;
    std::cout << "v_100=" << v1.transpose() * t_100 << std::endl;

    // initial vector
    v1[0] = 0.5;
    v1[1] = 0.5;

    std::cout << "v_3=" << v1.transpose() * t_3 << std::endl;
    std::cout << "v_50=" << v1.transpose() * t_50 << std::endl;
    std::cout << "v_100=" << v1.transpose() * t_100 << std::endl;

    return 0;
}
