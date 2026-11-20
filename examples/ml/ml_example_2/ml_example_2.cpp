/**
 * Example 22:
 *
 */

#include "cubeai/base/cubeai_config.h"
#include "cubeai/base/cubeai_consts.h"
#include "cubeai/base/cubeai_types.h"
#include "cubeai/datasets/iris_data_set.h"
#include "cubeai/io/data_set_loaders.h"
#include "cubeai/maths/lp_metric.h"
#include "cubeai/maths/matrix_utilities.h"
#include "cubeai/ml/classifiers/k_nearest_neighbors.h"

#include <iostream>

namespace ml_example_2
{

using cubeai::DynVec;
using cubeai::real_t;
using cubeai::uint_t;
using cubeai::maths::LpMetric;
using cubeai::ml::classifiers::KNearestNeighbors;

class LpMetricWrapper
{
  public:
    typedef real_t value_type;

    LpMetricWrapper() = default;

    template <typename DataPair> real_t evaluate(const DataPair &v1, const DataPair &v2) const
    {
        return LpMetric<2>::evaluate(v1.first, v2.first);
    }
};

} // namespace ml_example_2

int main()
{

    using namespace ml_example_2;

    try
    {

        cubeai::datasets::IrisDataSet data;

        std::cout << data << std::endl;

        KNearestNeighbors<cubeai::datasets::IrisDataSet::point_type> classifier(data.n_columns());

        auto comparison = [](const auto &v1, const auto &v2)
        {
            return v1.first[0] == v2.first[0] && v1.first[1] == v2.first[1] &&
                   v1.first[2] == v2.first[2] && v1.first[3] == v2.first[3];
        };

        auto info = classifier.fit(data, comparison);
        std::cout << info << std::endl;

        auto row = data[0];
        std::cout << "True class=" << row.second << "->" << data.get_class_name(row.second)
                  << std::endl;
        auto index = classifier.template predict<LpMetricWrapper>(row.first, 5);
        std::cout << "Predicted class=" << row.second << "->" << data.get_class_name(index)
                  << std::endl;

        auto closest_points = classifier.template nearest_k_points<LpMetricWrapper>(row.first, 5);

        std::cout << cubeai::CubeAIConsts::info_str() << " Query point is " << row << std::endl;
        for (auto &p : closest_points)
        {
            std::cout << "Distance=" << p.first << ", " << p.second << std::endl;
        }
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
