#include "bitrl/rl/algorithms/pg/a2c_config.h"
#include "bitsim/utils/io/json_file_reader.h"
#include <boost/log/trivial.hpp>

namespace bitrl
{
namespace rl
{
namespace algos
{
namespace pg
{

std::ostream &A2CConfig::print(std::ostream &out) const
{

    out << "Max its per episode= " << max_itrs_per_episode << std::endl;
    out << "Gamma              = " << gamma << std::endl;
    out << "Clip critic grad   = " << clip_critic_grad << std::endl;
    return out;
}

void A2CConfig::load_from_json(const std::string &filename)
{

    BOOST_LOG_TRIVIAL(info) << "Loading A2CConfig from path: " << filename;

    bitsim::utils::io::JSONFileReader json_reader(filename);
    json_reader.open();

    max_itrs_per_episode = json_reader.template get_value<uint_t>("max_itrs_per_episode");
    gamma = json_reader.template get_value<real_t>("gamma");
    clip_critic_grad = json_reader.template get_value<bool>("clip_critic_grad");
    BOOST_LOG_TRIVIAL(info) << "Done loading A2CConfig";
}

} // namespace pg
} // namespace algos
} // namespace rl
} // namespace bitrl