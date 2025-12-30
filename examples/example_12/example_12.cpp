/**
 * This example utilises the ```TensorboardServer``` class to log values of interest when running
 * an experiment. We can monitor the experimet  using <a
 *href="https://www.tensorflow.org/tensorboard">tensorboard</a>. The   ```TensorboardServer``` class
 *is a simple wrapper that exposes three functions
 *
 *	- ```add_scalar```
 *	- ```add_scalars```
 *	- ```add_text```
 *
 *	We will use ```add_scalar``` and ```add_text```. In order to run this example, fire up the
 *server using the ```torchboard_server/start_uvicorn.sh```. The server listens at port 8002. You
 *can change this however you want just make sure that the port is not used and also update the
 *	variable ```TORCH_SERVER_HOST``` in the code below accordingly. Note that the implementation
 *uses <a
 *href="https://pytorch.org/docs/stable/_modules/torch/utils/tensorboard/writer.html#SummaryWriter">SummaryWriter</a>
 *class. Thus you will need to have PyTorch installed on the machine that you run the server.
 */

#include "bitrl/bitrl_types.h"
#include "bitrl/utils/io/json_file_reader.h"
#include "bitrl/network/tensorboard_server.h"

#include <filesystem>
#include <iostream>
#include <random>
#include <string>

namespace example_12
{
using namespace bitrl;
using utils::io::JSONFileReader;
using network::TensorboardServer;

namespace fs = std::filesystem;
const std::string CONFIG = "config.json";

} // namespace example_12

int main()
{

    using namespace example_12;

    try
    {

        // load the json configuration
        JSONFileReader json_reader(CONFIG);
        json_reader.open();

        auto experiment_dict = json_reader.template get_value<std::string>("experiment_dict");
        auto experiment_id = json_reader.template get_value<std::string>("experiment_id");

        std::cout << "Experiment directory: " << experiment_dict << std::endl;
        std::cout << "Experiment id: " << experiment_id << std::endl << std::endl;

        const fs::path EXPERIMENT_DIR_PATH = experiment_dict + experiment_id;

        // the first thing we want to do when monitoring experiments
        // is to create a directory where all data will reside
        std::filesystem::create_directories(experiment_dict + experiment_id);

        const auto input_size = json_reader.template get_value<uint_t>("input_size");
        const auto output_size = json_reader.template get_value<uint_t>("output_size");
        const auto num_epochs = json_reader.template get_value<uint_t>("num_epochs");
        const auto learning_rate = json_reader.template get_value<real_t>("lr");
        auto log_path = json_reader.template get_value<std::string>("log_path");

        // log the hyperparameters
        std::cout << "Input size: " << input_size << std::endl;
        std::cout << "Output size: " << output_size << std::endl;
        std::cout << "Max epochs: " << num_epochs << std::endl;
        std::cout << "Learning rate: " << learning_rate << std::endl;

        TensorboardServer logger("http://0.0.0.0:8002");

        std::cout << "Logging results at " << logger.get_log_dir_path() << std::endl;
        logger.init(log_path);

        logger.add_scalar("lr", learning_rate);
        logger.add_scalar("seed", 42);
        logger.add_scalar("num_epochs", num_epochs);
        logger.add_text("optimizer", "torch::optim::SGD");
        logger.close();
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
