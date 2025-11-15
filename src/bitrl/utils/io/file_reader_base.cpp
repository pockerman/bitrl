#include "bitrl/utils/io/file_reader_base.h"
#include "bitrl/rlenvscpp_config.h"

namespace bitrl{
namespace utils{
namespace io{

FileReaderBase::FileReaderBase(const std::string& file_name, FileFormats::Type t)
:
FileHandlerBase<std::ifstream>(file_name, t)
{}


void
FileReaderBase::open(){

    auto& f = this->get_file_stream();

     if(!f.is_open()){

        try{
            f.open(this->get_filename(), std::ios_base::in);

#ifdef BITRL_DEBUG

            if(!f.good()){
                auto file_name = this->get_filename();
                std::string msg = "Failed to open file: " + file_name;
                assert(false && msg.c_str());
            }
#endif

        }
        catch(...){

#ifdef BITRL_DEBUG
            std::string msg("Failed to open file: ");
            auto file_name = this->get_filename();
            msg += file_name;
            assert(false && msg.c_str());
#else
            throw;
#endif

        }
    }
}

}
}
}
