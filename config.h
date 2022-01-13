#ifndef CONFIG_H
#define CONFIG_

#include <memory>

#include "common.h"
#include <opencv2/core.hpp>
namespace vo {
    
class Config {
private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config() {}
public:
    ~Config();

    static bool SetParameterFile(const std::string& filename);

    template<class T>
    static T Get(const std::string& key) {
        return T(Config::config_->file_[key]);
    }

};

}

#endif // CONFIG_H