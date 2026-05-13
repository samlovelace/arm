#ifndef CONFIGMANAGER_H
#define CONFIGMANAGER_H

#include <string> 
#include <vector>
#include <optional>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <kdl/frames.hpp>

class ConfigManager
{
public:
    
    static ConfigManager* getInstance()
    {
        static ConfigManager mInstance;  
        return &mInstance; 
    }

    struct Config   
    {
        std::string shareDir; 
        std::string manipType;
        std::string manipCommsType;  
        std::string urdfPath; 
        std::string inverseReachMap = ""; 
        int manipControlRate; 
        std::vector<double> initialPosition; 
        std::vector<double> accelLimit; 
        std::vector<double> jerkLimit; 
        double velLimitFraction; 
        KDL::Frame T_V_B; 
        std::string robotUrdfPath = "";
    };

    bool load(const std::string& filename) 
    {
        try {
            mRoot = YAML::LoadFile(filename);
            return true;
        } catch (const YAML::Exception& e) {
            std::cerr << "Error loading config: " << e.what() << std::endl;
            return false;
        }

        //std::cout << YAML::Dump(mRoot); 
    }

    // -------------------------------------------------------------
    // getValue<T>(rootNode, "A.B.C") — templated nested accessor
    // -------------------------------------------------------------
    template<typename T>
    T getValue(const std::string& fullPath) 
    {
        auto keys = splitPath(fullPath);

        YAML::Node current = YAML::Clone(mRoot); // clone to avoid changing mRoot

        for (const auto& key : keys) {
            if (!current[key]) {
                throw std::runtime_error("YAML path not found: " + fullPath);
            }
            current = current[key];
        }

        return Converter<T>::convert(current);
    }

    void loadConfig(const std::string& aConfigFilepath); 
    Config getConfig() {return mConfig; }
    YAML::Node& getRawConfig() {return mYamlConfig; }
    YAML::Node& getManipConfig() {return mManipConfig; }

private:
    ConfigManager(/* args */) {}
    ~ConfigManager() {}

    ConfigManager* mInstance; 

    Config mConfig; 
    YAML::Node mYamlConfig; 
    YAML::Node mManipConfig; 

    inline std::vector<std::string> splitPath(const std::string& path) 
    {
        std::vector<std::string> out;
        std::stringstream ss(path);
        std::string item;

        while (std::getline(ss, item, '.')) {
            if (!item.empty())
                out.push_back(item);
        }
        return out;
    }

    // -------------------------------------------------------------
    // Template specialization for std::vector<T>
    // Allows reading YAML sequences into std::vector<T>
    // -------------------------------------------------------------
    template<typename T>
    struct Converter {
        static T convert(const YAML::Node& node) {
            return node.as<T>();
        }
    };

    template<typename T>
    struct Converter<std::vector<T>> {
        static std::vector<T> convert(const YAML::Node& node) {
            std::vector<T> vec;
            for (const auto& n : node) {
                vec.push_back(n.as<T>());
            }
            return vec;
        }
    };

    YAML::Node mRoot;

};
#endif 
