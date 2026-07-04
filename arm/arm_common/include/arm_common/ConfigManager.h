#pragma once

#include <string>
#include <vector>
#include <optional>
#include <stdexcept>
#include <sstream>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <kdl/frames.hpp>

// ---------------------------------------------------------------------------
// ConfigManager — singleton that loads arm_bringup config and manip.yaml.
//
// Usage:
//   ConfigManager::getInstance()->loadConfig();
//   const auto& cfg = ConfigManager::getInstance()->getConfig();
//
// Path resolution (all derived from the arm_bringup package share dir):
//   config.yaml   →  <share>/config/config.yaml
//   manip.yaml    →  <share>/manipulators/<Manipulator.name>/manip.yaml
// ---------------------------------------------------------------------------
class ConfigManager
{
public:
    // -----------------------------------------------------------------------
    // Sub-structs — mirror the YAML hierarchy; used for per-subsystem access
    // -----------------------------------------------------------------------

    struct VehicleConfig
    {
        std::string              urdfPath;        // resolved absolute path
        std::string              manipAttachLink; // link on vehicle where arm attaches
        std::vector<std::string> jointNames;
        std::vector<double>      initialPositions;
        std::vector<double>      accelLimits;
        std::vector<double>      jerkLimits;
    };

    struct ManipulatorConfig
    {
        std::string              urdfPath;   // resolved absolute path
        std::string              baseLink;
        std::string              endLink;
        std::vector<std::string> jointNames;
        std::vector<double>      initialPositions;
        std::vector<double>      accelLimits;
        std::vector<double>      jerkLimits;

        // Named joint-space presets (optional, arm joints only)
        std::optional<std::vector<double>> deployPos;
        std::optional<std::vector<double>> stowPos;
    };

    struct Config
    {
        // Top-level
        std::string manipType;      // e.g. "dynamixel"
        std::string commsMode;      // "simulated" | "hardware"
        int         controlRateHz = 10;

        // Per-subsystem configs — use when addressing vehicle or arm
        // joints independently (e.g. IK solvers, named presets)
        VehicleConfig     vehicle;
        ManipulatorConfig manipulator;

        // Combined flat vectors — ordered [vehicle joints..., arm joints...]
        // Use for whole-body planners, limit arrays, joint state indexing, etc.
        std::vector<std::string> jointNames;
        std::vector<double>      initialPositions;
        std::vector<double>      accelLimits;
        std::vector<double>      jerkLimits;

        // Transform: vehicle frame -> manipulator base frame
        KDL::Frame T_V_B;
    };

    // -----------------------------------------------------------------------
    // Singleton access
    // -----------------------------------------------------------------------
    static ConfigManager* getInstance()
    {
        static ConfigManager instance;
        return &instance;
    }

    // -----------------------------------------------------------------------
    // Resolve paths from arm_bringup share dir, load config.yaml and the
    // manip.yaml it references. Throws std::runtime_error on any problem.
    // -----------------------------------------------------------------------
    void loadConfig();

    const Config&     getConfig()  const { return mConfig; }
    const YAML::Node& getRawNode() const { return mManipRoot; }  // manip.yaml root

    // Generic nested accessor into manip.yaml: getValue<T>("manipulator.base_link")
    template<typename T>
    T getValue(const std::string& dotPath) const
    {
        YAML::Node current = YAML::Clone(mManipRoot);
        for (const auto& key : splitPath(dotPath))
        {
            if (!current[key])
                throw std::runtime_error("[ConfigManager] YAML path not found: " + dotPath);
            current = current[key];
        }
        return current.as<T>();
    }

private:
    ConfigManager() = default;
    ~ConfigManager() = default;
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    static YAML::Node require(const YAML::Node& parent,
                           const std::string& key,
                           const std::string& context)
    {
        if (!parent[key] || parent[key].IsNull())
            throw std::runtime_error(
                "[ConfigManager] Missing required key '" + key +
                "' in section '" + context + "'");
        return parent[key];
    }

    // Read a required scalar.
    template<typename T>
    static T requireScalar(const YAML::Node& parent,
                           const std::string& key,
                           const std::string& context)
    {
        return require(parent, key, context).as<T>();
    }

    // Read an optional scalar with a default.
    template<typename T>
    static T optionalScalar(const YAML::Node& parent,
                            const std::string& key,
                            T defaultVal)
    {
        if (parent[key] && !parent[key].IsNull())
            return parent[key].as<T>();
        return defaultVal;
    }

    // Read a required sequence, validating length against expectedSize if nonzero.
    template<typename T>
    static std::vector<T> requireSeq(const YAML::Node& parent,
                                     const std::string& key,
                                     const std::string& context,
                                     std::size_t expectedSize = 0)
    {
        const YAML::Node& node = require(parent, key, context);
        if (!node.IsSequence())
            throw std::runtime_error(
                "[ConfigManager] Key '" + key + "' in '" + context +
                "' must be a sequence");
        auto vec = node.as<std::vector<T>>();
        if (expectedSize > 0 && vec.size() != expectedSize)
            throw std::runtime_error(
                "[ConfigManager] Key '" + key + "' in '" + context +
                "' has " + std::to_string(vec.size()) +
                " elements, expected " + std::to_string(expectedSize));
        return vec;
    }

    // Read an optional sequence (returns nullopt if absent).
    template<typename T>
    static std::optional<std::vector<T>> optionalSeq(const YAML::Node& parent,
                                                      const std::string& key,
                                                      std::size_t expectedSize = 0)
    {
        if (!parent[key] || parent[key].IsNull())
            return std::nullopt;
        auto vec = parent[key].as<std::vector<T>>();
        if (expectedSize > 0 && vec.size() != expectedSize)
            throw std::runtime_error(
                "[ConfigManager] Optional key '" + key +
                "' has " + std::to_string(vec.size()) +
                " elements, expected " + std::to_string(expectedSize));
        return vec;
    }

    // Append src onto dst in-place.
    template<typename T>
    static void append(std::vector<T>& dst, const std::vector<T>& src)
    {
        dst.insert(dst.end(), src.begin(), src.end());
    }

    // Resolve a path relative to a base directory.
    static std::string  resolvePath(const std::filesystem::path& base,
                                   const std::string& relative)
    {
        return (base / relative).lexically_normal().string();
    }

    static std::vector<std::string> splitPath(const std::string& path)
    {
        std::vector<std::string> out;
        std::stringstream ss(path);
        std::string item;
        while (std::getline(ss, item, '.'))
            if (!item.empty()) out.push_back(item);
        return out;
    }

    // -----------------------------------------------------------------------
    // Section loaders (all operate on mManipRoot)
    // -----------------------------------------------------------------------
    void loadVehicleSection    (const std::filesystem::path& manipDir);
    void loadManipulatorSection(const std::filesystem::path& manipDir);
    void loadMountSection      ();
    void mergeJointData        ();

    // -----------------------------------------------------------------------
    // State
    // -----------------------------------------------------------------------
    Config     mConfig;
    YAML::Node mManipRoot;  // root of the active manip.yaml
};