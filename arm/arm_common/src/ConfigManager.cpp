#include "arm_common/ConfigManager.h"
#include "plog/Log.h"
#include <stdexcept>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

void ConfigManager::loadConfig()
{
    // Locate the arm_bringup share directory via ament
    const fs::path shareDir = []{
        try {
            return fs::path(ament_index_cpp::get_package_share_directory("arm_bringup"));
        } catch (const std::exception& e) {
            throw std::runtime_error(
                std::string("[ConfigManager] Could not locate arm_bringup package: ") + e.what());
        }
    }();

    // Load top-level config.yaml to find which manipulator to use
    const fs::path configPath = shareDir / "config" / "config.yaml";
    if (!fs::exists(configPath))
        throw std::runtime_error(
            "[ConfigManager] Top-level config not found: " + configPath.string());

    LOGI << "Loading config: " << configPath.string();
    const YAML::Node topConfig = YAML::LoadFile(configPath.string());

    const YAML::Node manipNode = require(topConfig, "Manipulator", "config.yaml");
    const std::string manipName = requireScalar<std::string>(manipNode, "name", "Manipulator");

    // Resolve manip.yaml from the named manipulator subdirectory
    const fs::path manipDir     = shareDir / "config" / "manipulators" / manipName;
    const fs::path manipYamlPath = manipDir / "manip.yaml";
    if (!fs::exists(manipYamlPath))
        throw std::runtime_error(
            "[ConfigManager] manip.yaml not found for manipulator '" + manipName +
            "': " + manipYamlPath.string());

    LOGI << "Loading manipulator config: " << manipYamlPath.string();
    mManipRoot = YAML::LoadFile(manipYamlPath.string());

    // Top-level manip.yaml fields
    mConfig.manipType     = requireScalar<std::string>(mManipRoot, "type",  "manip.yaml");
    mConfig.commsMode     = requireScalar<std::string>(mManipRoot, "comms", "manip.yaml");
    mConfig.controlRateHz = optionalScalar<int>(mManipRoot, "rate", 10);

    loadVehicleSection    (manipDir);
    loadManipulatorSection(manipDir);
    loadMountSection      ();
    mergeJointData        ();

    LOGI << "Config loaded successfully — manipulator: '" << manipName
         << "', " << mConfig.jointNames.size() << " total joints ("
         << mConfig.vehicle.jointNames.size()     << " vehicle, "
         << mConfig.manipulator.jointNames.size()  << " arm)";
    LOGD << YAML::Dump(mManipRoot);
}

// ---------------------------------------------------------------------------
// vehicle:
// ---------------------------------------------------------------------------
void ConfigManager::loadVehicleSection(const fs::path& manipDir)
{
    const std::string ctx = "vehicle";
    const YAML::Node& v = require(mManipRoot, "vehicle", ctx);

    mConfig.vehicle.urdfPath        = resolvePath(manipDir, requireScalar<std::string>(v, "urdf", ctx));
    mConfig.vehicle.manipAttachLink = requireScalar<std::string>(v, "manip_attach_link", ctx);

    auto jointNames = requireSeq<std::string>(v, "joint_names", ctx);
    const std::size_t n = jointNames.size();
    mConfig.vehicle.jointNames = std::move(jointNames);

    mConfig.vehicle.initialPositions = requireSeq<double>(v, "initial_positions", ctx, n);
    mConfig.vehicle.accelLimits      = requireSeq<double>(v, "accel_limits",      ctx, n);
    mConfig.vehicle.jerkLimits       = requireSeq<double>(v, "jerk_limits",       ctx, n);
}

// ---------------------------------------------------------------------------
// manipulator:
// ---------------------------------------------------------------------------
void ConfigManager::loadManipulatorSection(const fs::path& manipDir)
{
    const std::string ctx = "manipulator";
    const YAML::Node& m = require(mManipRoot, "manipulator", ctx);

    mConfig.manipulator.urdfPath = resolvePath(manipDir, requireScalar<std::string>(m, "urdf", ctx));
    mConfig.manipulator.baseLink = requireScalar<std::string>(m, "base_link", ctx);
    mConfig.manipulator.endLink  = requireScalar<std::string>(m, "end_link",  ctx);

    auto jointNames = requireSeq<std::string>(m, "joint_names", ctx);
    const std::size_t n = jointNames.size();
    mConfig.manipulator.jointNames = std::move(jointNames);

    mConfig.manipulator.initialPositions = requireSeq<double>(m, "initial_positions", ctx, n);
    mConfig.manipulator.accelLimits      = requireSeq<double>(m, "accel_limits",      ctx, n);
    mConfig.manipulator.jerkLimits       = requireSeq<double>(m, "jerk_limits",       ctx, n);

    // Named presets validated against arm joint count only
    mConfig.manipulator.deployPos = optionalSeq<double>(m["deploy"], "pos", n);
    mConfig.manipulator.stowPos   = optionalSeq<double>(m["stow"],   "pos", n);
}

// ---------------------------------------------------------------------------
// mount:
// ---------------------------------------------------------------------------
void ConfigManager::loadMountSection()
{
    const std::string ctx = "mount";
    const YAML::Node& m = require(mManipRoot, "mount", ctx);

    auto xyz = requireSeq<double>(m, "position",   ctx, 3);
    auto q   = requireSeq<double>(m, "quaternion", ctx, 4); // x y z w

    const KDL::Vector   p(xyz[0], xyz[1], xyz[2]);
    const KDL::Rotation r = KDL::Rotation::Quaternion(q[0], q[1], q[2], q[3]);
    mConfig.T_V_B = KDL::Frame(r, p);
}

// ---------------------------------------------------------------------------
// mergeJointData — concatenate [vehicle..., arm...] into flat combined vectors
// ---------------------------------------------------------------------------
void ConfigManager::mergeJointData()
{
    append(mConfig.jointNames,       mConfig.vehicle.jointNames);
    append(mConfig.jointNames,       mConfig.manipulator.jointNames);

    append(mConfig.initialPositions, mConfig.vehicle.initialPositions);
    append(mConfig.initialPositions, mConfig.manipulator.initialPositions);

    append(mConfig.accelLimits,      mConfig.vehicle.accelLimits);
    append(mConfig.accelLimits,      mConfig.manipulator.accelLimits);

    append(mConfig.jerkLimits,       mConfig.vehicle.jerkLimits);
    append(mConfig.jerkLimits,       mConfig.manipulator.jerkLimits);
}