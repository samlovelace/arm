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

    loadManipulatorSection(manipDir);
    loadGripperSection    ();
    loadMountSection      ();

    // Mobile base is optional — a manipulator with no MobileBase entry is standalone
    if (topConfig["MobileBase"] && !topConfig["MobileBase"].IsNull())
    {
        const std::string baseName = requireScalar<std::string>(topConfig["MobileBase"], "name", "MobileBase");

        const fs::path baseDir     = shareDir / "config" / "mobile_base" / baseName;
        const fs::path baseYamlPath = baseDir / "base.yaml";
        if (!fs::exists(baseYamlPath))
            throw std::runtime_error(
                "[ConfigManager] base.yaml not found for mobile base '" + baseName +
                "': " + baseYamlPath.string());

        LOGI << "Loading mobile base config: " << baseYamlPath.string();
        mBaseRoot = YAML::LoadFile(baseYamlPath.string());

        loadMobileBaseSection(baseDir);
    }

    mergeJointData();

    LOGI << "Config loaded successfully — manipulator: '" << manipName
         << "', " << mConfig.jointNames.size() << " total joints ("
         << (mConfig.mobileBase ? mConfig.mobileBase->jointNames.size() : 0) << " mobile base, "
         << mConfig.manipulator.jointNames.size()  << " arm)";
    LOGD << YAML::Dump(mManipRoot);
}

// ---------------------------------------------------------------------------
// mobile_base/<name>/base.yaml:
// ---------------------------------------------------------------------------
void ConfigManager::loadMobileBaseSection(const fs::path& baseDir)
{
    const std::string ctx = "mobile_base";

    MobileBaseConfig mb;
    mb.urdfPath        = resolvePath(baseDir, requireScalar<std::string>(mBaseRoot, "urdf", ctx));
    mb.manipAttachLink = requireScalar<std::string>(mBaseRoot, "manip_attach_link", ctx);

    auto jointNames = requireSeq<std::string>(mBaseRoot, "joint_names", ctx);
    const std::size_t n = jointNames.size();
    mb.jointNames = std::move(jointNames);

    mb.initialPositions = requireSeq<double>(mBaseRoot, "initial_positions", ctx, n);
    mb.accelLimits      = requireSeq<double>(mBaseRoot, "accel_limits",      ctx, n);
    mb.jerkLimits       = requireSeq<double>(mBaseRoot, "jerk_limits",       ctx, n);

    mConfig.mobileBase = std::move(mb);
}

// ---------------------------------------------------------------------------
// manip.yaml (flattened — arm fields live at the root, no more nested key)
// ---------------------------------------------------------------------------
void ConfigManager::loadManipulatorSection(const fs::path& manipDir)
{
    const std::string ctx = "manip.yaml";
    const YAML::Node& m = mManipRoot;

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
// manip.yaml: gripper (optional — absent for a manipulator with no gripper).
// Deliberately not merged into Config's flat joint vectors — those feed
// WaypointExecutor/KinematicsHandler and must stay arm(+base)-only.
// ---------------------------------------------------------------------------
void ConfigManager::loadGripperSection()
{
    const YAML::Node& m = mManipRoot;

    if (!m["gripper"] || m["gripper"].IsNull())
        return;

    const std::string ctx = "gripper";
    const YAML::Node& g = m["gripper"];

    GripperConfig gripper;

    auto jointNames = requireSeq<std::string>(g, "joint_names", ctx);
    const std::size_t n = jointNames.size();
    gripper.jointNames = std::move(jointNames);

    gripper.initialPositions = requireSeq<double>(g, "initial_positions", ctx, n);

    gripper.openPos   = optionalSeq<double>(g["open"],   "pos", n);
    gripper.closedPos = optionalSeq<double>(g["closed"], "pos", n);

    mConfig.gripper = std::move(gripper);
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
// mergeJointData — concatenate [mobile base..., arm...] into flat combined
// vectors. If there's no mobile base, the combined vectors are arm-only.
// ---------------------------------------------------------------------------
void ConfigManager::mergeJointData()
{
    if (mConfig.mobileBase)
    {
        append(mConfig.jointNames,       mConfig.mobileBase->jointNames);
        append(mConfig.initialPositions, mConfig.mobileBase->initialPositions);
        append(mConfig.accelLimits,      mConfig.mobileBase->accelLimits);
        append(mConfig.jerkLimits,       mConfig.mobileBase->jerkLimits);
    }

    append(mConfig.jointNames,       mConfig.manipulator.jointNames);
    append(mConfig.initialPositions, mConfig.manipulator.initialPositions);
    append(mConfig.accelLimits,      mConfig.manipulator.accelLimits);
    append(mConfig.jerkLimits,       mConfig.manipulator.jerkLimits);
}