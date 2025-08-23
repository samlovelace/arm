#include "IrmGenerator.h"

IrmGenerator::IrmGenerator(ConfigManager::Config aConfig) : mConfig(aConfig)
{
    std::string urdfFilePath = mConfig.shareDir + "manipulators/" + mConfig.manipType + "/manipulator.urdf";
    std::cout << "urdfFilePath: " << urdfFilePath << std::endl;

    mKinematicsHandler = std::make_shared<KinematicsHandler>();
    if (!mKinematicsHandler->init(urdfFilePath))
    {
        throw(std::runtime_error("Failed to initialize kinematics handler\n"));
    }

    std::cout << "Initialized kinematics handler!\n";
}

IrmGenerator::~IrmGenerator() {}

bool IrmGenerator::generate()
{
    std::cout << "Generating Inverse Reachability Map for " << mConfig.manipType << std::endl;

    KDL::JntArray jntMaxLimits = mKinematicsHandler->getJointLimits("upper");
    KDL::JntArray jntMinLimits = mKinematicsHandler->getJointLimits("lower");

    int numJoints = jntMinLimits.rows();
    std::cout << "NumJoints: " << numJoints << std::endl;
    std::cout << "Lower Joint Limits: " << jntMinLimits.data << std::endl;
    std::cout << "Upper Joint Limits: " << jntMaxLimits.data << std::endl;

    float resolution = 10; // TODO: make configurable
    KDL::JntArray jntCfg(numJoints);
    std::vector<size_t> indices(numJoints, 0);

    size_t total_samples = 1;
    for (int r = 0; r < numJoints; r++)
        total_samples *= (resolution + 1);

    std::cout << "Generating IRM with " << total_samples << " samples" << std::endl;

    for (size_t samples = 0; samples < total_samples; samples++)
    {
        for (int i = 0; i < numJoints; i++)
        {
            float step = (jntMaxLimits(i) - jntMinLimits(i)) / resolution;
            jntCfg(i) = jntMinLimits(i) + step * indices[i];
        }

        compute(jntCfg);

        // Increment multi-digit counter
        for (size_t j = 0; j < numJoints; ++j)
        {
            if (++indices[j] <= resolution)
                break;
            indices[j] = 0;
        }
    }

    std::cout << "Writing IRM to file\n";
    if (!toFile())
    {
        std::cerr << "Failed to write IRM to file\n";
        return false;
    }

    return true;
}

bool IrmGenerator::compute(const KDL::JntArray& aJntCfg)
{
    double manipulability = mKinematicsHandler->computeManipulability(aJntCfg);

    if (std::isnan(manipulability) || manipulability < 0.05)
        return false;

    KDL::Frame frame;
    if (!mKinematicsHandler->solveFk(aJntCfg, frame))
        return false;

    KDL::Frame inverse = frame.Inverse();

    IrmEntryBinary entry;
    entry.position[0] = inverse.p[0];
    entry.position[1] = inverse.p[1];
    entry.position[2] = inverse.p[2];

    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            entry.orientation[r * 3 + c] = inverse.M(r, c);

    entry.manipulability = manipulability;
    mIrmEntries.push_back(entry);

    return true;
}

bool IrmGenerator::toFile()
{
    std::ofstream outfile("IRM.bin", std::ios::binary);
    if (!outfile.is_open())
    {
        std::cerr << "Failed to open IRM.bin for writing\n";
        return false;
    }

    outfile.write(reinterpret_cast<const char*>(mIrmEntries.data()),
                  mIrmEntries.size() * sizeof(IrmEntryBinary));

    return true;
}
