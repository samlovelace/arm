
#include "common/KinematicsHandler.h"
#include "plog/Log.h"
#include <iostream> 
#include <eigen3/Eigen/Dense>

KinematicsHandler::KinematicsHandler() : mModel(std::make_shared<urdf::Model>())
{

}

KinematicsHandler::~KinematicsHandler()
{

}

bool KinematicsHandler::init(const std::string& anUrdfPath)
{ 
    if(!kdl_parser::treeFromFile(anUrdfPath, mTree))
    {
        LOGE << "Failed to parse urdf to KDL::Tree"; 
        return false; 
    }

    LOGD << "Successfully parsed urdf with " << mTree.getNrOfJoints() << " joints!"; 

    // TODO: this will break with any other arm that doesnt have base_link or tool0
    if (!mTree.getChain("base_link", "tool0", mChain)) 
    {
        throw std::runtime_error("Failed to extract KDL chain from tree");
    }

	for(int i = 0; i < mChain.getNrOfSegments(); i++)
	{
		const auto& seg = mChain.getSegment(i); 

		mSegmentNameMap.insert({seg.getName(), i});

		LOGV << "Segment " << i << " has"
			 <<	" name: " << seg.getName() 
			 << " joint: " << seg.getJoint().getName(); 
	}

    mModel->clear(); 
    if(!mModel->initFile(anUrdfPath))
    {
        LOGE << "Could not parse model from urdf"; 
        return false; 
    }

	if(!parseCollisionGeometry())
	{
		LOGE << "Failed to parse collision geometry"; 
		return false; 
	}

    KDL::JntArray lower(mChain.getNrOfJoints()); 
    KDL::JntArray upper(mChain.getNrOfJoints()); 
    KDL::JntArray velocity(mChain.getNrOfJoints()); 
    int idx = 0; 

    // TODO: add parsing and saving of effort and velocity limits 
    for(size_t idx = 0; idx < mChain.getNrOfJoints(); idx++)
    {
        KDL::Segment segment = mChain.getSegment(idx); 
        std::string jointName = segment.getJoint().getName(); 
        mJointNames.push_back(jointName); 
        
        auto urdfJoint = mModel->getJoint(jointName); 

        lower(idx) = urdfJoint->limits->lower; 
        upper(idx) = urdfJoint->limits->upper;  
        velocity(idx) = urdfJoint->limits->velocity; 
    }

    mLimitsMap.insert({"lower", lower}); 
    mLimitsMap.insert({"upper", upper});
    mLimitsMap.insert({"velocity", velocity});  

    KDL::JntArray q_min = getJointLimits("lower"); 
    KDL::JntArray q_max = getJointLimits("upper"); 

    mFkSolver = std::make_shared<KDL::ChainFkSolverPos_recursive>(mChain);
    mVelSolver = std::make_shared<KDL::ChainIkSolverVel_pinv>(mChain);
    mIkSolver = std::make_shared<KDL::ChainIkSolverPos_NR_JL>(mChain, q_min, q_max, *mFkSolver, *mVelSolver, 200, 1e-1);
    mJacobianSolver = std::make_shared<KDL::ChainJntToJacSolver>(mChain); 

    LOGD << "KinematicsHandler initialized successfully"; 
    return true; 
}

bool KinematicsHandler::parseCollisionGeometry() 
{
	mCollisionShells.resize(mChain.getNrOfSegments());  
	std::vector<CollisionShell> linkCollisions; 

	for (const auto& kv : mModel->links_) 
	{
		const urdf::LinkConstSharedPtr& link = kv.second;
		linkCollisions.clear(); 

		for (const auto& coll : link->collision_array) 
		{
			if (!coll || !coll->geometry) continue;

			double roll, pitch, yaw; 
			coll->origin.rotation.getRPY(roll, pitch, yaw);
			
			KDL::Rotation rot = KDL::Rotation::RPY(roll, pitch, yaw);
			KDL::Vector t = KDL::Vector(coll->origin.position.x, coll->origin.position.y, coll->origin.position.z);
			KDL::Frame T_link_shell(rot, t);

			CollisionShell cs;
			cs.T_link_shell = T_link_shell; 
			cs.T_base_shell = KDL::Frame::Identity(); // TODO: probably compute this based on initial joint state

			CollisionShape shape; 
			std::string shapeType = ""; 

			switch (coll->geometry->type) 
			{
				case urdf::Geometry::BOX: 
				{
					shapeType = "BOX"; 
					auto* box = dynamic_cast<urdf::Box*>(coll->geometry.get());
					shape.type = CollisionShape::Box;
					shape.bx = box->dim.x; shape.by = box->dim.y; shape.bz = box->dim.z;
					break;
				}
				case urdf::Geometry::CYLINDER: 
				{
					shapeType = "CYLINDER"; 
					auto* cyl = dynamic_cast<urdf::Cylinder*>(coll->geometry.get());
					shape.type = CollisionShape::Cylinder;
					shape.radius = cyl->radius; shape.length = cyl->length;
					break;
				}
				case urdf::Geometry::SPHERE: 
				{
					shapeType = "SPHERE"; 
					auto* sph = dynamic_cast<urdf::Sphere*>(coll->geometry.get());
					shape.type = CollisionShape::Sphere;
					shape.sr = sph->radius;
					break;
				}
				case urdf::Geometry::MESH: 
				{
					shapeType = "MESH"; 
					auto* mesh = dynamic_cast<urdf::Mesh*>(coll->geometry.get());
					shape.type = CollisionShape::Mesh;
					shape.mesh_filename = mesh->filename;
					shape.mesh_scale_x = mesh->scale.x;
					shape.mesh_scale_y = mesh->scale.y;
					shape.mesh_scale_z = mesh->scale.z;
					break;
				}

				default: break;
			}

			cs.shape = shape; 
			linkCollisions.push_back(std::move(cs)); 
		}

		if(linkCollisions.size() == 0)
		{
			continue;
		}

		LOGV << "Adding " << linkCollisions.size() << " collision geometries for link " 
			 << link->name;

		unsigned int index = -1; 
		if(mSegmentNameMap.find(link->name) == mSegmentNameMap.end())
		{
			LOGW << "No matching link name for collision geometry"; 
			continue; 
		}

		index = mSegmentNameMap.at(link->name);
		mCollisionShells[index] = linkCollisions;  
	}

	return true; 
}

bool KinematicsHandler::solveIK(const KDL::JntArray& anInitPos, const KDL::Frame& aGoalPose, KDL::JntArray& aResultOut)
{
    //LOGW << "Chain size: " << mChain.getNrOfJoints() << " anInitPos size: " << anInitPos.rows(); 
    // TODO: error checking on JntArray sizes 
    int result = mIkSolver->CartToJnt(anInitPos, aGoalPose, aResultOut);
    
    // KDL::Frame fkPose;
    // int fkResult = mFkSolver->JntToCart(aResultOut, fkPose);
    // KDL::Frame err = aGoalPose * fkPose.Inverse(); 

    // // Position
    // double x = err.p.x();
    // double y = err.p.y();
    // double z = err.p.z();

    // // Orientation (Quaternion)
    // double qx, qy, qz, qw;
    // err.M.GetQuaternion(qx, qy, qz, qw);

    // LOGD << "Error Frame Position: x=" << x << ", y=" << y << ", z=" << z;
    // //LOGD << "Frame Orientation (quaternion): qw=" << qw
    // //    << ", qx=" << qx << ", qy=" << qy << ", qz=" << qz;

    if(result != 0)
    {
        //LOGW << "Failed to solve IK, error code: " << result; 
        return false; 
    }

    return true; 
}

bool KinematicsHandler::solveIK(const KDL::JntArray& anInitPos, const KDL::Twist& aGoalVel, KDL::JntArray& aResultOut)
{
   int result = mVelSolver->CartToJnt(anInitPos, aGoalVel, aResultOut); 

   if(result != 0)
   {    
        LOGW << "Failed to solve IK for goal vel. Error code: " << result; 
        return false; 
   }
   
   return true; 
}

bool KinematicsHandler::solveFk(const KDL::JntArray& anInitPos, KDL::Frame& aFrameOut, int aSegmentNr)
{
    int fkResult = mFkSolver->JntToCart(anInitPos, aFrameOut, aSegmentNr);
    
    // // Position
    // double x = aFrameOut.p.x();
    // double y = aFrameOut.p.y();
    // double z = aFrameOut.p.z();

    // // Orientation (Quaternion)
    // double qx, qy, qz, qw;
    // aFrameOut.M.GetQuaternion(qx, qy, qz, qw);

    // LOGD << "Frame Position: x=" << x << ", y=" << y << ", z=" << z;
    // LOGD << "Frame Orientation (quaternion): qw=" << qw
    //     << ", qx=" << qx << ", qy=" << qy << ", qz=" << qz; 

    if(0 > fkResult)
    {
        return false; 
    }

    return true; 
}

KDL::JntArray KinematicsHandler::getJointLimits(const std::string& aLimitType)
{
    KDL::JntArray limits(mChain.getNrOfJoints()); 

    if(mLimitsMap.find(aLimitType) != mLimitsMap.end())
    {
        return mLimitsMap.at(aLimitType); 
    }

    return limits; 
}

double KinematicsHandler::computeManipulability(const KDL::JntArray& aJntCnfg)
{
    KDL::Jacobian jac(mChain.getNrOfJoints());

    if (mJacobianSolver->JntToJac(aJntCnfg, jac) < 0) 
    {
        std::cerr << "Error computing Jacobian" << std::endl;
        return -1;
    }

    // Map KDL Jacobian safely (unaligned + row-major)
    Eigen::Map<
        const Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::RowMajor>,
        Eigen::Unaligned
    > Jmap(jac.data.data(), 6, jac.columns());

    Eigen::MatrixXd J = Jmap;
    Eigen::MatrixXd JJt = J * J.transpose();
    double det = JJt.determinant();

    if (det <= 0.0)
        return 0.0;

    return std::sqrt(det);
}

void KinematicsHandler::updateCollisionShells(const KDL::JntArray& aCurrentJointState)
{
    for (size_t index = 0; index < mCollisionShells.size(); ++index)
    { 
        KDL::Frame T_base_link;
        if (!solveFk(aCurrentJointState, T_base_link, index))
        {
            LOGW << "Failed to solve FK for segment " << index;
            continue;
        }

        KDL::Segment seg = mChain.getSegment(index);

        for (auto& shell : mCollisionShells[index])
        {
            KDL::Vector lp = shell.T_link_shell.p;
            //LOGV << "Shell pos rel to link: " << lp.x() << "," << lp.y() << "," << lp.z();

            shell.T_base_shell = T_base_link * shell.T_link_shell;
            KDL::Vector sp = shell.T_base_shell.p;

            //LOGV << "Updated shell pose on link " << seg.getName()
            //     << " to " << sp.x() << "," << sp.y() << "," << sp.z();
        }
    }
}

bool KinematicsHandler::checkCollisions(const KDL::JntArray& aJntConfig)
{
	std::stringstream s; 
	s << "Checking collision shells for joint configuration "; 
	for(int i = 0; i < aJntConfig.rows(); i++)
	{
		s << aJntConfig(i) << ",";
	}
	LOGV << s.str(); 
	
	updateCollisionShells(aJntConfig); 

	for (size_t i = 0; i < mCollisionShells.size(); ++i) 
	{
		for (size_t j = i + 1; j < mCollisionShells.size(); ++j) 
		{
			if (j == i + 1) continue; // skip adjacent segments

			for (const auto& shellA : mCollisionShells[i]) 
			{
				for (const auto& shellB : mCollisionShells[j]) 
				{
					if (collides(shellA, shellB)) 
					{
						const auto& linkAName = mChain.getSegment(i).getName();
						const auto& linkBName = mChain.getSegment(j).getName();
						LOGW << "Detected collision between " << linkAName << " and " << linkBName;
						return false; 
					}
				}
			}
		}
	}

	return true; 
}

bool KinematicsHandler::collides(const CollisionShell& aFirstShell, const CollisionShell& aSecondShell)
{
	std::vector<KDL::Vector> firstPts; 
	std::vector<KDL::Vector> secondPts; 
	shellToPoints(aFirstShell, firstPts); 
	shellToPoints(aSecondShell, secondPts); 

	CollisionShape::Type firstShellType = aFirstShell.shape.type; 
	CollisionShape::Type secondShellType = aSecondShell.shape.type; 

	if(firstShellType == CollisionShape::Cylinder && secondShellType == CollisionShape::Cylinder)
	{
		// LOGV << "p1: " << firstPts[0].x() << "," << firstPts[0].y() << "," << firstPts[0].z()
		// 	 << ", " << "q1: " << firstPts[1].x() << "," << firstPts[1].y() << "," << firstPts[1].z(); 
		
		// LOGV << "p2: " << secondPts[0].x() << "," << secondPts[0].y() << "," << secondPts[0].z()
		// 	 << ", " << "q2: " << secondPts[1].x() << "," << secondPts[1].y() << "," << secondPts[1].z();
		
		double dist = segmentDistance(firstPts[0], firstPts[1], secondPts[0], secondPts[1]); 
		double r1 = aFirstShell.shape.radius; 
		double r2 = aSecondShell.shape.radius; 

		return dist <= (r1 + r2); 
	}
	else
	{
		LOGW << "Checking collision between dissimilar types not yet supported"; 
		return false; 
	}

}

void KinematicsHandler::shellToPoints(const CollisionShell& aShell, std::vector<KDL::Vector>& aPoints)
{
	switch (aShell.shape.type)
	{
	case CollisionShape::Cylinder:
	{
		KDL::Vector axis = aShell.T_base_shell.M * KDL::Vector(0, 0, 1); 
		axis.Normalize(); 

		double h = aShell.shape.length; 
		KDL::Vector center = aShell.T_base_shell.p; 

		//
		KDL::Vector p1 = center; 
		KDL::Vector p2 = center + h * axis; 
		//KDL::Vector p1 = center - (0.5 * h * axis);
		//KDL::Vector p2 = center + (0.5 * h * axis); 

		aPoints.push_back(p1); 
		aPoints.push_back(p2); 
		break;
	}
	default:
		LOGW << "Converting shell to points for type: " << aShell.shape.type << " not supported yet"; 
		break;
	}

}

double KinematicsHandler::segmentDistance(const KDL::Vector& p1, const KDL::Vector& q1,
                                          const KDL::Vector& p2, const KDL::Vector& q2)
{
    KDL::Vector d1 = q1 - p1;
    KDL::Vector d2 = q2 - p2;
    KDL::Vector r  = p1 - p2;

    double a = KDL::dot(d1, d1);
    double e = KDL::dot(d2, d2);
    double f = KDL::dot(d2, r);

    double s = 0.0, t = 0.0;
    constexpr double eps = 1e-9;

    if (a <= eps && e <= eps) {
        return (p1 - p2).Norm();
    }
    if (a <= eps) {
        t = std::clamp(f / e, 0.0, 1.0);
        s = 0.0;
    } else {
        double c = KDL::dot(d1, r);
        if (e <= eps) {
            s = std::clamp(-c / a, 0.0, 1.0);
            t = 0.0;
        } else {
            double b = KDL::dot(d1, d2);
            double denom = a*e - b*b;
            if (std::abs(denom) > eps) {
                s = std::clamp((b*f - c*e) / denom, 0.0, 1.0);
            } else {
                s = 0.0; // nearly parallel; fallback
            }
            t = (b*s + f) / e;
            if (t < 0.0) { t = 0.0; s = std::clamp(-c / a, 0.0, 1.0); }
            else if (t > 1.0) { t = 1.0; s = std::clamp((b - c) / a, 0.0, 1.0); }
        }
    }

    KDL::Vector cp1 = p1 + d1 * s;
    KDL::Vector cp2 = p2 + d2 * t; // fixed
    return (cp1 - cp2).Norm();
}
