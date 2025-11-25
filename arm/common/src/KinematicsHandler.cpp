
#include "common/KinematicsHandler.h"
#include "plog/Log.h"
#include <iostream> 
#include <eigen3/Eigen/Dense>

KinematicsHandler::KinematicsHandler() : 
    mModel(std::make_shared<urdf::Model>()), 
    mRobotModel(std::make_shared<urdf::Model>())
{

}

KinematicsHandler::~KinematicsHandler()
{

}

bool KinematicsHandler::init(const std::string& anUrdfPath, const std::string& aRobotUrdfPath)
{ 
    if(!kdl_parser::treeFromFile(anUrdfPath, mTree))
    {
        LOGE << "Failed to parse urdf to KDL::Tree"; 
        return false; 
    } 

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

	if(!parseManipCollisionGeometry())
	{
		LOGE << "Failed to parse manipulator collision geometry"; 
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

    // parse mobile robot if present 
    if(!aRobotUrdfPath.empty())
    {
        mRobotModel->clear();  
        if(!mRobotModel->initFile(aRobotUrdfPath))
        {
            LOGE << "Failed to parse robot urdf file"; 
            return false; 
        } 

        LOGV << "Parsed robot urdf file"; 

        if(!parseRobotCollisionGeometry())
        {
            LOGE << "Failed to parse robot collision geometry"; 
            return false; 
        }
    }

    LOGD << "KinematicsHandler initialized successfully"; 
    return true; 
}

bool KinematicsHandler::parseManipCollisionGeometry()  
{
	mCollisionShells.resize(mChain.getNrOfSegments());  
	std::vector<CollisionShell> linkCollisions; 

	for (const auto& kv : mModel->links_) 
	{
        urdf::LinkConstSharedPtr link = kv.second; 
        parseLinkCollisions(link, linkCollisions); 

		if(linkCollisions.empty())
			continue;

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

bool KinematicsHandler::parseRobotCollisionGeometry()
{
    std::vector<CollisionShell> linkShells; 

    for(const auto& kv : mRobotModel->links_)
    {
        urdf::LinkConstSharedPtr link = kv.second; 
        parseLinkCollisions(link, linkShells); 

        if(linkShells.empty())
            continue;
        
        LOGV << "Adding " << linkShells.size() << " collision geometries for link " 
			 << link->name;
        mRobotShells.push_back(linkShells); 
    }   

    return true; 
}

void KinematicsHandler::parseLinkCollisions(urdf::LinkConstSharedPtr& aLink, std::vector<CollisionShell>& aLinkShellsOut)
{
    aLinkShellsOut.clear(); 

    for (const auto& coll : aLink->collision_array) 
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
                auto* box = dynamic_cast<urdf::Box*>(coll->geometry.get());
                shape.type = CollisionShape::Box;
                shape.bx = box->dim.x; shape.by = box->dim.y; shape.bz = box->dim.z;
                break;
            }
            case urdf::Geometry::CYLINDER: 
            {
                auto* cyl = dynamic_cast<urdf::Cylinder*>(coll->geometry.get());
                shape.type = CollisionShape::Cylinder;
                shape.radius = cyl->radius; shape.length = cyl->length;
                break;
            }
            case urdf::Geometry::SPHERE: 
            {
                auto* sph = dynamic_cast<urdf::Sphere*>(coll->geometry.get());
                shape.type = CollisionShape::Sphere;
                shape.sr = sph->radius;
                break;
            }
            case urdf::Geometry::MESH: 
            {
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
        aLinkShellsOut.push_back(std::move(cs)); 
    }
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

    const int rows = 6;
    const int cols = jac.columns();

    // SAFE COPY
    Eigen::MatrixXd J(rows, cols);
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            J(r, c) = jac(r, c);

    // Compute manipulability
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
						//LOGW << "Detected collision between " << linkAName << " and " << linkBName;
						return false; 
					}
				}
			}
		}

        for(size_t j = 0; j < mRobotShells.size(); j++)
        {
            for(const auto& shellA : mCollisionShells[i])
            {
                for(const auto& shellB : mRobotShells[j])
                {
                    // skip manip base link shell
                    if(i == 0) continue;

                    if(collides(shellA, shellB))
                    {
                        const auto& manipLinkName = mChain.getSegment(i).getName(); 

                        //LOGW << "Detected collision between manip " << manipLinkName 
                        //     << " and robot";  
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
		double dist = segmentDistance(firstPts[0], firstPts[1], secondPts[0], secondPts[1]); 
		double r1 = aFirstShell.shape.radius; 
		double r2 = aSecondShell.shape.radius; 

		return dist <= (r1 + r2); 
	}
    else if(firstShellType == CollisionShape::Cylinder && secondShellType == CollisionShape::Box)
    {   
        double dist = segmentBoxDistance(firstPts[0], firstPts[1],
                                         aSecondShell.T_base_shell, 
                                         aSecondShell.shape.bx, 
                                         aSecondShell.shape.by, 
                                         aSecondShell.shape.bz); 
        double r = aFirstShell.shape.radius; 
        return dist <= r; 
    }
    else if(firstShellType == CollisionShape::Box && secondShellType == CollisionShape::Cylinder)
    {
        double dist = segmentBoxDistance(secondPts[0], secondPts[1],
                                         aFirstShell.T_base_shell, 
                                         aFirstShell.shape.bx, 
                                         aFirstShell.shape.by, 
                                         aFirstShell.shape.bz); 
        double r = aSecondShell.shape.radius; 
        return dist <= r;
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

		KDL::Vector p1 = center; 
		KDL::Vector p2 = center + h * axis; 

		aPoints.push_back(p1); 
		aPoints.push_back(p2); 
		break;
	}
	default:
		//LOGW << "Converting shell to points for type: " << aShell.shape.type << " not supported yet"; 
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

double KinematicsHandler::segmentBoxDistance(const KDL::Vector& p1_world,
                                             const KDL::Vector& p2_world,
                                             const KDL::Frame& box_frame,
                                             double bx, double by, double bz)
{
    // Box half extents
    KDL::Vector h(bx/2.0, by/2.0, bz/2.0);

    // Transform endpoints into box-local coordinates
    KDL::Rotation R = box_frame.M;
    KDL::Vector t  = box_frame.p;

    KDL::Vector p1 = R.Inverse() * (p1_world - t);
    KDL::Vector p2 = R.Inverse() * (p2_world - t);

    // Direction and vector to compute t
    KDL::Vector d = p2 - p1;

    double t0 = 0.0, t1 = 1.0;

    // Liang–Barsky clipping for segment vs AABB
    auto clip = [&](double denom, double numer)->bool {
        if (std::abs(denom) < 1e-9) {
            if (numer < 0) return false;
            return true;
        }
        double t = numer / denom;
        if (denom > 0) {
            if (t > t1) return false;
            if (t > t0) t0 = t;
        } else {
            if (t < t0) return false;
            if (t < t1) t1 = t;
        }
        return true;
    };

    // AABB defines planes along x,y,z
    if (!clip( d.x(), -h.x() - p1.x() )) return (p1 - p2).Norm();
    if (!clip( d.x(),  h.x() - p1.x() )) return (p1 - p2).Norm();

    if (!clip( d.y(), -h.y() - p1.y() )) return (p1 - p2).Norm();
    if (!clip( d.y(),  h.y() - p1.y() )) return (p1 - p2).Norm();

    if (!clip( d.z(), -h.z() - p1.z() )) return (p1 - p2).Norm();
    if (!clip( d.z(),  h.z() - p1.z() )) return (p1 - p2).Norm();

    // If clipped segment intersects box
    if (t0 <= t1) {
        return 0.0;  // collision/contact
    }

    // Otherwise compute shortest distance from endpoints to box
    auto pointAABBDistance = [&](const KDL::Vector& p)->double {
        double dx = std::max({0.0, p.x() - h.x(), -h.x() - p.x()});
        double dy = std::max({0.0, p.y() - h.y(), -h.y() - p.y()});
        double dz = std::max({0.0, p.z() - h.z(), -h.z() - p.z()});
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    };

    double d1 = pointAABBDistance(p1);
    double d2 = pointAABBDistance(p2);

    return std::min(d1, d2);
}

