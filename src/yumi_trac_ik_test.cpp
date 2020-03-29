#include "ros/ros.h"
#include <trac_ik/trac_ik.hpp>

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

using namespace KDL;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ik_test");
    ros::NodeHandle nh("~");

    int num_samples;
    std::string chain_start, chain_end, urdf_param;
    double timeout;
    const double error = 1e-5;

    nh.param("chain_start", chain_start, std::string(""));
    nh.param("chain_end", chain_end, std::string(""));

    if (chain_start=="" || chain_end=="") {
        ROS_FATAL("Missing chain info in launch file");
        exit (-1);
    }

    nh.param("timeout", timeout, 0.005);
    nh.param("urdf_param", urdf_param, std::string("/robot_description"));

    if (num_samples < 1)
        num_samples = 1;


    TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, urdf_param, timeout, error, TRAC_IK::Speed);  

    KDL::Chain chain;
    bool valid = ik_solver.getKDLChain(chain);

    if (!valid) {
        ROS_ERROR("There was no valid KDL chain found");
        return -1;
    }

    // Create position array
    unsigned int nj = chain.getNrOfJoints();
    ROS_INFO ("Using postion");

    // Create the frame that will contain the results
    geometry_msgs::PoseStamped command_pose;
    float myinput;  
    printf ("Enter the position x :");
    scanf ("%e",&myinput);
    command_pose.pose.position.x=(double)myinput;
    printf ("Enter the position y :");
    scanf ("%e",&myinput);
    command_pose.pose.position.y=(double)myinput;
    printf ("Enter the position z :");
    scanf ("%e",&myinput);
    command_pose.pose.position.z=(double)myinput;
    printf ("Enter the orientation x :");
    scanf ("%e",&myinput);
    command_pose.pose.orientation.x=(double)myinput;
    printf ("Enter the orientation y :");
    scanf ("%e",&myinput);
    command_pose.pose.orientation.y=(double)myinput;
    printf ("Enter the orientation z :");
    scanf ("%e",&myinput);
    command_pose.pose.orientation.z=(double)myinput;
    printf ("Enter the orientation w :");
    scanf ("%e",&myinput);
    command_pose.pose.orientation.w=(double)myinput;

    KDL::Rotation desired_orient = KDL::Rotation::Quaternion(command_pose.pose.orientation.x, command_pose.pose.orientation.y, command_pose.pose.orientation.z, command_pose.pose.orientation.w);
    KDL::Vector desired_position =  KDL::Vector(command_pose.pose.position.x, command_pose.pose.position.y, command_pose.pose.position.z);
    KDL::Frame desired_pose = KDL::Frame(desired_orient, desired_position);
    
    KDL::JntArray joint_seed(nj);
    KDL::SetToZero(joint_seed);
    KDL::JntArray result(joint_seed);
   
    int rc=ik_solver.CartToJnt(joint_seed,desired_pose,result);
    if(rc < 0)
        printf("%s \n","Error: could not calculate forward kinematics :(");
    else{
        printf("%s \n","TRAC IK Succes");
        for(unsigned int i = 0; i < nj; i++)
            std::cout << result(i) << " ";
    }

    return 0;
}