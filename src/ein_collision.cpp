#include "ein_words.h"
#include "ein.h"

#include <kdl_parser/kdl_parser.hpp>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

namespace ein_words {

WORD(InitializeKdlTree)
virtual void execute(MachineState * ms)
{
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(ms->config.robot_description, tree)){
    ROS_ERROR("Failed to construct kdl tree");
  }
  cout << "KDL Joints: " << tree.getNrOfJoints() << endl;


  KDL::SegmentMap::const_iterator root = tree.getRootSegment();

  KDL::SegmentMap::const_iterator arm = tree.getSegment(ms->config.left_or_right_arm + "_arm_mount");

  // XX:  consider storing the arm chain in config
  KDL::Chain chain;
  bool result = tree.getChain(ms->config.left_or_right_arm + "_arm_mount", ms->config.left_or_right_arm + "_gripper_base", chain);
  if (! result) {
    ROS_ERROR("Couldn't get chain.");
  }

  
  // cout << "Key: " << root->first << endl;
  // for (int i = 0; i < root->second.children.size(); i++) {
  // KDL::SegmentMap::const_iterator child = root->second.children[i];
  // cout << "Child: " << child->first << endl;
  // for (int j = 0; j < child->second.children.size(); j++) {
  //    KDL::SegmentMap::const_iterator child1 = child->second.children[j];
  //    cout << "  Child1: " << child1->first << endl;
  //  }
  // }

  KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);


  KDL::JntArray jointpositions = KDL::JntArray(NUM_JOINTS);
  KDL::Frame cartpos;    

  // todo:  verify  that known joint angles give correct outputs with the EE pose.
  bool kinematics_status = fksolver.JntToCart(jointpositions, cartpos);
  if (kinematics_status>=0) {
  std::cout << cartpos <<std::endl;
  printf("%s \n","Succes, thanks KDL!");
  } else {
  printf("%s \n","Error: could not calculate forward kinematics :(");
  }

}
END_WORD
REGISTER_WORD(InitializeKdlTree)


}
