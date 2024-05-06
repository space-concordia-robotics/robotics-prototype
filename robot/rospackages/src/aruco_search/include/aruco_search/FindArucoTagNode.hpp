#ifndef ARUCO_SEARCH_SRC_FINDARUCOTAGNODE_H_
#define ARUCO_SEARCH_SRC_FINDARUCOTAGNODE_H_
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/wait.hpp"

class FindArucoTagNode : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::Wait>
{
 public:
	FindArucoTagNode(const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);
  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();
 private:
	  bool initialized_;
};

#endif
