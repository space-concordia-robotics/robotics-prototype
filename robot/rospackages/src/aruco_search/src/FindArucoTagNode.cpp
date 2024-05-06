#include "aruco_search/FindArucoTagNode.hpp"
FindArucoTagNode::FindArucoTagNode(const std::string& xml_tag_name,
	const std::string& action_name,
	const BT::NodeConfiguration& conf) : BtActionNode<nav2_msgs::action::Wait>(xml_tag_name, action_name, conf),
										 initialized_(false)
{
	
}