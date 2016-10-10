#include <reemc_walking_tutorials/LinkStatesListener.h>

LinkStatesListener::LinkStatesListener(ros::NodeHandle &nh, const std::vector<std::string> &link_names)
{
  link_listener_ = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 100, &LinkStatesListener::Callback, this);
  info_received_ = false;

  Eigen::Vector3d zero_vec;
  zero_vec.setZero();
  Eigen::Quaterniond id_orient;
  id_orient.setIdentity();

  for(size_t i = 0; i < link_names.size(); i++)
  {
    map_positions_[link_names[i]] = zero_vec;
    map_orientations_[link_names[i]] = id_orient;
  }
}

void LinkStatesListener::Callback(const gazebo_msgs::LinkStates::ConstPtr &msg)
{
  info_received_ = true;
  for(size_t i = 0; i < msg->name.size(); i++)
  {
    if(map_positions_.find(msg->name[i]) != map_positions_.end())
    {
      map_positions_[msg->name[i]] << msg->pose[i].position.x, msg->pose[i].position.y, msg->pose[i].position.z;
      map_orientations_[msg->name[i]] = Eigen::Quaterniond(msg->pose[i].orientation.w, msg->pose[i].orientation.x,
                                                           msg->pose[i].orientation.y, msg->pose[i].orientation.z);
    }
  }
}

bool LinkStatesListener::isListening() const
{
  return info_received_;
}

void LinkStatesListener::wait_until_Listener_ready() const
{
  while(!info_received_)
  {
    ros::spinOnce();
    sleep(0.1);
  }
}

Eigen::Vector3d LinkStatesListener::getMapPosition(const std::string &name) const
{
  auto it = map_positions_.find(name);
  if(it == map_positions_.end())
  {
    throw std::runtime_error("Map object not found");
  }
  return it->second;
}

Eigen::Quaterniond LinkStatesListener::getMapOrientation(const std::string &name) const
{
  auto it = map_orientations_.find(name);
  if(it == map_orientations_.end())
  {
    throw std::runtime_error("Map object not found");
  }
  return it->second;
}
