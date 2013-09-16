#include <ros/ros.h>
#include <sstream>
#include <walking/WalkSteps.h>

const std::string WALK_STEPS_SERVICE = "/walking_controller/walk_steps";

int main(int argc, char **argv)
{

  int nsteps = 0;
  double step_length = 0.0;
  double step_time = 0.0;

  ros::init(argc, argv, "walking_client_example");
  ros::NodeHandle n;

  std::ostringstream usage;
  usage << "Usage: " << argv[0];

  if (4 != argc)
  {
    std::cout << usage.str() <<" [nsteps] [step_lenght] [step_time]\n";
    return EXIT_FAILURE;
  }

  if (4 == argc)
  {
    try
    {
      nsteps = boost::lexical_cast<int>(argv[1]);
      step_length = boost::lexical_cast<double>(argv[2]);
      step_time = boost::lexical_cast<double>(argv[3]);
    }
    catch(boost::bad_lexical_cast &)
    {
      std::cout << usage.str() << std::endl;
      return EXIT_FAILURE;
    }
  }

  ros::ServiceClient client = n.serviceClient<walking::WalkSteps>(WALK_STEPS_SERVICE);
  walking::WalkSteps srv;
  srv.request.nsteps = nsteps;
  srv.request.step_length = step_length;
  srv.request.step_time = step_time;

  if (client.call(srv))
  {
    ROS_INFO("Succesfully called service WalkSteps");
  }
  else
  {
    ROS_ERROR("Failed to call service WalkSteps");
    return 1;
  }

  return 0;
}

