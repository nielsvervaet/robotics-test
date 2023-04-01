#include <ros/ros.h> // ROS
#include <pick_place_task.h> // MTC pick/place demo implementation

constexpr char LOGNAME[] = "pick_place_demo_ur_robot";

int main(int argc, char** argv) {
  ros::init(argc, argv, "pick_place_demo_ur_robot");
  ros::NodeHandle nh, pnh("~");

  // Handle Task introspection requests from RViz & feedback during execution
  ros::AsyncSpinner spinner(1);
  spinner.start();

  mtc_ur_robot::setupDemoScene(pnh);

  // Construct and run pick/place task
  mtc_ur_robot::PickPlaceTask pick_place_task("pick_place_demo_ur_robot", pnh);
  if (!pick_place_task.init()) {
    ROS_INFO_NAMED(LOGNAME, "Initialization failed");
    return 1;
  }

  if (pick_place_task.plan()) {
    ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
    if (pnh.param("execute", false)) {
      pick_place_task.execute();
      ROS_INFO_NAMED(LOGNAME, "Execution complete");
    } else {
      ROS_INFO_NAMED(LOGNAME, "Execution disabled");
    }
  } else {
    ROS_INFO_NAMED(LOGNAME, "Planning failed");
  }

  ros::waitForShutdown();   // Keep introspection alive
  return 0;
}
