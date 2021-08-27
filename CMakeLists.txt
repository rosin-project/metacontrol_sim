cmake_minimum_required(VERSION 2.8.3)
project(metacontrol_sim)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ridgeback_description
  std_msgs
)

add_service_files(
  DIRECTORY
        srv
  FILES
  IncreaseConsumptionFactor.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/battery_load_control.py
  scripts/activate_observer.py
  scripts/objective_error_publisher.py
  scripts/safety_distance_publisher.py
  scripts/send_goal_from_yaml.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(
  ${catkin_INCLUDE_DIRS}
)
