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


include_directories(
  ${catkin_INCLUDE_DIRS}
)
