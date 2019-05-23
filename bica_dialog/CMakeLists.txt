cmake_minimum_required(VERSION 2.8.3)
project(bica_dialog)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dialogflow_ros_msgs
  roslint
  std_srvs
  std_msgs
  sound_play
  actionlib
  actionlib_msgs
  bica
  bica_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES bica_dialog
	CATKIN_DEPENDS roscpp dialogflow_ros_msgs actionlib_msgs
	DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(ROSLINT_CPP_OPTS "--filter=-build/header_guard,-runtime/references,-whitespace/braces,-build/c++11")
roslint_cpp()

add_library(bica_dialog
	src/DialogInterface.cpp
)

add_executable(pinger src/pinger.cpp)
add_executable(batterycontrol src/batterycontrol.cpp)
add_executable(main_DialogInterface src/main_DialogInterface.cpp)
add_executable(location_DialogInterface src/location_DialogInterface.cpp)
add_executable(order_DialogInterface src/order_DialogInterface.cpp)
add_executable(commands_DialogInterface src/commands_DialogInterface.cpp)
add_executable(talkPublisher src/talkPublisher.cpp)
add_executable(talkSubscriber src/talkSubscriber.cpp)
add_executable(locationsSubscriber src/locationsSubscriber.cpp)


add_dependencies(bica_dialog ${catkin_EXPORTED_TARGETS})
add_dependencies(pinger ${catkin_EXPORTED_TARGETS})
add_dependencies(batterycontrol ${catkin_EXPORTED_TARGETS})
add_dependencies(main_DialogInterface ${catkin_EXPORTED_TARGETS})
add_dependencies(location_DialogInterface ${catkin_EXPORTED_TARGETS})
add_dependencies(order_DialogInterface ${catkin_EXPORTED_TARGETS})
add_dependencies(commands_DialogInterface ${catkin_EXPORTED_TARGETS})
add_dependencies(talkPublisher ${catkin_EXPORTED_TARGETS})
add_dependencies(talkSubscriber ${catkin_EXPORTED_TARGETS})
add_dependencies(locationsSubscriber ${catkin_EXPORTED_TARGETS})


target_link_libraries(bica_dialog ${catkin_LIBRARIES})
target_link_libraries(batterycontrol ${catkin_LIBRARIES} bica_dialog)
target_link_libraries(pinger ${catkin_LIBRARIES} bica_dialog)
target_link_libraries(main_DialogInterface ${catkin_LIBRARIES} bica_dialog)
target_link_libraries(location_DialogInterface ${catkin_LIBRARIES} bica_dialog)
target_link_libraries(order_DialogInterface ${catkin_LIBRARIES} bica_dialog)
target_link_libraries(commands_DialogInterface ${catkin_LIBRARIES} bica_dialog)
target_link_libraries(talkPublisher ${catkin_LIBRARIES} bica_dialog)
target_link_libraries(talkSubscriber ${catkin_LIBRARIES} bica_dialog)
target_link_libraries(locationsSubscriber ${catkin_LIBRARIES} bica_dialog)


add_executable(example_df_node src/example/exampleDF.cpp)
add_dependencies(example_df_node ${catkin_EXPORTED_TARGETS} )
target_link_libraries(example_df_node ${catkin_LIBRARIES} bica_dialog)

install(TARGETS bica_dialog bica_dialog
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(${PROJECT_NAME}-test test/test_DialogInterface.launch test/test_DialogInterface.cpp)
  target_link_libraries(
    ${PROJECT_NAME}-test
    ${catkin_LIBRARIES}
    bica_dialog
  )
endif()
