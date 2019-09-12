cmake_minimum_required(VERSION 2.8.3)
project(charpkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(EIGEN3 eigen3>=3.2 REQUIRED)

SET(QPOASES_INCLUDE_DIRS "/home/facundo/dev/install/include")
SET(QPOASES_LIBRARIES "/home/facundo/dev/install/lib/libqpOASES.a")


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
# include
  ${EIGEN3_INCLUDE_DIRS}
  ${QPOASES_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

ADD_EXECUTABLE(main src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(main
   ${catkin_LIBRARIES}
   ${QPOASES_LIBRARIES}
   
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_charpkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
