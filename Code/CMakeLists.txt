CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(CHARLOTTE CXX)

FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(EIGEN3 eigen3>=3.2 REQUIRED)

SET(QPOASES_INCLUDE_DIRS "/home/facundo/dev/install/include")
SET(QPOASES_LIBRARIES "/home/facundo/dev/install/lib/libqpOASES.a")

INCLUDE_DIRECTORIES(
  ${EIGEN3_INCLUDE_DIRS}
  ${QPOASES_INCLUDE_DIRS}
)

ADD_EXECUTABLE(main src/main.cpp)
TARGET_LINK_LIBRARIES(main ${QPOASES_LIBRARIES})

