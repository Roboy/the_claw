cmake_minimum_required(VERSION 2.8.3)
project( the_claw_control )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g")

find_package(catkin REQUIRED COMPONENTS common_utilities roscpp roslint rqt_gui rqt_gui_cpp std_msgs roboy_communication_middleware )
FIND_PACKAGE(yaml-cpp REQUIRED)

include_directories( include qcustomplot)

## System dependencies are found with CMake's conventions

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

roslint_cpp()

catkin_package( INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs roboy_communication_middleware common_utilities 
        DEPENDS YAML_CPP)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(the_claw_control_MOCS include/the_claw_control/the_claw_control.hpp qcustomplot/qcustomplot.h)
  qt5_wrap_ui(roboy_control_UIS_H src/the_claw_control.ui)
else()
  qt4_wrap_cpp(roboy_control_MOCS include/the_claw_control/the_claw_control.hpp qcustomplot/qcustomplot.h)
  qt4_wrap_ui(roboy_control_UIS_H src/the_claw_control.ui)
endif()

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_library( the_claw_control src/the_claw_control.cpp ${roboy_control_MOCS} ${roboy_control_UIS_H}
        qcustomplot/qcustomplot.cpp)

add_dependencies(the_claw_control roboy_communication_middleware_gencpp)

target_link_libraries(the_claw_control ${catkin_LIBRARIES} )

target_link_libraries(the_claw_control ${YAML_CPP_LIBRARIES} -ldl)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(the_claw_control Qt5::Widgets)
else()
  target_link_libraries(the_claw_control ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

#############
## Install ##
#############

install(TARGETS the_claw_control
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/roboy_rqt_plugins
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )