cmake_minimum_required(VERSION 3.0)

project(Example_TorqueControl)
#set(Cohoma ..//..//..)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Define command line options.
add_definitions(-DMODE=${MODE})
add_definitions(-DUNICODE)

# Choose architecture on UNIX
if (UNIX)
    add_definitions(-DARCH=${ARCH}) 
endif (UNIX)

include_directories(_HEADERS)
#include_directories(TrajGen/include/TrajGen)

SET(SRC_FILES
   src/Example_TorqueControl.cpp
)


ADD_EXECUTABLE(${PROJECT_NAME}
	${SRC_FILES}
)

#ADD_EXECUTABLE(Test_TorqueControl
#	${SRC_FILES2}
#)

if (UNIX)
    target_link_libraries(Example_TorqueControl ${CMAKE_DL_LIBS})
    #target_link_libraries(Test_TorqueControl ${CMAKE_DL_LIBS})
endif (UNIX)

