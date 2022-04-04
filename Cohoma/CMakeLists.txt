cmake_minimum_required(VERSION 3.0)

project(Example_CartesianControl)
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
include_directories(TrajGen/include/TrajGen)

SET(SRC_FILES
    Example_CartesianControl/src/Example_CartesianControl.cpp
    Example_CartesianControl/src/pid.cpp
)

SET(SRC_FILES2
    Traj_CartesianOL/src/Test_TrajOL.cpp
)

ADD_EXECUTABLE(${PROJECT_NAME}
	${SRC_FILES}
)

ADD_EXECUTABLE(Test_CartesianOL
	${SRC_FILES2}
)

if (UNIX)
    target_link_libraries(Example_CartesianControl ${CMAKE_DL_LIBS})
    target_link_libraries(Test_CartesianOL ${CMAKE_DL_LIBS})
endif (UNIX)

