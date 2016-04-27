set(SOURCE_FILES 
 "HRBFNode.cpp"
 "HRBFNode.h"
 "MayaDualQuaternion.cpp"
 "MayaDualQuaternion.h"
 "MayaHRBFManager.cpp"
 "MayaHRBFManager.h"
 "MayaHRBF.cpp"
 "MayaHRBF.h" 
 "FloatGrid3D.hpp"
 "FloatGrid3D.cpp"
)

find_package(Maya REQUIRED)

include_directories(${MAYA_INCLUDE_DIR})
link_directories(${MAYA_LIBRARY_DIR})
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${MAYA_LIBRARIES})

MAYA_PLUGIN(${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} ${MAYA_TARGET_TYPE} DESTINATION plug-ins)