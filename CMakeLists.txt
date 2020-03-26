CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT(AutoAtlas)

set (STATISMO_DIR "/usr/local/lib/cmake/statismo")

FIND_PACKAGE(STATISMO REQUIRED)
FIND_PACKAGE(VTK REQUIRED)
FIND_PACKAGE(ITK REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS system thread filesystem)

add_definitions("-O3 -g")

include (${VTK_USE_FILE})
include (${ITK_USE_FILE})

find_library(STATISMO_CORE_LIB statismo_core)
find_library(STATISMO_VTK_LIB statismo_VTK)

include_directories(${STATISMO_INCLUDE_DIRS}) 
link_directories( ${ITK_LIBRARY_DIRS} ${VTK_LIBRARY_DIRS} ${STATISMO_LIBRARY_DIRS})

add_library(FileIO OBJECT FileIO.cpp)
add_library(utils OBJECT utils.cpp)
add_library(GaussModel OBJECT GaussModel.cpp)
add_library(Landmark OBJECT Landmark.cpp)
add_library(Bone OBJECT Bone.cpp)
add_library(BoneManager OBJECT BoneManager.cpp)
add_library(SingleBoneTypeManager OBJECT SingleBoneTypeManager.cpp)
add_library(GaussianKernel OBJECT GaussianKernel.cpp)
add_library(MultiscaleGaussianKernel OBJECT MultiscaleGaussianKernel.cpp)
add_library(ConfigUtils OBJECT ConfigUtils.cpp)

add_executable (AtlasCreation AtlasCreation.cpp $<TARGET_OBJECTS:utils> $<TARGET_OBJECTS:FileIO> $<TARGET_OBJECTS:GaussModel> $<TARGET_OBJECTS:Landmark> $<TARGET_OBJECTS:Bone> $<TARGET_OBJECTS:BoneManager> $<TARGET_OBJECTS:SingleBoneTypeManager> $<TARGET_OBJECTS:GaussianKernel> $<TARGET_OBJECTS:MultiscaleGaussianKernel>) 
target_link_libraries (AtlasCreation ${STATISMO_LIBRARIES} ${VTK_LIBRARIES} ${ITK_LIBRARIES} ${STATISMO_CORE_LIB} ${STATISMO_VTK_LIB} ${Boost_LIBRARIES})

add_executable (ConfigCreate ConfigCreate.cpp $<TARGET_OBJECTS:ConfigUtils>) 
target_link_libraries (ConfigCreate ${Boost_LIBRARIES})

add_executable (ConfigFit ConfigFit.cpp $<TARGET_OBJECTS:ConfigUtils>) 
target_link_libraries (ConfigFit ${Boost_LIBRARIES})

add_executable (GenerateTests GenerateTests.cpp) 
target_link_libraries (GenerateTests ${Boost_LIBRARIES})

add_executable (Statistics Statistic.cpp AbstractStatistics.cpp LandmarkStatistics.cpp BoneStatistics.cpp) 
target_link_libraries (Statistics ${Boost_LIBRARIES})
target_compile_options(Statistics PRIVATE "-std=c++11")

add_executable (Tune Tune.cpp) 
target_link_libraries (Tune ${Boost_LIBRARIES} ${ITK_LIBRARIES})

add_executable (FitAtlas AtlassFit.cpp $<TARGET_OBJECTS:utils> $<TARGET_OBJECTS:FileIO> $<TARGET_OBJECTS:Landmark>) 
target_link_libraries (FitAtlas ${STATISMO_LIBRARIES} ${VTK_LIBRARIES} ${ITK_LIBRARIES} ${STATISMO_CORE_LIB} ${STATISMO_VTK_LIB} ${Boost_LIBRARIES})
