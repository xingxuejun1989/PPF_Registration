# Specify the minimun camke version
CMAKE_MINIMUM_REQUIRED(VERSION 3.10 FATAL_ERROR)

# Create a project.
PROJECT(PPF)

#Start support for the C++ standard
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
# The explicit requirement indicates that the C++ standard is supported.
SET(CMAKE_CXX_STANDARD 14)
# 
SET(CMAKE_C_STANDARD 14)
# The explicit requirement indicates that the C standard is supported,
SET(CMAKE_C_STANDARD_REQUIRED ON)


# Find third-party dependent libraries.
FIND_PACKAGE(PCL 1.12 REQUIRED)

# Add an executable file.
ADD_EXECUTABLE(PPF src/main.cpp)

# Add header file directory
TARGET_INCLUDE_DIRECTORIES(PPF PRIVATE ${PCL_INCLUDE_DIRS})

# Add library directory
TARGET_LINK_DIRECTORIES(PPF PRIVATE ${PCL_LIBRARY_DIRS})

# Add compiler definition
TARGET_COMPILE_DEFINITIONS(PPF PRIVATE ${PCL_DEFINITIONS})

# Cancel the existing compiler definition.
SET_TARGET_PROPERTIES(PPF PROPERTIES COMPILE_FLAGS "/UBOOST_ALL_NO_LIB-DBOOST_ALL_NO_LIB")

# 添加依赖库文件
target_link_libraries(PPF PRIVATE ${PCL_LIBRARIES})
