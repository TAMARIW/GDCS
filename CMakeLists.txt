cmake_minimum_required(VERSION 3.5.0)
project(tmwstm32 VERSION 0.1.0)

#add RODOS
add_subdirectory(lib/rodos)

#Collect datalink source files add them to the executable
file(GLOB_RECURSE SRC_FILES src/*.cpp)
add_executable(${PROJECT_NAME} ${SRC_FILES})

#Create the datalink executable
target_link_libraries(${PROJECT_NAME} PUBLIC rodos)

#Add ORPE header files
target_include_directories(${PROJECT_NAME} PUBLIC lib/ORPE/include/)