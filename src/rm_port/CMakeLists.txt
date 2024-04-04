cmake_minimum_required(VERSION 3.10)
project(rm_port)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")   
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Protobuf REQUIRED)
ament_auto_find_build_dependencies()



add_subdirectory(EasySerialPorting)


set(ProtoDir protos)
execute_process(
        COMMAND rm Recieve.pb.cc
        COMMAND rm Recieve.pb.h
        COMMAND rm Send.pb.cc
        COMMAND rm Send.pb.h
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/${ProtoDir}
)
execute_process(
        COMMAND protoc --cpp_out=. Recieve.proto
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/${ProtoDir}
)
execute_process(
        COMMAND protoc --cpp_out=. Send.proto
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/${ProtoDir}
)
#设置Proto目录 cmake将自动调用protoc生成文件
set(ProtoFiles
        ${ProtoDir}/Recieve.pb.cc
        ${ProtoDir}/Recieve.pb.h
        ${ProtoDir}/Send.pb.cc
        ${ProtoDir}/Send.pb.h
        )


ament_auto_add_library(${PROJECT_NAME} SHARED
        src/rm_port.cpp 
        include/rm_port/rm_port.hpp
        ${ProtoFiles}
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        ${PROTOBUF_INCLUDE_DIRS}
        ${ProtoDir}
        EasySerialPorting
        include
        )

target_link_libraries(${PROJECT_NAME}
        ${PROTOBUF_LIBRARIES}
        EasySerialPorting
)

rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN auto_aim_port::rm_port
        EXECUTABLE ${PROJECT_NAME}_node
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
        INSTALL_TO_SHARE
        config
        launch
)

