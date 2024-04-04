cmake_minimum_required(VERSION 3.10)
project(ros2_galaxy_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)


# find dependencies

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########
ament_auto_add_library(${PROJECT_NAME} SHARED
        DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC 
        include
        sdk/GX_include
        )
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ros2_galaxy_camera::galaxy_camera_node
  EXECUTABLE ${PROJECT_NAME}_node
)


if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        target_link_directories(${PROJECT_NAME} PUBLIC 
                sdk/GX_lib/x86/
                )
        
        target_link_libraries(${PROJECT_NAME}
                libgxiapi.so
        )
        install(
                DIRECTORY sdk/GX_lib/x86/
                DESTINATION lib
        )

elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        target_link_directories(${PROJECT_NAME} PUBLIC
                sdk/GX_lib/arm/
                )
        target_link_libraries(${PROJECT_NAME}
                libgxiapi.so
        )
        install(
                DIRECTORY sdk/GX_lib/arm/
                DESTINATION lib
        )
else()
        message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()



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
        launch
        config
)

