cmake_minimum_required(VERSION 3.8)
project(susy_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


install(
   DIRECTORY urdf launch models 3Dmodels rviz worlds photos
   DESTINATION share/${PROJECT_NAME}
)


find_package(ament_cmake REQUIRED)


ament_package()
