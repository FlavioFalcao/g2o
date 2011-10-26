find_package(ecto)
if(NOT ecto_FOUND)
  return()
endif()

find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)
find_package(image_pipeline 0.1.0 REQUIRED)
include_directories(${image_pipeline_INCLUDE_DIRS})

# target_link_libraries(my_thing ${image_pipeline_LIBRARIES})

ecto_python_env_gen(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

ectomodule(g2o
   module.cpp
   Sba.cpp
   SbaDisparity.cpp
   pe3d.cpp
)

link_ecto(g2o
  core
  solver_cholmod
  types_icp
  image_pipeline
)

install_ecto_module(g2o)
