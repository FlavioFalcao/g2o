find_package(ecto)
if(NOT ecto_FOUND)
  return()
endif()

find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)
find_package(ecto_image_pipeline REQUIRED)
include_directories(${ecto_image_pipeline_INCLUDE_DIRS})

ectomodule(g2o
           module.cpp
           Sba.cpp
           SbaDisparity.cpp
)

link_ecto(g2o
          core
          solver_cholmod
          ${ecto_image_pipeline_LIBRARIES}
          types_icp
)

install_ecto_module(g2o)
