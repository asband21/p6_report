cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(point_cloud_visual)

# Find required packages
find_package(PCL 1.2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(VTK 9.1 REQUIRED COMPONENTS 
  ChartsCore CommonColor CommonComputationalGeometry CommonCore CommonDataModel
  CommonExecutionModel CommonMath CommonMisc CommonTransforms FiltersCore
  FiltersExtraction FiltersGeneral FiltersGeometry FiltersModeling FiltersSources
  ImagingCore ImagingSources InteractionImage InteractionStyle InteractionWidgets
  IOCore IOGeometry IOImage IOLegacy IOPLY RenderingAnnotation RenderingCore
  RenderingContext2D RenderingLOD RenderingFreeType ViewsCore ViewsContext2D
  RenderingOpenGL2 GUISupportQt RenderingContextOpenGL2)

# Include directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${VTK_INCLUDE_DIRS})

# Definitions and link directories
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# Create the executable
add_executable(visual visual.cpp)
target_link_libraries(visual ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${VTK_LIBRARIES})

