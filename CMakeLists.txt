cmake_minimum_required(VERSION 2.8)
PROJECT( MyProject )

FIND_PACKAGE(RTABMap REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(PCL 1.7 REQUIRED)

# Find Qt5 first
FIND_PACKAGE(Qt5 COMPONENTS Widgets Core Gui Svg QUIET)
IF(NOT Qt5_FOUND)
   FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtSvg)
ENDIF(NOT Qt5_FOUND)

# fix libproj.so not found on Xenial
if(NOT "${PCL_LIBRARIES}" STREQUAL "")
   list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
endif()

if(POLICY CMP0020)
   cmake_policy(SET CMP0020 OLD)
endif()

# include directories
SET(INCLUDE_DIRS
    ${RTABMap_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

IF(QT4_FOUND)
    INCLUDE(${QT_USE_FILE})
ENDIF(QT4_FOUND)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# libraries
SET(LIBRARIES
    ${RTABMap_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${QT_LIBRARIES} 
    ${PCL_LIBRARIES}
)

IF(QT4_FOUND)
    QT4_WRAP_CPP(moc_srcs MapBuilder.h)
ELSE()
    QT5_WRAP_CPP(moc_srcs MapBuilder.h)
ENDIF()

ADD_EXECUTABLE(rgbd_mapping main.cpp ${moc_srcs})
ADD_EXECUTABLE(resample resampling.cpp)
ADD_EXECUTABLE(mesh mesh.cpp)
  
TARGET_LINK_LIBRARIES(rgbd_mapping ${LIBRARIES})
TARGET_LINK_LIBRARIES(resample ${LIBRARIES})
TARGET_LINK_LIBRARIES(mesh ${LIBRARIES})

