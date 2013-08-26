cmake_minimum_required(VERSION 2.8)

project("SofaOptimus")

include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(HEADER_FILES

    	src/initOptimusPlugin.h
	src/OptimParams.h
	src/OptimParams.inl
	src/ekfilter.hpp
	src/ekfilter_impl.inl
	src/kfilter.hpp
	src/kfilter_impl.inl
	src/KalmanFilter.h
	src/KalmanFilter.inl
	src/TestingParams.h
	src/BubblePackingForceField.h
	src/BubblePackingForceField.inl

	)

set(SOURCE_FILES

    	src/initOptimusPlugin.cpp
	src/OptimParams.cpp
	src/KalmanFilter.cpp
	src/TestingParams.cpp
	src/BubblePackingForceField.cpp

	)



set(README_FILE "Optimus.txt" )	


set(COMPILER_DEFINES "SOFA_BUILD_OPTIMUS")
set(LINKER_DEPENDENCIES SofaBaseLinearSolver )



include(${SOFA_CMAKE_DIR}/postProject.cmake)
