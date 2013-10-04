cmake_minimum_required(VERSION 2.8)

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

set(README_FILES 
	Optimus.txt 
)	

if(SOFA-EXT_VERDANDI)
    list(APPEND HEADER_FILES
        verdandiSofa/VerdandiAnimationLoop.h
        verdandiSofa/SofaModelWrapper.h
    )

    list(APPEND SOURCE_FILES
        verdandiSofa/VerdandiAnimationLoop.cpp
        verdandiSofa/SofaModelWrapper.cpp
    )

    include_directories(${SOFA_EXTLIBS_DIR}/verdandi-1.5)
    include_directories(${SOFA_EXTLIBS_DIR}/verdandi-1.5/share)
    include_directories(${SOFA_EXTLIBS_DIR}/verdandi-1.5/include)
    include_directories(${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/lua/src)

    AddLinkerDependencies(lua5.1)
endif()

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${MOC_FILES} ${SOURCE_FILES} ${README_FILES})

AddCompilerDefinitions("SOFA_BUILD_OPTIMUS")
AddLinkerDependencies( SofaBaseLinearSolver )



include(${SOFA_CMAKE_DIR}/postProject.cmake)
