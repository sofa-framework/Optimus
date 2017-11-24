cmake_minimum_required(VERSION 2.8.12)
project(Optimus)

set(OPTIMUS_MAJOR_VERSION 0)
set(OPTIMUS_MINOR_VERSION 1)
set(OPTIMUS_VERSION ${OPTIMUS_MAJOR_VERSION}.${OPTIMUS_MINOR_VERSION})

find_package(SofaFramework REQUIRED)
find_package(SofaBase REQUIRED)
find_package(SofaGeneral REQUIRED)
find_package(SofaPardisoSolver QUIET)

set(LINKER_DEPENDENCIES SofaComponentBase SofaComponentGeneral SofaBaseTopology blas gsl)
set(COMPILATION_FLAGS "-DSOFA_BUILD_OPTIMUS -Wno-unused-local-typedef -Wno-deprecated")

set ( STOCHASTIC_FILTERING "0" CACHE BOOL "Set to activate the compilation the new filtering module replacing Verdandi" )

set(HEADER_FILES
    src/initOptimusPlugin.h
    src/genericComponents/OptimParams.h
    src/genericComponents/OptimParams.inl
    src/genericComponents/PointProjection.h
    src/genericComponents/PointProjection.inl
    src/genericComponents/ObservationSource.h
    src/genericComponents/SimulatedStateObservationSource.h
    src/genericComponents/SimulatedStateObservationSource.inl
    src/genericComponents/AddNoiseEngine.h
    src/genericComponents/AddNoiseEngine.inl
    src/genericComponents/Transform3dToRigid.inl
    src/genericComponents/Transform3dToRigid.h
   src/genericComponents/CorrectionForceField.h
   src/genericComponents/CorrectionForceField.inl
    )

set(SOURCE_FILES
    src/initOptimusPlugin.cpp
    src/genericComponents/OptimParams.cpp
    src/genericComponents/PointProjection.cpp
    src/genericComponents/SimulatedStateObservationSource.cpp
    src/genericComponents/AddNoiseEngine.cpp
    src/genericComponents/Transform3dToRigid.cpp
   src/genericComponents/CorrectionForceField.cpp
    )

set(README_FILES 
    Optimus.txt 
    )

if ( ${STOCHASTIC_FILTERING} )
	message(STATUS "Activating stochastic filtering")
	list(APPEND HEADER_FILES
		src/stochasticFiltering/FilteringAnimationLoop.h
		src/stochasticFiltering/StochasticFilterBase.h
		src/stochasticFiltering/ROUKFilter.h
		src/stochasticFiltering/ROUKFilter.inl
		src/stochasticFiltering/UKFilter.h
		src/stochasticFiltering/UKFilter.inl
		src/stochasticFiltering/UKFilterSimCorr.h
		src/stochasticFiltering/UKFilterSimCorr.inl
		src/stochasticFiltering/StochasticStateWrapperBase.h
		src/stochasticFiltering/EKFilter.h
		src/stochasticFiltering/EKFilter.inl
		src/stochasticFiltering/KFilter.h
		src/stochasticFiltering/KFilter.inl
		src/stochasticFiltering/StochasticStateWrapper.h
		src/stochasticFiltering/StochasticStateWrapper.inl
                src/stochasticFiltering/StochasticStateObservationWrapper.h
		src/stochasticFiltering/StochasticStateObservationWrapper.inl
		src/stochasticFiltering/ObservationManagerBase.h
		src/stochasticFiltering/MappedStateObservationManager.h
		src/stochasticFiltering/MappedStateObservationManager.inl
                src/stochasticFiltering/UKMappedStateObservationManager.h
		src/stochasticFiltering/UKMappedStateObservationManager.inl
		src/stochasticFiltering/SimpleObservationManager.h
		src/stochasticFiltering/SimpleObservationManager.inl
		src/stochasticFiltering/PreStochasticWrapper.h
		src/stochasticFiltering/ForceFieldSensitivity.h
	)

	list(APPEND SOURCE_FILES
		src/stochasticFiltering/FilteringAnimationLoop.cpp
		src/stochasticFiltering/ROUKFilter.cpp
		src/stochasticFiltering/UKFilter.cpp
		src/stochasticFiltering/UKFilterSimCorr.cpp
		src/stochasticFiltering/EKFilter.cpp
		src/stochasticFiltering/KFilter.cpp
		src/stochasticFiltering/StochasticStateWrapper.cpp
                src/stochasticFiltering/StochasticStateObservationWrapper.cpp
		src/stochasticFiltering/MappedStateObservationManager.cpp
                src/stochasticFiltering/UKMappedStateObservationManager.cpp
		src/stochasticFiltering/SimpleObservationManager.cpp
		src/stochasticFiltering/PreStochasticWrapper.cpp
	)

endif()


link_directories("/usr/local/gfortran/lib")
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

if(SofaPardisoSolver_FOUND)
    set(LINKER_DEPENDENCIES ${LINKER_DEPENDENCIES} SofaPardisoSolver )
else()
    message("SofaPardisoSolver not found, some scenes may not work")
endif()

target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>")
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${OPTIMUS_VERSION}")
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${COMPILATION_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")
target_link_libraries(${PROJECT_NAME} ${LINKER_DEPENDENCIES})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src")

get_target_property(compile_defs ${PROJECT_NAME} COMPILE_FLAGS)
message(STATUS "Compile definitions: ${compile_defs}")

install(TARGETS Optimus
        EXPORT  OptimusTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION include/Optimus)
