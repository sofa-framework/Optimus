cmake_minimum_required(VERSION 3.1)
project(Optimus)

set(OPTIMUS_MAJOR_VERSION 1)
set(OPTIMUS_MINOR_VERSION 1)
set(OPTIMUS_VERSION ${OPTIMUS_MAJOR_VERSION}.${OPTIMUS_MINOR_VERSION})

# required standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(SofaFramework REQUIRED)
find_package(SofaSimulation REQUIRED)
find_package(SofaBase REQUIRED)
find_package(SofaGeneralAnimationLoop REQUIRED)
find_package(SofaGeneralEngine REQUIRED)
find_package(SofaImplicitOdeSolver REQUIRED)
find_package(SofaGeneralLinearSolver REQUIRED)
find_package(SofaGeneralRigid REQUIRED)
find_package(SofaSimpleFem REQUIRED)
find_package(SofaGeneralTopology REQUIRED)
find_package(SofaTopologyMapping REQUIRED)
find_package(SofaUserInteraction REQUIRED)
find_package(SofaConstraint REQUIRED)
find_package(SofaGeneralLoader REQUIRED)
find_package(SofaMiscEngine REQUIRED)
find_package(SofaMiscForceField REQUIRED)
find_package(SofaMiscMapping REQUIRED)
find_package(SofaMiscSolver REQUIRED)
find_package(SofaMiscTopology REQUIRED)
find_package(SofaMiscFem REQUIRED)
find_package(SofaExporter REQUIRED)
find_package(SofaGui REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


include_directories(${EIGEN3_INCLUDE_DIR})
set(LINKER_DEPENDENCIES SofaCore SofaBase SofaGeneralAnimationLoop SofaGeneralEngine SofaImplicitOdeSolver SofaGeneralLinearSolver SofaGeneralRigid SofaBoundaryCondition SofaBaseTopology SofaExporter SofaUserInteraction SofaConstraint SofaSimpleFem SofaGeneralTopology SofaTopologyMapping SofaUserInteraction SofaConstraint SofaGeneralLoader SofaMiscEngine SofaMiscFem SofaMiscForceField SofaMiscMapping SofaMiscSolver SofaMiscTopology SofaGuiCommon blas pthread Eigen3::Eigen)
set(COMPILATION_FLAGS "-DSOFA_BUILD_OPTIMUS -Wno-unused-local-typedef -Wno-deprecated")

set(STOCHASTIC_FILTERING "1" CACHE BOOL "Set to activate the compilation of the filtering module" FORCE)

set(HEADER_FILES
    src/initOptimusPlugin.h
    src/genericComponents/TimeProfiling.h
    src/genericComponents/ShowSpheres.h
    src/genericComponents/ShowSpheres.inl
    src/genericComponents/OptimParams.h
    src/genericComponents/OptimParams.inl
    src/genericComponents/PointProjection.h
    src/genericComponents/PointProjection.inl
    src/genericComponents/ObservationSource.h
    src/genericComponents/SimulatedStateObservationSource.h
    src/genericComponents/SimulatedStateObservationSource.inl
    src/genericComponents/SimulatedStateObservationStreamer.h
    src/genericComponents/SimulatedStateObservationStreamer.inl
    src/genericComponents/AddNoiseEngine.h
    src/genericComponents/AddNoiseEngine.inl
    src/genericComponents/CorrectionForceField.h
    src/genericComponents/CorrectionForceField.inl
    src/genericComponents/TransformStochasticEngine.h
    src/genericComponents/TransformStochasticEngine.inl
    src/genericComponents/FilteringUpdateEngine.h
    src/genericComponents/FilteringUpdateEngine.inl
    src/genericComponents/StochasticPositionHandler.h
    src/genericComponents/StochasticPositionHandler.inl
    src/genericComponents/FilterEvents.h
    src/genericComponents/VTKExporterDA.h
    src/genericComponents/SigmaPointsVTKExporter.h
    src/genericComponents/OptimMonitor.h
    src/genericComponents/OptimMonitor.inl
    src/genericComponents/Indices2ValuesTransformer.h
    src/genericComponents/Indices2ValuesTransformer.inl
    src/genericComponents/StepPCGLinearSolver.h
    src/genericComponents/MatrixCovariance.h
    src/genericComponents/MatrixCovariance.inl
    src/test/GeoEmulator.h
    src/test/GeoEmulator.inl
    src/test/GeoListener.h
    src/test/GeoListener.inl
    )

set(SOURCE_FILES
    src/initOptimusPlugin.cpp
    src/genericComponents/ShowSpheres.cpp
    src/genericComponents/OptimParams.cpp
    src/genericComponents/PointProjection.cpp
    src/genericComponents/SimulatedStateObservationSource.cpp
    src/genericComponents/SimulatedStateObservationStreamer.cpp
    src/genericComponents/AddNoiseEngine.cpp
    src/genericComponents/CorrectionForceField.cpp
    src/genericComponents/TransformStochasticEngine.cpp
    src/genericComponents/FilteringUpdateEngine.cpp
    src/genericComponents/StochasticPositionHandler.cpp
    src/genericComponents/FilterEvents.cpp
    src/genericComponents/VTKExporterDA.cpp
    src/genericComponents/SigmaPointsVTKExporter.cpp
    src/genericComponents/OptimMonitor.cpp
    src/genericComponents/Indices2ValuesTransformer.cpp
    src/genericComponents/StepPCGLinearSolver.cpp
    src/genericComponents/MatrixCovariance.cpp
    src/test/GeoEmulator.cpp
    src/test/GeoListener.cpp
    )

set(README_FILES 
    ReadMe.md
    )

if ( ${STOCHASTIC_FILTERING} )
    message(STATUS "Activating stochastic filtering")
    list(APPEND HEADER_FILES
        src/stochasticFiltering/FilteringAnimationLoop.h
        src/stochasticFiltering/StochasticFilterBase.h
        src/stochasticFiltering/ROUKFilter.h
        src/stochasticFiltering/ROUKFilter.inl
        src/stochasticFiltering/UKFilterSimCorr.h
        src/stochasticFiltering/UKFilterSimCorr.inl
        src/stochasticFiltering/UKFilterClassic.h
        src/stochasticFiltering/UKFilterClassic.inl
        src/stochasticFiltering/EnTKFilter.h
        src/stochasticFiltering/EnTKFilter.inl
        src/stochasticFiltering/UKFilterClassicWithSVD.h
        src/stochasticFiltering/UKFilterClassicWithSVD.inl
        src/stochasticFiltering/AdaptativeUKFilterClassic.h
        src/stochasticFiltering/AdaptativeUKFilterClassic.inl
        src/stochasticFiltering/StochasticStateWrapperBase.h
        src/stochasticFiltering/StochasticStateWrapper.h
        src/stochasticFiltering/StochasticStateWrapper.inl
        src/stochasticFiltering/ObservationManagerBase.h
        src/stochasticFiltering/MappedStateObservationManager.h
        src/stochasticFiltering/MappedStateObservationManager.inl
        src/stochasticFiltering/MappedStateVelocityObservationManager.h
        src/stochasticFiltering/MappedStateVelocityObservationManager.inl
        src/stochasticFiltering/SimpleObservationManager.h
        src/stochasticFiltering/SimpleObservationManager.inl
        src/stochasticFiltering/SimpleUncorrespondentObservationManager.h
        src/stochasticFiltering/SimpleUncorrespondentObservationManager.inl
        src/stochasticFiltering/BindedSimpleObservationManager.h
        src/stochasticFiltering/BindedSimpleObservationManager.inl
        src/stochasticFiltering/PreStochasticWrapper.h
        src/stochasticFiltering/ForceFieldSensitivity.h
    )

    list(APPEND SOURCE_FILES
        src/stochasticFiltering/FilteringAnimationLoop.cpp
        src/stochasticFiltering/ROUKFilter.cpp
        src/stochasticFiltering/UKFilterSimCorr.cpp
        src/stochasticFiltering/UKFilterClassic.cpp
        src/stochasticFiltering/EnTKFilter.cpp
        src/stochasticFiltering/UKFilterClassicWithSVD.cpp
        src/stochasticFiltering/AdaptativeUKFilterClassic.cpp
        src/stochasticFiltering/StochasticStateWrapper.cpp
        src/stochasticFiltering/MappedStateObservationManager.cpp
        src/stochasticFiltering/MappedStateVelocityObservationManager.cpp
        src/stochasticFiltering/SimpleObservationManager.cpp
        src/stochasticFiltering/SimpleUncorrespondentObservationManager.cpp
        src/stochasticFiltering/BindedSimpleObservationManager.cpp
        src/stochasticFiltering/PreStochasticWrapper.cpp
    )

endif()


add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

if(SofaPardisoSolver_FOUND)
    set(LINKER_DEPENDENCIES ${LINKER_DEPENDENCIES} SofaPardisoSolver)
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
