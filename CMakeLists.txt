cmake_minimum_required(VERSION 2.8.12)
project(Optimus)

set(OPTIMUS_MAJOR_VERSION 0)
set(OPTIMUS_MINOR_VERSION 1)
set(OPTIMUS_VERSION ${OPTIMUS_MAJOR_VERSION}.${OPTIMUS_MINOR_VERSION})

find_package(SofaFramework REQUIRED)
find_package(SofaBase REQUIRED)
find_package(SofaGeneral REQUIRED)
find_package(SofaPardisoSolver QUIET)

set(LINKER_DEPENDENCIES SofaComponentBase SofaComponentGeneral SofaBaseTopology )
set(COMPILATION_FLAGS -DSOFA_BUILD_OPTIMUS)

set ( SOFA-EXTERNAL_VERDANDI "0" CACHE BOOL "Set to activate the compilation of filtering using Verdandi" )
set ( STOCHASTIC_FILTERING "0" CACHE BOOL "Set to activate the compilation the new filtering module replacing Verdandi" )

set(HEADER_FILES
    src/initOptimusPlugin.h
    src/OptimParams.h
    src/OptimParams.inl
#    src/ekfilter.hpp
#    src/ekfilter_impl.inl
#    src/kfilter.hpp
#    src/kfilter_impl.inl
#    src/KalmanFilter.h
#    src/KalmanFilter.inl
#    src/TestingParams.h
#    src/BubblePackingForceField.h
#    src/BubblePackingForceField.inl 
#    src/PointProjection.h
#    src/PointProjection.inl
    )

set(SOURCE_FILES
    src/initOptimusPlugin.cpp
    src/OptimParams.cpp
#    src/PointProjection.cpp
#    src/KalmanFilter.cpp
#    src/TestingParams.cpp
#    src/BubblePackingForceField.cpp
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
		src/stochasticFiltering/StochasticStateWrapperBase.h
		src/stochasticFiltering/StochasticStateWrapper.h
		src/stochasticFiltering/StochasticStateWrapper.inl
	)

	list(APPEND SOURCE_FILES
		src/stochasticFiltering/FilteringAnimationLoop.cpp
		src/stochasticFiltering/ROUKFilter.cpp
		src/stochasticFiltering/StochasticStateWrapper.cpp
	)

endif()



if(${SOFA-EXTERNAL_VERDANDI})
	set(SOFA-EXTERNAL_VERDANDI_PATH "verdandiSofa/verdandi-1.5")

	find_package(BLAS REQUIRED)
	find_package(LAPACK REQUIRED)
        #message(WARNING "Compiling Optimus with verdandi ${SOFA-EXTERNAL_VERDANDI} at path ${SOFA-EXTERNAL_VERDANDI_PATH}")
	# we probably need to compile LUA
	add_subdirectory(extlibs/lua)

	message(STATUS "Compiling Optimus with verdandi available in ${SOFA-EXTERNAL_VERDANDI_PATH}")


    list(APPEND HEADER_FILES

        verdandiSofa/VerdandiAnimationLoop.h
        verdandiSofa/SofaModelWrapper.h
        verdandiSofa/SofaModelWrapper.inl
        verdandiSofa/SofaModelWrapperParallel.h
        verdandiSofa/SofaModelWrapperParallel.inl
        verdandiSofa/VerdandiClasses.h
        ##verdandiSofa/VerdandiROUKFParams.h
        ##verdandiSofa/VerdandiObservationParams.h
        verdandiSofa/ObservationSource.h
        verdandiSofa/SimulatedStateObservationSource.h
        verdandiSofa/SimulatedStateObservationSource.inl
        ${SOFA-EXTERNAL_VERDANDI_PATH}/Verdandi.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/model/QuadraticModel.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/model/ClampedBar.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/model/ModelTemplate.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/model/CheckingModel.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/model/Lorenz.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/model/ShallowWater.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/model/PetscClampedBar.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/model/PythonModel.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/share/VerdandiOps.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/share/Functions_Vector3.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/share/MessageHandler.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/share/LockFile.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/share/Functions_Vector2.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/share/Error.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/share/OutputSaver.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/share/VerdandiBase.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/share/Logger.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/share/Variable.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/share/UsefulFunction.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/sequential_aggregation/BaseForecaster.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/sequential_aggregation/Aggregate.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/sequential_aggregation/DiscountedRidgeRegression.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/error/DiagonalMatrix.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/error/IsotropicBalgovindMatrix.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/error/BalgovindMatrix.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/VerdandiHeader.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/method/ReducedMinimax.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/method/TRNGPerturbationManager.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/method/EnsembleKalmanFilter.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/method/SigmaPoint.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/method/ExtendedKalmanFilter.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/method/NewranPerturbationManager.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/method/FourDimensionalVariational.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/method/BasePerturbationManager.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/method/ReducedOrderUnscentedKalmanFilter.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/method/HamiltonJacobiBellman.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/method/ReducedOrderExtendedKalmanFilter.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/method/ObservationGenerator.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/method/MonteCarlo.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/method/ForwardDriver.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/method/UnscentedKalmanFilter.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/method/BLUE.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/method/TR1PerturbationManager.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/method/TrajectoryManager.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/method/OptimalInterpolation.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/method/chi_2.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/observation_manager/GridToNetworkObservationManager.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/observation_manager/ObservationManagerTemplate.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/observation_manager/LinearObservationManager.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/observation_manager/ObservationAggregator.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/observation_manager/PetscLinearObservationManager.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/observation_manager/PythonObservationManager.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/ops/OpsHeader.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/ops/ClassOps.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/ops/Ops.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/ops/Error.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/optimization/NLopt.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/optimization/NLoptSolver.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/basic_functions/Functions_Vector.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/basic_functions/Functions_Matrix.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/basic_functions/Functions_MatVect.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/solver/iterative/Iterative.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/solver/SparseCholeskyFactorisation.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/solver/SparseSolver.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/solver/preconditioner/IlutPreconditioning.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/solver/Ordering.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/interfaces/Blas_1.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/interfaces/Lapack_LeastSquares.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/interfaces/Blas_2.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/interfaces/Blas_3.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/interfaces/direct/Mumps.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/interfaces/direct/Pastix.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/interfaces/direct/SuperLU.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/interfaces/direct/UmfPack.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/interfaces/direct/Cholmod.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/interfaces/Lapack_Eigenvalues.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/interfaces/Lapack_LinearEquations.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/share/Properties.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/share/Allocator.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/share/Errors.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/share/Common.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/share/Storage.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/share/MatrixFlag.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix/Matrix_Triangular.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix/Matrix_Base.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix/Matrix_HermPacked.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix/Matrix_Pointers.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix/MatrixCollection.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix/Matrix_TriangPacked.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix/Matrix_Hermitian.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix/Matrix_SymPacked.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix/SubMatrix_Base.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix/PetscMatrix.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix/Functions.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix/SubMatrix.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix/HeterogeneousMatrixCollection.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix/Matrix_Symmetric.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/Seldon.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/array3d/Array3D.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/SeldonSolver.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix_sparse/Matrix_Conversions.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix_sparse/Matrix_ComplexSparse.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix_sparse/IOMatrixMarket.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix_sparse/Matrix_SymComplexSparse.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix_sparse/Matrix_ArrayComplexSparse.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix_sparse/Matrix_ArraySparse.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix_sparse/Matrix_Sparse.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/matrix_sparse/Matrix_SymSparse.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/SeldonPreconditioner.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/SeldonHeader.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/vector/PetscVector.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/vector/HeterogeneousCollection.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/vector/Vector2.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/vector/Vector3.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/vector/VectorCollection.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/vector/Vector.hxx
        ${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/vector/SparseVector.hxx
        )

    list(APPEND SOURCE_FILES
        verdandiSofa/VerdandiAnimationLoop.cpp
        verdandiSofa/SofaModelWrapper.cpp
        verdandiSofa/SofaModelWrapperParallel.cpp
        verdandiSofa/VerdandiClasses.cpp
        verdandiSofa/SimulatedStateObservationSource.cpp
        ##verdandiSofa/VerdandiROUKFParams.cpp
        ##verdandiSofa/VerdandiObservationParams.cpp        
        )

endif()


add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})


if(SofaPardisoSolver_FOUND)
    set(LINKER_DEPENDENCIES ${LINKER_DEPENDENCIES} SofaPardisoSolver )
else()
    message("SofaPardisoSolver not found, some scenes may not work")
endif()


if(${SOFA-EXTERNAL_VERDANDI})
    target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/extlibs/lua/src")
endif()


target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>")
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${OPTIMUS_VERSION}")
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${COMPILATION_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")


target_link_libraries(${PROJECT_NAME} ${LINKER_DEPENDENCIES})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src")



if(${SOFA-EXTERNAL_VERDANDI})
    include_directories(${SOFA-EXTERNAL_VERDANDI_PATH})
    include_directories(${SOFA-EXTERNAL_VERDANDI_PATH}/share)
    include_directories(${SOFA-EXTERNAL_VERDANDI_PATH}/include)
    include_directories(${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon)
    include_directories(${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon/computation/interfaces)
    include_directories(${LUA_INCLUDE_DIR})

    ###link_directories(${SOFA-EXTERNAL_VERDANDI_PATH}/python)
    ###link_directories(${SOFA-EXTERNAL_VERDANDI_PATH}/include/seldon)

    set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_HAVE_VERDANDI -DSELDON_WITH_CBLAS")
    target_link_libraries(${PROJECT_NAME} SofaCore blas lapack liblua pthread)

    # MAC:
    ###FIND_LIBRARY(BlasLapackLib Accelerate)
    ###TARGET_LINK_LIBRARIES(${BlasLapackLib})
    ###AddLinkerDependencies(SofaCore SofaValidation  ${BlasLapackLib} lua)

    # LINUX:
    #set(LINKER_DEPENDENCIES "${LINKER_DEPENDENCIES} -lSofaCore -lSofaValidation -llua -lblas -llapack")
    ###AddLinkerFlags("-Wl,--no-undefined")
endif()

install(TARGETS Optimus
        EXPORT  OptimusTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION include/Optimus)
