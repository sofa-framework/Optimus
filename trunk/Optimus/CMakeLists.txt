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
        verdandiSofa/SofaModelWrapper.inl
	verdandiSofa/VerdandiClasses.h
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/Verdandi.hxx
	${SOFA_EXTLIBS_DIR}/verdandi-1.5/model/QuadraticModel.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/model/ClampedBar.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/model/ModelTemplate.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/model/CheckingModel.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/model/Lorenz.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/model/ShallowWater.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/model/PetscClampedBar.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/model/PythonModel.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/share/VerdandiOps.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/share/Functions_Vector3.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/share/MessageHandler.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/share/LockFile.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/share/Functions_Vector2.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/share/Error.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/share/OutputSaver.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/share/VerdandiBase.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/share/Logger.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/share/Variable.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/share/UsefulFunction.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/sequential_aggregation/BaseForecaster.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/sequential_aggregation/Aggregate.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/sequential_aggregation/DiscountedRidgeRegression.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/error/DiagonalMatrix.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/error/IsotropicBalgovindMatrix.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/error/BalgovindMatrix.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/VerdandiHeader.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/method/ReducedMinimax.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/method/TRNGPerturbationManager.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/method/EnsembleKalmanFilter.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/method/SigmaPoint.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/method/ExtendedKalmanFilter.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/method/NewranPerturbationManager.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/method/FourDimensionalVariational.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/method/BasePerturbationManager.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/method/ReducedOrderUnscentedKalmanFilter.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/method/HamiltonJacobiBellman.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/method/ReducedOrderExtendedKalmanFilter.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/method/ObservationGenerator.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/method/MonteCarlo.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/method/ForwardDriver.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/method/UnscentedKalmanFilter.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/method/BLUE.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/method/TR1PerturbationManager.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/method/TrajectoryManager.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/method/OptimalInterpolation.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/method/chi_2.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/observation_manager/GridToNetworkObservationManager.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/observation_manager/ObservationManagerTemplate.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/observation_manager/LinearObservationManager.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/observation_manager/ObservationAggregator.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/observation_manager/PetscLinearObservationManager.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/observation_manager/PythonObservationManager.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/ops/OpsHeader.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/ops/ClassOps.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/ops/Ops.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/ops/Error.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/optimization/NLopt.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/optimization/NLoptSolver.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/basic_functions/Functions_Vector.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/basic_functions/Functions_Matrix.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/basic_functions/Functions_MatVect.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/solver/iterative/Iterative.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/solver/SparseCholeskyFactorisation.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/solver/SparseSolver.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/solver/preconditioner/IlutPreconditioning.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/solver/Ordering.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/interfaces/Blas_1.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/interfaces/Lapack_LeastSquares.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/interfaces/Blas_2.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/interfaces/Blas_3.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/interfaces/direct/Mumps.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/interfaces/direct/Pastix.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/interfaces/direct/SuperLU.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/interfaces/direct/UmfPack.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/interfaces/direct/Cholmod.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/interfaces/Lapack_Eigenvalues.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/computation/interfaces/Lapack_LinearEquations.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/share/Properties.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/share/Allocator.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/share/Errors.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/share/Common.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/share/Storage.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/share/MatrixFlag.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix/Matrix_Triangular.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix/Matrix_Base.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix/Matrix_HermPacked.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix/Matrix_Pointers.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix/MatrixCollection.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix/Matrix_TriangPacked.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix/Matrix_Hermitian.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix/Matrix_SymPacked.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix/SubMatrix_Base.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix/PetscMatrix.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix/Functions.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix/SubMatrix.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix/HeterogeneousMatrixCollection.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix/Matrix_Symmetric.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/Seldon.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/array3d/Array3D.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/SeldonSolver.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix_sparse/Matrix_Conversions.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix_sparse/Matrix_ComplexSparse.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix_sparse/IOMatrixMarket.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix_sparse/Matrix_SymComplexSparse.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix_sparse/Matrix_ArrayComplexSparse.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix_sparse/Matrix_ArraySparse.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix_sparse/Matrix_Sparse.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/matrix_sparse/Matrix_SymSparse.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/SeldonPreconditioner.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/SeldonHeader.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/vector/PetscVector.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/vector/HeterogeneousCollection.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/vector/Vector2.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/vector/Vector3.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/vector/VectorCollection.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/vector/Vector.hxx
        ${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon/vector/SparseVector.hxx
    )

    list(APPEND SOURCE_FILES
        verdandiSofa/VerdandiAnimationLoop.cpp
        verdandiSofa/SofaModelWrapper.cpp
	verdandiSofa/VerdandiClasses.cpp
    )

    include_directories(${SOFA_EXTLIBS_DIR}/verdandi-1.5)
    include_directories(${SOFA_EXTLIBS_DIR}/verdandi-1.5/share)
    include_directories(${SOFA_EXTLIBS_DIR}/verdandi-1.5/include)
    include_directories(${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/lua/src)

    link_directories(${SOFA_EXTLIBS_DIR}/verdandi-1.5/python)
    link_directories(${SOFA_EXTLIBS_DIR}/verdandi-1.5/include/seldon)

    AddLinkerDependencies(lua5.1)    
    AddLinkerDependencies(SofaCore SofaValidation lapack blas)
    AddLinkerFlags("-Wl,--no-undefined")

    AddCompilerDefinitions(SOFA_HAVE_VERDANDI)
endif()

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${MOC_FILES} ${SOURCE_FILES} ${README_FILES})

AddCompilerDefinitions("SOFA_BUILD_OPTIMUS")
AddLinkerDependencies( SofaBaseLinearSolver SofaBoundaryCondition )



include(${SOFA_CMAKE_DIR}/postProject.cmake)
