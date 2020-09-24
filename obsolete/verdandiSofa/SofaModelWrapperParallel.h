/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program. If not, see <http://www.gnu.org/licenses/>.              *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_SIMULATION_SOFA_MODEL_WRAPPER_PARALLEL_H
#define SOFA_SIMULATION_SOFA_MODEL_WRAPPER_PARALLEL_H

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/BaseAnimationLoop.h>
#include <sofa/core/ExecParams.h>
#include <sofa/simulation/common/common.h>
#include <sofa/simulation/common/Node.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/simulation/common/Simulation.h>

#include <sofa/simulation/common/CollisionAnimationLoop.h>
#include <SofaConstraint/LCPConstraintSolver.h>
//#include <sofa/component/component.h>

#include "../src/genericComponents/OptimParams.h"

#include "VerdandiClasses.h"
#include <SofaBoundaryCondition/FixedConstraint.h>
#include "../src/genericComponents/ObservationSource.h"
#include "../src/genericComponents/SimulatedStateObservationSource.h"
#include "SofaModelWrapper.h"

#include "sofa/core/Mapping.h"
#include <SofaBaseTopology/TriangleSetTopologyContainer.h>

#include "../src/genericComponents/PointProjection.h"

#include <sofa/helper/gl/template.h>

#include <sofa/simulation/common/AnimateEndEvent.h>
#include <sofa/simulation/common/AnimateBeginEvent.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>


#include <pthread.h> /* pthread_t, pthread_barrier_t */

#ifdef __APPLE__
#include "pthread_barrier.h"
#endif

#include <utility> /* std::pair */

#define SNCOUTP(arg)     { std::cout << "[" << this->getName() << "] " << arg << std::endl; }

#define SERRP(arg)     { std::cerr << "[" << this->getName() << "] " << arg << std::endl; }


using namespace sofa::core::objectmodel;
using namespace sofa::core::behavior;

namespace sofa
{

namespace simulation
{

/* enumerated types used by the components implemented here */
enum FilterTypeP { UNDEFP, FORWARDP, UKFP, ROUKFP };
enum SigmaPointType {SIMPLEX, STAR, CANONICAL};
enum ThreadSignal {INITIALIZED = 0, WORK_ENQUEUED, NO_WORK, CANCEL_THREAD};

/***********************************************************************************************************
 ***********************************************************************************************************
 **************************      START OF THE KALMAN FILTER SECTION      *****************************
 ***********************************************************************************************************
 ***********************************************************************************************************/

/**
 *  \brief Wrapper class implementing an interface between SOFA and Verdandi
 */
template <class Model, class ObservationManager >
class SOFA_SIMULATION_COMMON_API SofaReducedOrderUKFParallel : public SofaVerdandiFilter<Model, ObservationManager>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(SofaReducedOrderUKFParallel, Model, ObservationManager), SOFA_TEMPLATE2(SofaVerdandiFilter,Model,ObservationManager));
protected:
    bool positionInState, velocityInState;

public:            
    //typedef typename Verdandi::ReducedOrderUnscentedKalmanFilter<Model, ObservationManager> Inherit1;
    //typedef typename sofa::core::objectmodel::BaseObject Inherit2;

    Data<std::string> m_outputDirectory, m_configFile, m_sigmaPointType, m_observationErrorVariance, m_paramFileName, m_paramVarFileName;
    Data<bool> m_saveVQ, m_showIteration, m_showTime, m_analyzeFirstStep, m_withResampling;
    Data<bool> m_positionInState, m_velocityInState;
    //Data<int> m_threadCount;

    SofaReducedOrderUKFParallel();

    void init();

    void InitializeFilter();
    void InitializeParams();
    void InitializeStructures();
    void FinalizeStep();
    
    Model* getModel() {
        return &(this->model_);
    }

private:
    bool saveParams, saveParamVar;

}; // class SofaReducedOrderUKFParallel



template <class Model, class ObservationManager >
class SOFA_SIMULATION_COMMON_API SofaUnscentedKalmanFilterParallel : public Verdandi::UnscentedKalmanFilter<Model, ObservationManager>, public sofa::core::objectmodel::BaseObject
{
public:
}; // class SofaUnscentedKalmanFilterParallel



template <class Model>
class SOFA_SIMULATION_COMMON_API SofaForwardDriverParallel : public Verdandi::ForwardDriver<Model>, public sofa::core::objectmodel::BaseObject
{
public:
}; // class SofaForwardDriverParallel



/***********************************************************************************************************
 ***********************************************************************************************************
 **************************      START OF THE SOFA MODEL PARALLEL SECTION      *****************************
 ***********************************************************************************************************
 ***********************************************************************************************************/

template <class Type>
class SOFA_SIMULATION_COMMON_API SofaModelWrapperParallel : public SofaModelWrapperBase<Type>
{
public:
    //SOFA_CLASS(SOFA_TEMPLATE(SofaModelWrapperParallel,Type),sofa::core::objectmodel::BaseObject);
    SOFA_CLASS(SOFA_TEMPLATE(SofaModelWrapperParallel,Type),SOFA_TEMPLATE(SofaModelWrapperBase,Type));
    /// SOFA TYPES
    typedef sofa::core::objectmodel::BaseObject Inherit;

    /// VERDANDI TYPES

    //! The numerical type (e.g., double).
    typedef Type value_type;
    //! Pointer to the numerical type.
    typedef Type* pointer;
    //! Const pointer to the numerical type.
    typedef const Type* const_pointer;
    //! Reference to the numerical type.
    typedef Type& reference;
    //! Const reference to the numerical type.
    typedef const Type& const_reference;
#ifdef VERDANDI_SPARSE
    //! Type of the background error covariance matrix.
    typedef Seldon::Matrix<Type, General, RowSparse> state_error_variance;
    //! \brief Type of the reduced matrix \f$U\f$ in the \f$LUL^T\f$
    //  decomposition of the background error covariance matrix.
    typedef Seldon::Matrix<Type, General, RowSparse> state_error_variance_reduced;
    //! Type of a row of the background error variance.
    typedef Seldon::Vector<Type> state_error_variance_row;
    //! Type of the model/observation crossed matrix.
    typedef Seldon::Matrix<Type, General, RowSparse> matrix_state_observation;
    //! Type of the tangent linear operator.
    typedef Seldon::Matrix<Type, General, RowSparse> tangent_linear_operator;
#else
    //! Type of the background error covariance matrix.
    typedef Seldon::Matrix<Type> state_error_variance;
    // ! \brief Type of the reduced matrix \f$U\f$ in the \f$LUL^T\f$
    //  decomposition of the background error covariance matrix.
    typedef Seldon::Matrix<Type> state_error_variance_reduced;
    //! Type of a row of the background error variance.
    typedef Seldon::Vector<Type> state_error_variance_row;
    //! Type of the model/observation crossed matrix.
    typedef Seldon::Matrix<Type> matrix_state_observation;
    //! Type of the tangent linear operator.
    typedef Seldon::Matrix<Type> tangent_linear_operator;
#endif
    //! Type of the model state vector.
    typedef Seldon::Vector<Type> state;
    //! Collection of vector state.
    typedef Seldon::Vector<state, Seldon::Collection> state_collection;

    typedef core::behavior::MechanicalState<defaulttype::Vec3dTypes> MechStateVec3d;
    typedef component::projectiveconstraintset::FixedConstraint<defaulttype::Vec3dTypes> FixedConstraintVec3d;

    typedef core::behavior::MechanicalState<defaulttype::Rigid3dTypes> MechStateRigid3d;
    typedef component::projectiveconstraintset::FixedConstraint<defaulttype::Rigid3dTypes> FixedConstraintRigid3d;

    typedef sofa::component::container::OptimParamsBase OptimParams;


    /**
      * Structure for launching SofaModelWrapper from SofaReducedOrderUKFParallel
      */
    struct ModelData {
        simulation::Node* gnode;
        FilterTypeP filterTypeP;
        bool positionInState;
        bool velocityInState;
        double errorVarianceSofaState;
        bool verbose;
        SigmaPointType sigmaType;
        
        ModelData() 
            : gnode(NULL)            
            , positionInState(false)
            , velocityInState(false)
            , errorVarianceSofaState(0.0)
            , verbose(false)
            , sigmaType(SIMPLEX)
            
        {}
    };



    /**
      * Wrapper for sofa mechanical state pointer.
      */
    struct SofaObjectParallel {
        simulation::Node* node;                 /// associated node
        MechStateVec3d* vecMS;                  /// pointer to mechanical state (Vec3D), to be templated
        MechStateRigid3d* rigidMS;              /// pointer to mechanical state (Rigid3D), to be templated
        FixedConstraintVec3d* vecFC;            /// pointer to fixed constraints (fixed DoFs must be ommitted from Verdandi state vector)
        FixedConstraintRigid3d* rigidFC;
        helper::vector<std::pair<size_t, size_t> > positionPairs;       /// map to match positions in SOFA and Verdandi state vector
        helper::vector<std::pair<size_t, size_t> > velocityPairs;       /// map to match velocities in SOFA and Verdandi state vector
        
        SofaObjectParallel() 
            :node(NULL)
            ,vecMS(NULL)
            ,rigidMS(NULL)
            ,vecFC(NULL)
            ,rigidFC(NULL)
        {}
    };

    /**
     * structure for ordering SofaObjectParallel objects by name of parent node and name of wrapped mechanical state
     */
    typedef std::pair<string, SofaObjectParallel> ObjID;


public:
    const core::ExecParams* execParams;

    int current_row_; // saves the index of last loaded row of vector variance, which is stored
    size_t dim_; // dimension of the data [always 3]


    /**
      * Struct specific to each thread. Defines the slave scene the computations are performed on.
      * Also stores the arguments for work task - thread signal and sigma point indices.
      * Thread signals might be used in future for half-duplex communication with master, via barriers.
      **/
    typedef struct
    {
        simulation::Node* localRoot; // root of a slave scene
        OptimParams* paramsSlave; // params governing the scene
        helper::vector<SofaObjectParallel> sofaObjectsSlave; // Sofa objects in this local scene
        ThreadSignal threadSignal; // signal for data assimilation step
        helper::vector<int> sigmaPointIndices; // indices to be computed
    } ThreadData_t;




    SigmaPointType m_sigmaType; // type of ROUKF used - simplex, canonical, star
    int m_sigmaPointCount; // number of sigma points
    int m_slaveCount; // number of slave nodes, i.e. number of slave threads

    state* t_sigmaPoints; // ROUKF sigma points
    pthread_t* t_threadHandlers; // thread handlers
    ThreadData_t* t_threadData; // thread specific data
    pthread_barrier_t   t_initDone, t_workAssigned, t_workDone; // barriers for thread syncrhonization
    bool t_input_preserveState; // step argument
    bool t_input_updateForce; // step argument





    simulation::Node* rootMaster;
    helper::vector<SofaObjectParallel> sofaObjectsMaster; // Sofa objects in master scene's scene
    OptimParams* paramsMaster; // optimized params at the master scene
    state verdandiStateMaster; // verdandi state describing the master scene

    size_t state_size_parallel; // size of verdandi state
    size_t reduced_state_size_parallel; // reduced size of the verdandi state
    size_t reduced_state_index_parallel;    // where the params start in the Verdandi state 


    //! Background error covariance matrix (B).
    state_error_variance state_error_variance_;
    //! Inverse of the background error covariance matrix (B^-1).
    state_error_variance state_error_variance_inverse_;
    //! Value of the row of B currently stored.
    state_error_variance_row state_error_variance_row_;

    //! Projector matrix L in the decomposition of the
    //  background error covariance matrix (\f$B\f$) as a product LUL^T
    state_error_variance state_error_variance_projector_;
    //! Reduced matrix U in the decomposition of the
    //  background error covariance matrix (\f$B\f$) as a product LUL^T
    state_error_variance_reduced state_error_variance_reduced_;     /// remains constant during the assimilation
    //! Is state error variance projector allocated?
    bool variance_projector_allocated_;
    //! Is reduced state error variance allocated?
    bool variance_reduced_allocated_;

    double time_;

    ModelData modelData;

    sofa::core::behavior::ConstraintSolver *constraintSolver;

private:
    static helper::vector<SofaObjectParallel> getSofaObjects(const simulation::Node* root);
    static helper::vector<ObjID> getObjIDsFromObjects (const helper::vector<SofaObjectParallel>& objects);
    static helper::vector<ObjID> getSortedObjIDsFromObjects (const helper::vector<SofaObjectParallel>& objects);
    void synchronizeSofaObjects ();

    void distributeWork();

    void initThreadStructures();

public:
    SofaModelWrapperParallel();
    virtual ~SofaModelWrapperParallel();

    void setInitStepData(const core::ExecParams* _execParams) { execParams = _execParams;}


    void initSimuData(const ModelData& _md);

    SofaObjectParallel* getObject(const MechStateVec3d *_state);
    void SetSofaVectorFromVerdandiState(defaulttype::Vec3dTypes::VecCoord & vec, const state& _state, SofaObjectParallel *obj); // last change, some stuff in obs

    /// functions propagating state between SOFA and Verdandi

    void StateSofa2VerdandiParallel(const helper::vector<SofaObjectParallel>& mechanicalObjects, OptimParams* oparams, state& verdandiState);
    void StateVerdandi2SofaParallel(helper::vector<SofaObjectParallel>& mechanicalObjects, OptimParams* oparams, const state& verdandiState);

    /// functions required by Verdandi API:

    int GetNstate() const { return state_size_parallel; }
    double GetTime() { return time_; }
    state& GetState();

    void Initialize(std::string &) { Initialize();  }

    void Initialize();
    //void InitializeParallel();
    void InitializeStep() {}
    void Finalize() {}
    void FinalizeStep();
    bool HasFinished() { return(false); }
    void StateUpdated();
    void SetTime(double _time); // not thread safe

    double ApplyOperator(state& _x, bool _preserve_state = true, bool _update_force = true);
    double ApplyOperatorParallel(state* sigmaPoints, bool _preserve_state = true, bool _update_force = true);

    void Forward(bool _update_force = true, bool _update_time = true, simulation::Node* = NULL);

    void StepDefault(bool _update_force, bool _update_time, simulation::Node* = NULL);
    void StepDefaultOrig(bool _update_force, bool _update_time, simulation::Node* = NULL);
    void StepFreeMotion(bool _update_force, bool _update_time);

    void Message(string _message);


    state_error_variance& GetStateErrorVariance();
    state_error_variance_row& GetStateErrorVarianceRow(int row);
    state_error_variance& GetStateErrorVarianceProjector();
    state_error_variance_reduced& GetStateErrorVarianceReduced();

    void Verb(string _s) {
        if (modelData.verbose)
            std::cout << "[" << this->getName() << "]: " << _s << std::endl;
    }



    void printMatrix(Seldon::Matrix<Type>& M, std::ostream &of) {
        for (int i = 0; i < M.GetM(); i++)
          {
            for (int j = 0; j < M.GetN(); j++)
              of << M(i, j) << '\t';
            of << '\n';
          }
    }



    void printMatrixInRow(Seldon::Matrix<Type>& M, std::ostream &of) {
        for (int i = 0; i < M.GetM(); i++)
          {
            for (int j = 0; j < M.GetN(); j++)
              of << M(i, j) << '\t';
          }
        of << '\n';
    }



    void printVector(Seldon::Vector<Type>& V, std::ofstream &of) {
        for (int i = 0; i < V.GetSize(); i++)
            of << V(i) << '\n';
    }

}; // class SofaModelWrapperParallel



/***********************************************************************************************************
 ***********************************************************************************************************
 *************************      START OF THE OBSERVATION MANAGERS' SECTION      ****************************
 ***********************************************************************************************************
 ***********************************************************************************************************/

/***********************************************************************************************************
 *********************************      Linear observation manager      ************************************
 ***********************************************************************************************************/


template <class T>
class SOFA_SIMULATION_COMMON_API SofaLinearObservationManagerParallel : public Verdandi::LinearObservationManager<T>, public sofa::core::objectmodel::BaseObject
{
public:            
    typedef typename Verdandi::LinearObservationManager<T> Inherit1;

    Data<double> m_observationStdev;

    SofaLinearObservationManagerParallel()
        : m_observationStdev( initData(&m_observationStdev, double(0.0), "observationStdev", "standard deviation in observations") )
    {
    }

    void SetTime(double _time) {
        //std::cout<<"SofaLinearObservationManagerParallel // time set: //"<<_time;
        this->time_ = _time;
    }

    virtual void SetTime(SofaModelWrapperParallel<double>& /*model*/, double _time) {
        //std::cout<<"SofaLinearObservationManagerParallel // time set: //"<<_time;
        this->time_ = _time;
    }

    bool HasObservation() const {
        return true;
    }

    bool HasObservation(double /*time*/) {
        return true;
    }

    virtual typename Inherit1::observation& GetInnovation(const typename SofaModelWrapperParallel<double>::state& x) {
        return Inherit1::GetInnovation(x);
    }

    virtual int GetNobservation() {
        return Inherit1::GetNobservation();
    }

    virtual const typename Inherit1::error_variance& GetErrorVariance() {
        return Inherit1::GetErrorVariance();
    }

    virtual const typename Inherit1::error_variance& GetErrorVarianceInverse() {
        return Inherit1::GetErrorVarianceInverse();
    }

    virtual void Initialize(SofaModelWrapperParallel<double>& /*model*/, std::string /*confFile*/) {
        Verb("initialize sofaLinObsManager");
        //Inherit1::Initialize(model, confFile);

        return;
    }

    void Verb(string _s) {
        //if (modelData.verbose)
            std::cout << "[" << this->getName() << "]: " << _s << std::endl;
    }

}; // class SofaLinearObservationManagerParallel

/***********************************************************************************************************
 ******************************      Mapped Points Obervation Manager      *********************************
 ***********************************************************************************************************/

template <class DataTypes1, class DataTypes2>
class SOFA_SIMULATION_COMMON_API MappedPointsObservationManagerParallel : public SofaLinearObservationManagerParallel<double>
{
public:
    typedef typename DataTypes1::Real Real1;
    typedef SofaLinearObservationManagerParallel<double> Inherit;
    typedef core::behavior::MechanicalState<DataTypes1> MasterState;
    typedef core::behavior::MechanicalState<DataTypes2> MappedState;
    typedef sofa::core::Mapping<DataTypes1, DataTypes2> Mapping;
    typedef sofa::component::container::SimulatedStateObservationSource<DataTypes1> ObservationSource;

protected:

    SofaModelWrapperParallel<Real1>* sofaModel;
    typename SofaModelWrapperParallel<Real1>::SofaObjectParallel* m_sofaObjectParallel;
    size_t masterStateSize;
    size_t mappedStateSize;    

public:

    MappedPointsObservationManagerParallel()
        : Inherit()
        , actualTime(-1.0)
        , inputObservationData( initData (&inputObservationData, "observations", "observations read from a file") )
        , mappedObservationData( initData (&mappedObservationData, "mappedObservations", "mapped observations") )
        , m_noiseStdev( initData(&m_noiseStdev, double(0.0), "noiseStdev", "standard deviation of generated noise") )
        , m_abberantIndex( initData(&m_abberantIndex, int(-1), "abberantIndex", "index of an aberrant point") )

    {
    }

    Mapping* mapping;
    MappedState* mappedState;
    MasterState* masterState;
    ObservationSource *observationSource;
    double actualTime;
    Data<typename DataTypes1::VecCoord> inputObservationData;
    Data<typename DataTypes2::VecCoord> mappedObservationData;
    Data<double> m_noiseStdev;
    Data<int> m_abberantIndex;

    boost::mt19937* pRandGen; // I don't seed it on purpouse (it's not relevant)
    boost::normal_distribution<>* pNormDist;
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> >* pVarNorm;

    int actualStep;


    helper::vector<double> noise;

    void init();
    void bwdInit();

    virtual typename Inherit::observation& GetInnovation(const typename SofaModelWrapperParallel<double>::state& x);

    virtual void SetTime(SofaModelWrapperParallel<double>& /*model*/, double time) {
        //std::cout << "MappedPointsObservationManagerParallel / time set: " << time << std::endl;
        Inherit::SetTime(time);
    }

    virtual int GetNobservation() {
        return this->Nobservation_;
    }

    virtual const typename Inherit::error_variance& GetErrorVariance() {        
        return Inherit::GetErrorVariance();
    }

    virtual const typename Inherit::error_variance& GetErrorVarianceInverse() {        
        return Inherit::GetErrorVarianceInverse();
    }

    virtual void Initialize(SofaModelWrapperParallel<double>& /*model*/, std::string /*confFile*/);

    virtual void handleEvent(core::objectmodel::Event *event) {
        if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
        {
            Verb("creating noise!");
            if (this->getContext()->getTime() == actualTime)
            {
                Verb("not creating noise!");
                return;
            }
            actualTime = this->getContext()->getTime();
            actualStep++;

            if (m_noiseStdev.getValue() != 0.0) {
                std::cout << this->getName() << ": generating noise in the time step: " << std::endl;
                for (size_t i = 0; i < 3*mappedObservationData.getValue().size(); i++)
                    noise[i] = (*pVarNorm)();
                //std::cout << noise << std::endl;
            }

            /*std::cout << "Generate noise in the observations in time " << actualTime << std::endl;


            srand(time(NULL));
            for (size_t i = 0; i < 3*mappedObservationData.getValue().size(); i++) {
                double sgn = (rand()%2 == 0) ? -1.0 : 1.0;
                if ( i == 19 ||  i == 3) {
                //if (i == 10 || i == 21) {
                    if (noise[i] > -0.15)
                        noise[i] -= double(rand()%1000)/double(100000);
                    else
                        noise[i] += double(rand()%1000)/double(100000);
                }
                else {
                    noise[i] = sgn*double(rand()%1000)/double(100000);
                }
            }*/


        }
    }

}; // class MappedPointsObservationManagerParallel

/***********************************************************************************************************
 ***********************************      ARObservation Manager      ***************************************
 ***********************************************************************************************************/

template <class DataTypes1, class DataTypes2>
class SOFA_SIMULATION_COMMON_API ARObservationManagerParallel : public SofaLinearObservationManagerParallel<double>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(ARObservationManagerParallel, DataTypes1, DataTypes2), SOFA_TEMPLATE(SofaLinearObservationManagerParallel, double));

    typedef typename DataTypes1::Real Real1;
    typedef typename DataTypes1::VecCoord VecCoord1;
    typedef typename DataTypes1::Coord Coord1;
    typedef SofaLinearObservationManagerParallel<double> Inherit;
    typedef core::behavior::MechanicalState<DataTypes1> MasterState;
    typedef core::behavior::MechanicalState<DataTypes2> MappedState;
    typedef sofa::core::Mapping<DataTypes1, DataTypes2> Mapping;
    typedef sofa::component::container::SimulatedStateObservationSource<DataTypes1> ObservationSource;

    typedef sofa::component::topology::TriangleSetTopologyContainer TriangleContainer;
    typedef PointProjection<Real1> PointProjection1;

protected:
    //SingleLink<ARObservationManager<DataTypes1,DataTypes2>, TriangleContainer, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> triangleContainerLink;
    //SingleLink<ARObservationManager<DataTypes1,DataTypes2>, MechanicalState<DataTypes1>, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> triangleMStateLink;
    SingleLink<ARObservationManagerParallel<DataTypes1,DataTypes2>, MechanicalState<DataTypes1>, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> featureMStateLink;


    SofaModelWrapperParallel<Real1>* sofaModel;
    typename SofaModelWrapperParallel<Real1>::SofaObjectParallel* sofaObjectParallel;
    //size_t masterStateSize;
    //size_t mappedStateSize;

    //TriangleContainer* triangleContainer;
    //PointProjection<Real1>* pointProjection;
    MechanicalState<DataTypes1>* mappedMState;
    MechanicalState<DataTypes1>* featureMState;
    MechanicalState<DataTypes1>* triangleMState;

    //VecCoord1 projectedFeatures;

    //SingleLink<ARObservationManager<DataTypes1, DataTypes2>, TriangleContainer, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> triangleTopologyContainer;


public:

    TriangleContainer* triaCont;
    ARObservationManagerParallel()
        : Inherit()
        //, triangleContainerLink(initLink("triangleTopologyContainer", "link to the triangle topology needed for point projection"), triangleContainer)
        //, triangleMStateLink(initLink("triangleMState", "link to the mstate of the surface needed for point projection"), triangleMState)
        , featureMStateLink(initLink("featureMState", "link to the mstate the moving features"), triangleMState)
    {
    }

    Mapping* mapping;
    MappedState* mappedState;
    MasterState* masterState;
    ObservationSource *observationSource;

    void init() {
        simulation::Node* gnode = dynamic_cast<simulation::Node*>(this->getContext());
        gnode->get(mapping);
        if (mapping) {
            std::cout << "[" << this->getName() << "]: " << "found mapping: " << mapping->getName() << std::endl;
        } else
            std::cerr << "[" << this->getName() << "]: ERROR no mapping found " << std::endl;

        gnode->get(mappedMState);
        if (mappedMState == NULL)
            std::cerr << "[" << this->getName() << "]: ERROR: cannot find the local mechanical state" << std::endl;
        else
            std::cout << "[" << this->getName() << "]: the local mechanical state name: " << mappedMState->getName() << std::endl;

        featureMState = featureMStateLink.get();
        if (featureMState == NULL)
            std::cerr << "[" << this->getName() << "]: ERROR: cannot find the master mechanical state" << std::endl;
        else
            std::cout << "[" << this->getName() << "]: the master mechanical state name: " << featureMState->getName() << std::endl;

        if (mappedMState->getSize() != featureMState->getSize())
            std::cerr << "[" << this->getName() << "]: ERROR: different number of nodes in feature and local states" << std::endl;

        /*triangleMState = triangleMStateLink.get();
        if (triangleMState == NULL)
            std::cerr << "[" << this->getName() << "]: ERROR: cannot find the triangle mechanical state" << std::endl;
        else
            std::cout << "[" << this->getName() << "]: the triangle  mechanical state name: " << triangleMState->getName() << std::endl;

        triangleContainer = triangleContainerLink.get();
        if (triangleContainer == NULL)
            std::cerr << "[" << this->getName() << "]: ERROR: cannot find the triangle container" << std::endl;
        else
            std::cout << "[" << this->getName() << "]: the triangle  container name: " << triangleContainer->getName() << std::endl;

        pointProjection = new PointProjection<Real1>(*triangleContainer);*/

        /// project initially all the points from triangle mstate to the surface and store in the local m-state!

        /*helper::ReadAccessor<Data<VecCoord1> > trianglePos = *triangleMState->read(sofa::core::VecCoordId::position());
        helper::ReadAccessor<Data<VecCoord1> > masterPos = *masterMState->read(sofa::core::VecCoordId::position());
        //helper::WriteAccessor<Data<VecCoord1> > localFreePos = *mappedMState->write(sofa::core::VecCoordId::freePosition());
        helper::WriteAccessor<Data<VecCoord1> > localPos = *mappedMState->write(sofa::core::VecCoordId::position());

        if (localPos.size() != masterPos.size()) {
            std::cerr << "[" << this->getName() << "]: ERROR: difference in feature and local state size" << std::endl;
        }

        projectedFeatures.resize(masterPos.size());

        typename PointProjection1::Index index;
        typename PointProjection1::Vec3 bary;

        for (size_t i = 0; i < localPos.size(); i++) {
            Coord1 projectedCoord;
            pointProjection->ProjectPoint(bary, projectedCoord, index, masterPos[i], trianglePos.ref());
            localPos[i] = projectedCoord;
            projectedFeatures[i] = projectedCoord;
            //localFreePos[i] = projectedCoord;
            //std::cout << "Point " << i << " projected on triangle " << index << std::endl;
            //std::cout << "      " << localPos[i] << " => " << projectedCoord << std::endl;
        }*/


        gnode->get(sofaModel, core::objectmodel::BaseContext::SearchUp);
        if (sofaModel) {
            std::cout << "[" << this->getName() << "]: " << "found SOFA model: " << sofaModel->getName() << std::endl;
        } else
            std::cerr << "[" << this->getName() << "]: ERROR no SOFA model found " << std::endl;

        sofaObjectParallel = NULL;
        masterState = dynamic_cast<MasterState*>(mapping->getFromModel());
        if (masterState != NULL) {
            sofaObjectParallel = sofaModel->getObject(masterState);
        }
        else
            std::cerr << this->getName() << "ERROR SOFA MO not found!" << std::endl;

        if (!sofaObjectParallel)
            std::cerr << this->getName() << "ERROR SOFA object not found " << std::endl;
        else

            std::cout << this->getName() << "Sofa object found " << std::endl;
    }


    void bwdInit() {

    }

    virtual void SetTime(SofaModelWrapperParallel<double>& model, double time) {
        std::cout << "Setting time: " << time << std::endl;
        return Inherit::SetTime(model, time);
    }

    virtual int GetNobservation() {
        return this->Nobservation_;
    }

    virtual const typename Inherit::error_variance& GetErrorVariance() {
        return Inherit::GetErrorVariance();
    }

    virtual const typename Inherit::error_variance& GetErrorVarianceInverse() {
        return Inherit::GetErrorVarianceInverse();
    }

    virtual void Initialize(SofaModelWrapperParallel<double>& /*model*/, std::string /*confFile*/) {
        this->Delta_t_ = 0.001;
        this->Nskip_= 1;
        this->initial_time_ = 0.0;
        this->final_time_ = 1000.0;

        this->error_variance_value_ = m_observationStdev.getValue() * m_observationStdev.getValue();
        this->Nobservation_ = 3*mappedMState->getSize();
        this->error_variance_.Reallocate(this->Nobservation_, this->Nobservation_);
        this->error_variance_.SetIdentity();
        Mlt(this->error_variance_value_, this->error_variance_);
        this->error_variance_inverse_.Reallocate(this->Nobservation_, this->Nobservation_);
        this->error_variance_inverse_.SetIdentity();
        Mlt(double(double(1.0)/ this->error_variance_value_), this->error_variance_inverse_);

        std::cout << this->getName() << " size of observed state: " << this->Nobservation_ << std::endl;

        return;
    }

    virtual typename Inherit::observation& GetInnovation(const typename SofaModelWrapperParallel<double>::state& x) {
        std::cout << "[" << this->getName() << "]: new get innovation " << std::endl;

        /// the observation already projected on the liver surface => copying to verdandi vector (TODO optimize)
        helper::ReadAccessor<Data<VecCoord1> > fX = *featureMState->read(sofa::core::VecCoordId::position());

        Inherit::observation actualObs(fX.size()*3);
        for (size_t i = 0; i < fX.size(); i++)
            for (size_t d = 0; d < 3; d++)
                actualObs(3*i+d) = fX[i][d];        

        Data<VecCoord1> actualStateData;
        Data<VecCoord1> mappedStateData;

        VecCoord1& actualState = *actualStateData.beginEdit();
        VecCoord1& mappedState = *mappedStateData.beginEdit();

        std::cout<<"AAAAAA: crash here?\n";

        mappedState.resize(mappedMState->getSize());                
        sofaModel->SetSofaVectorFromVerdandiState(actualState, x, sofaObjectParallel);

        std::cout<<"AAA:::: yes indeed!\n";

        sofa::core::MechanicalParams mp;
        mapping->apply(&mp, mappedStateData, actualStateData);        

        //std::cout << this->getName() << ": size of mapped state: " << mappedState.size() << std::endl;
        this->innovation_.Reallocate(mappedState.size()*3);
        for (size_t i = 0; i < mappedState.size(); i++)
            for (size_t d = 0; d < 3; d++)
                this->innovation_(3*i+d) = mappedState[i][d];

        //this->innovation_.Reallocate(this->Nobservation_);
        //this->ApplyOperator(x, this->innovation_);
        Inherit::observation predObs  = this->innovation_;
        Mlt(double(-1.0), this->innovation_);
        //Add(double(1.0), this->GetObservation(), this->innovation_);
        Add(double(1.0), actualObs, this->innovation_);
        //std::cout << this->getName() << ": innovation updated" << std::endl;

        //std::cout << "ERROR VARIANCE: " << this->error_variance_ << std::endl;

        for (size_t i = 0; i < this->innovation_.GetM(); i++)
            std::cout << actualObs(i) << " " << predObs(i) << " " << this->innovation_(i) << std::endl;

        actualStateData.endEdit();
        mappedStateData.endEdit();

        return this->innovation_;


    }

    void handleEvent(core::objectmodel::Event *event) {
        if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event)) {
            /// update projection: TODO more efficiently by taking into account the previous triangle

            /*helper::ReadAccessor<Data<VecCoord1> > trianglePos = *triangleMState->read(sofa::core::VecCoordId::position());
            helper::ReadAccessor<Data<VecCoord1> > masterPos = *masterMState->read(sofa::core::VecCoordId::position());
            //helper::WriteAccessor<Data<VecCoord1> > localPos = *localMState->write(sofa::core::VecCoordId::position());
            //helper::WriteAccessor<Data<VecCoord1> > localFreePos = *localMState->write(sofa::core::VecCoordId::freePosition());

            typename PointProjection1::Index index;
            typename PointProjection1::Vec3 bary;

            for (size_t i = 0; i < masterPos.size(); i++) {
                Coord1 projectedCoord;
                pointProjection->ProjectPoint(bary, projectedCoord, index, masterPos[i], trianglePos.ref());
                projectedFeatures[i] = projectedCoord;
                //localFreePos[i] = projectedCoord;
                //std::cout << "Point " << i << " projected on triangle " << index << std::endl;
                //std::cout << "      " << projectedFeatures[i] << " => " << projectedCoord << std::endl;
            }*/

        } else if (dynamic_cast<sofa::simulation::AnimateEndEvent *>(event)) {

        }

    }


    void draw(const core::visual::VisualParams* /*vparams*/) {
        //helper::ReadAccessor<Data<VecCoord1> > fX = *featureMState->read(sofa::core::VecCoordId::position());
        //helper::ReadAccessor<Data<VecCoord1> > X = *localMState->read(sofa::core::VecCoordId::position());

        /*glDisable(GL_LIGHTING);
        for (size_t i = 0; i < fX.size(); i++)  {
            glLineWidth(2.0);
            glBegin(GL_LINES);
            glColor4f(1,1,1,1);
            helper::gl::glVertexT(fX[i]);
            helper::gl::glVertexT(projectedFeatures[i]);
            glEnd();
        }

        glPointSize(10);
        glColor4f(0.5,0.5,0.3,1.0);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < projectedFeatures.size(); i++) {
            helper::gl::glVertexT(projectedFeatures[i]);
        }
        glEnd();

        glEnable(GL_LIGHTING);*/
    }

}; // class ARObservationManagerParallel




} // namespace simulation

} // namespace sofa

#endif  /* SOFA_SIMULATION_SOFA_MODEL_WRAPPER_PARALLEL_H */
