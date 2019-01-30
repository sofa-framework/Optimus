Advanced methods of state estimation and parameter identification in SOFA
=========================================================================

Optimus plugin was created to provide a testing environment for data-driven physics-based modeling (typically finite elements, FE).
While actually the plugin implements only stochastic methods based on Kalman filtering, its architecture allows for implementation of 
generic prediction--correction schemes where the model is employed as a predictor and correction is performed using given observation data. 


The concepts of estimation of state and parameter identification are summarized in a documentation in doc/OptimusDoc. To generate the 
PDF, compile the tex file as 
pdflatex ConceptsBehindOptimus.tex

Doxygen documentation [HTML and LaTeX] can be generated using existing doxygen file located in folder doc:
doxygen Optimus.doxygen.


Components
==========

Generic components
------------------

These components provide various functionality employed in Optimus scenes. 

`OptimParams`

-   Container of Gaussian stochastic variables given by expected value and standard deviation. 

`SimulatedStateObservationSource`

-   Tool for reading observations exported previously in SOFA Monitor format. 

`VTKExporterDA`

-   Component extending the functionality of SOFA VTKExporter (reaction to different events).


Stochastic filtering
--------------------

THese components are the core of Optimus plugin and implement functionality necessary for stochastic estimation. 
Compilation of these components must be activated by CMake macro STOCHASTIC_FILTERING.

`FilteringAnimationLoop`

-   Compliant with SOFA API for animation loops.

-   Calls filter to perform the assimilation in each step.
\medskip

`StochasticStateWrapper`
-   Wrapper of model provided by SOFA. This wrapper implements the interface between the stochastic components (filters) and SOFA simulation. 

-   Must be placed inside the SOFA subnode containing the physical simulation components. 

-   Requires MechanicalObject and assumes OptimParams in the same node. 

\medskip

`UKFilter`

-   Unscented Kalman Filter proposed by Julier and Uhlman (1997). Implementation performed according to code in Reduced order Kalman filter implemented according to Moireau, Philippe, and Dominique Chapelle. "Reduced-order Unscented Kalman Filtering with application to parameter identification in large-dimensional systems." 

\medskip

`ROUKFilter`

-   Reduced order Kalman filter implemented according to Moireau, Philippe, and Dominique Chapelle. "Reduced-order Unscented Kalman Filtering with application to parameter identification in large-dimensional systems." 

\medskip

`UKFilterSimCorr`

-   Special version of UKF filter purely for data assimilation. Currently being studied. 

\medskip

`MappedStateObservationManager`

-   One instance of observation manager which handles the observations and provides them to the filter via computation of innovation. 

-   Assumes that the predicted observations are mapped to the main mechanical object (associated with StochasticStateWrapper). 

`PreStochasticWrapper`

-   Special version of wrapper which allows for including other simulated object into Optimus scene which do not contain any quantities being estimated. 

-   Typical example is an obstacle which displays a physical behaviour, however, none of its features is directly involved in the estimation. 


Compilation and Usage
=====================

The plugin is stored in OptimusPlugin repository gitlab.inria.fr. In order to
obtain the plugin, it is necessary to perform git clone of the plugin to your local PC with SOFA installed. 

To compile:

-   Path to the plugin is to be added to SOFA_EXTERNAL_DIRECTORIES in SOFA Cmake configuration. 

-   To compile the stochastic components, set STOCHASTIC_FILTERING macro.

Except for SOFA, the plugin dependencies are BLAS, Eigen (already in SOFA). It is highly recommended to use Pardiso solver with Optimus, as other solvers in SOFA are unreliable and might impact the estimation. 


Examples
--------

In general, Optimus scenes are written in Python (occasionally in XML). It is highly recommended to use YAML (Python module) to define parameters of scenes. 
Enormously increases the efficiency and avoids mishaps. 

Examples are given folder examples:

`SingleParameterIdentification`

-   An example showing stochastic identification of Young's modulus of a homogeneous beam subjected to gravity. 

-   It's necessary to generate observations first: runSofa exampleParamId_GenObs.py --argv beamApplyForce.yml

-   The data assimilation is exectued by running: runSofa exampleParamId_SDA.py --argv beamApplyForce.yml

-   The example requires Pardiso plugin and ImageMeshAux plugin.

More examples will be added soon. 



Benchmarks
----------

In order to facilitate development of Optimus, it is highly recommended to create benchmarks or regression tests. 
The goal of these is to verify that the functionality of Optimus has not been changed. Thus, the main goal of benchmarks is to 
compare results of simulations to previously generated reliable data. If the comparison shows important difference, it is necesary 
to identify the source of this change which probably indicates a serious issue either in Optimus or in SOFA itself. 

Currently, two benchmarks have been implemented: *** assimBC_synthBrick *** and ***   *** 

`assimStiffness_cylinder_yaml`

-   ROUKF-based identification of 10 values of Young's modulus of a heterogeneous cylinder subjected to gravity. See Peterlík and A. Klíma.Towards an efficient data assimilation in physically-based medical simulations, 2015.

`assimBC_synthBrick`

-   ROUKF-based identification of stiffnesses of 16 values of spring stiffness (identification of boundary conditions), see I. Peterlík, N. Haouchine, L. Ručka and S. Cotin. Image-driven Stochastic Identification of Boundary Conditions for Predictive Simulation. In International Conference on Medical Image Computing and Computer-Assisted Intervention, 2017.

Both benchmarks are executed using script verify.sh which runs the observation generation, then executes data assimilation and finally computes differences between obtained values and results stored previously. 


Data
----

The plugin contains several volume (VTK) and surface (STL) meshes used in the examples, benchmarks and scenes. Most of meshes were generated by GMesh generator.
