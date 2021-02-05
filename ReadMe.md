Advanced methods of state estimation and parameter identification in SOFA
=========================================================================

Optimus plugin was created to provide a testing environment for data-driven physics-based modeling (typically finite elements, FE).
While actually the plugin implements only stochastic methods based on Kalman filtering, its architecture allows for the implementation of
generic prediction--correction schemes where the model is employed as a predictor and correction is performed using given observation data. 


The concepts of estimation of state and parameter identification are summarized in a document in doc/OptimusDoc. To generate the
PDF, compile the tex file as pdflatex ConceptsBehindOptimus.tex.

Doxygen documentation [HTML and LaTeX] can be generated using the existing Doxygen file located in folder doc:
Doxygen Optimus.doxygen.




Compilation and Usage
=====================

In order to obtain the plugin, it is necessary to perform git clone of the plugin to your local PC with SOFA installed.


Requirements
------------

Except for SOFA, the plugin dependencies are BLAS, Eigen, pthread. It is highly recommended to use Pardiso solver with Optimus, as other solvers in SOFA are less reliable and might impact the estimation.


In-tree build
-------------

To compile:

-   Path to the plugin is to be added to SOFA_EXTERNAL_DIRECTORIES in SOFA Cmake configuration.

-   To compile the stochastic components, set STOCHASTIC_FILTERING macro.


Out-of-tree build
-----------------

This plugin could be compiled with out-of-tree builds. You might need to add the Sofa installation path to the CMake prefix path. If you compiled Sofa in directory $SOFA_ROOT/build, consider doing an install step (make install, ninja install, etc.).

To compile:

-   Add Sofa installation path to the plugin (example cmake -DCMAKE_PREFIX_PATH=$SOFA_ROOT/build/install ..)

-   To compile the stochastic components, set STOCHASTIC_FILTERING macro.




Components
==========

Generic components
------------------

These components provide various functionality employed in Optimus scenes. 

`OptimParams`

-   Container of Gaussian stochastic variables given by expected value and standard deviation.

`OptimMonitor`

-   Component to store observations in SOFA Monitor format, extending the functionality of SOFA Monitor.

`SimulatedStateObservationSource`

-   Tool for reading observations exported previously in SOFA Monitor format.

`SimulatedStateObservationStreamer`

-   Tool for reading observations from another component on SOFA scene (unstable).

`VTKExporterDA`

-   Component extending the functionality of SOFA VTKExporter (reaction to different events).

`StochasticPositionHandler`

-   Another component to save the data of mechanical object during stochastic simulation.

`SigmaPointsVTKExporter`

-   Component extending the functionality of SOFA VTKExporter to save the mechanical object for every sigma point.

`AddNoiseEngine`

-   Component to add the noise (for now only Gaussian is avaialble) with specified direction, mean, and variance.



Stochastic filtering
--------------------

These components are the core of Optimus plugin and implement functionality necessary for stochastic estimation.
Compilation of these components must be activated by CMake macro STOCHASTIC_FILTERING.

`FilteringAnimationLoop`

-   Compliant with SOFA API for animation loops.

-   Calls filter to perform the assimilation in each step.

-   Issues special events: PredictionEndEvent after calling the prediction step of the filter, and CorrectionEndEvent after calling the correction step of the filter.


`StochasticStateWrapper`

-   Wrapper of model provided by SOFA. This wrapper implements the interface between the stochastic components (filters) and SOFA simulation. 

-   Must be placed inside the SOFA subnode containing the physical simulation components. 

-   Requires MechanicalObject and assumes OptimParams in the same node. 


`PreStochasticWrapper`

-   Special version of wrapper which allows for including other simulated object into Optimus scene which do not contain any quantities being estimated.

-   Typical example is an obstacle which displays a physical behaviour, however, none of its features is directly involved in the estimation.


`MappedStateObservationManager`

-   One instance of observation manager which handles the observations and provides them to the filter via computation of innovation.

-   Assumes that the predicted observations are mapped to the main mechanical object (associated with StochasticStateWrapper).



`UKFilterClassic`

-   Unscented Kalman Filter proposed by Julier and Uhlman (1997).

-   Main methods are initializeStep(), computePrediction() and computeCorrection(), all called by the FilteringAnimationLoop.


`ROUKFilter`

-   Reduced order Kalman filter implemented according to Philippe Moireau and Dominique Chapelle "Reduced-order Unscented Kalman Filtering with application to parameter identification in large-dimensional systems."

-   Main methods are initializeStep(), computePrediction() and computeCorrection(), all called by the FilteringAnimationLoop.


`UKFilterSimCorr`

-   Special version of UKF filter purely for data assimilation.

-   Main methods are initializeStep(), computePrediction() and computeCorrection(), all called by the FilteringAnimationLoop.


`EnTKFilter`

-   Ensemble Transform Kalman filter. Its description is given in Axel Hutt (2020) "Divergence of the ensemble transform kalman filter by nonlocal observations". Instead of sigma points the transformation are performed with Ensemble members.

-   Main methods are initializeStep(), computePrediction() and computeCorrection(), all called by the FilteringAnimationLoop.




Data
----

The plugin contains several volume (VTK) and surface (STL) meshes used in the examples, benchmarks and scenes. Most of meshes were generated by GMesh generator.



Visualization
-------------

Optimus plugin has several python scripts that allow to show the dynamics of estimated parameters (stiffnesses), variance values, and correlation between parameters. All scripts use Matplotlib python library to draw figures and charts and YAML configuration files to load main scene parameters.


All visualization scripts are stored in python_src/visualisation subfolder in Optimus. To scripts generally require only path to output folder(s) with results, since special subfolders structure, which is recognized by scripts, is generated and YAML is copied to output folder during data assimilation process.


List of scripts:

-   DA_visu.py - script to plot estimated parameters, their standart deviation, and innovation values; path to output folder as command line argument is required

-   correlation_visu.py - script to plot correlation values between parameters; path to output folder as command line argument is required

-   DA_comparison_visu.py - script to plot estimated parameters, their standart deviation, and innovation values for different cases; paths to output folders is stored in inputList variable and case descriptions in labels variable in the header of the script

-   correlation_comparison_visu.py - script to plot correlation values between parameters for different cases; paths to output folders is stored in inputList variable and case descriptions in labels variable in the header of the script

-   DA_visu.py - script to plot estimated parameters, their standart deviation, and innovation values; path to output folder as command line argument is required

-   performance_visu.py - script to plot performance statistics for the estimation process; json output from SOFA AdvancedTimer is needed

-   performance_comparison_visu.py - script to plot performance statistics for the estimation process for different situations; json output from SOFA AdvancedTimer is needed; paths to output folders is stored in inputList variable and case descriptions in labels variable in the header of the script

-   performance_comparison_with_parameter_2d_visu.py - an attempt to plot performance statistics for two parameters as 3d surface; json output from SOFA AdvancedTimer is needed; description are stored in performance_labels and parameter_labels variables; a special names for output folders are needed



Examples
--------

In general, Optimus scenes are written in Python(3) (occasionally in XML). It is recommended to use YAML (Python module) to define parameters of scenes.
Enormously increases the efficiency and avoids mishaps. 

Examples are given in folder examples:

`SingleParameterIdentification`

-   An example showing stochastic identification of Young's modulus of a homogeneous beam subjected to gravity.

-   It's necessary to generate observations first: runSofa identify1YoungMod_GenObs.py

-   The data assimilation is exectued by running: runSofa identify1YoungMod_SDA.py


`MultipleParameterIdentification`

-   An example similar to previous, but estimated 10 Young's moduli of a heterogeneous cylinder under gravity.

-   It's necessary to generate observations first: runSofa identify10YoungMods_GenObs.py

-   The data assimilation is exectued by running: runSofa identify10YoungMods_SDA.py


`SpringStiffnessIdentification`

-   An example showing stochastic identification of spring stiffnesses for a brick deformed due to constraint movement.

-   It's necessary to generate observations first: runSofa identifySpringStiffness_GenObs.py

-   The data assimilation is exectued by running: runSofa identifySpringStiffness_SDA.py





Benchmarks
----------

In order to facilitate development of Optimus, it is highly recommended to create benchmarks or regression tests. 
The goal of these is to verify that the functionality of Optimus has not been changed. Thus, the main goal of benchmarks is to 
compare results of simulations to previously generated reliable data. If the comparison shows important difference, it is necesary 
to identify the source of this change which probably indicates a serious issue either in Optimus or in SOFA itself.

Currently, seven benchmarks have been implemented: *** assimBC_liver_geomagic_cutting_yaml *** , *** assimBC_liver_polynomial_springs_point_cloud_yaml *** , *** assimBC_synthBrick *** , *** assimStiffness_cylinder_geomagic_yaml *** , *** assimStiffness_cylinder_python3_yaml *** , *** assimStiffness_cylinder_UKFSimCorr_yaml *** , and *** assimStiffness_cylinder_yaml ***


`assimBC_liver_geomagic_cutting_yaml`

-   ROUKF-based identification of 3 values of spring stiffnesses for a deformation of a liver model. The liver is deformed following the recorded manipulation of GeoMagic device. At some moment the simulation of ligament cutting is performed, see S. Nikolaev, I. Peterlik, S. Cotin. Stochastic Correction of Boundary Conditions during Liver Surgery, 2018.


`assimBC_liver_polynomial_springs_point_cloud_yaml`

-   ROUKF-based identification of 6 coefficients of 2 two polynomial springs.


`assimBC_synthBrick`

-   ROUKF-based identification of 16 values of spring stiffness (identification of boundary conditions), see I. Peterlík, N. Haouchine, L. Ručka and S. Cotin. Image-driven Stochastic Identification of Boundary Conditions for Predictive Simulation. In International Conference on Medical Image Computing and Computer-Assisted Intervention, 2017.

All benchmarks are executed using script verify.sh which runs the observation generation, then executes data assimilation and finally computes differences between obtained values and results stored previously.


`assimStiffness_cylinder_geomagic_yaml`

-   ROUKF-based identification of 2 values of Young's modulus of a heterogeneous cylinder. The cylinder is deformed following the recorded manipulation of GeoMagic device.


`assimStiffness_cylinder_python3_yaml`

-   ROUKF-based identification of 3 values of Young's modulus of a heterogeneous cylinder subjected to periodic force. A Python3 compatible scene to verify also the functionality of python3 components


`assimStiffness_cylinder_UKFSimCorr_yaml`

-   UKFSimCorr-based identification of 2 values of Young's modulus of a heterogeneous cylinder subjected to gravity.


`assimStiffness_cylinder_yaml`

-   ROUKF-based identification of 10 values of Young's modulus of a heterogeneous cylinder subjected to gravity. See Peterlík and A. Klíma. Towards an efficient data assimilation in physically-based medical simulations, 2015.

