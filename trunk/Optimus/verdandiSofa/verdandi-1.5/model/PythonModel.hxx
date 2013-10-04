// Copyright (C) 2011-2012 INRIA
// Author(s): Kévin Charpentier, Vivien Mallet
//
// This file is part of the data assimilation library Verdandi.
//
// Verdandi is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the Free
// Software Foundation; either version 2.1 of the License, or (at your option)
// any later version.
//
// Verdandi is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
// more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with Verdandi. If not, see http://www.gnu.org/licenses/.
//
// For more information, visit the Verdandi web site:
//      http://verdandi.gforge.inria.fr/


#ifndef VERDANDI_FILE_MODEL_PYTHONMODEL_CXX

#undef VERDANDI_PYTHON_INCLUDE
#define VERDANDI_PYTHON_INCLUDE(v) <python v

#include VERDANDI_PYTHON_INCLUDE(VERDANDI_PYTHON_VERSION)/Python.h>
#include <numpy/arrayobject.h>

namespace Verdandi
{

    class PythonModel: public VerdandiBase
    {

    public:
        //! Type of the state vector.
        typedef Vector<double> state;

        //! Type of the state error variance.
        typedef Matrix<double> state_error_variance;
        /*! \brief Type of the reduced matrix \f$U\f$ in the \f$LUL^T\f$
          decomposition of the background error covariance matrix. */
        typedef Matrix<double> state_error_variance_reduced;
        //! Type of a row of the state error variance.
        typedef Vector<double> state_error_variance_row;
        //! Type of the state/observation crossed matrix.
        typedef Matrix<double> matrix_state_observation;
        //! Type of the tangent linear model.
        typedef Matrix<double> tangent_linear_operator;
        //! Type of the model error variance.
        typedef Matrix<double> error_variance;
        //! Type of an uncertain parameter.
        typedef Vector<double> uncertain_parameter;

    protected:

        //! Instance of the Python model class.
        PyObject *pyModelInstance_;
        //! Name of the python model file.
        PyObject *pyModelFile_;
        //! Imported model module from Python.
        PyObject *pyModelModule_;
        //! Dictionary object that describes the module's namespace.
        PyObject *pyModelDict_;
        //! Model class from the module.
        PyObject *pyModelClass_;
        //! Has the Python module been initialized?
        bool is_module_initialized_;

        //! Name of the Python module that contains the model class.
        string module_;
        /*! \brief Directory to include in "sys.path" for the module to be
          found. If no directory is to be added to "sys.path", this
          attribute remains empty. */
        string directory_;
        //! Name of the Python model class.
        string class_name_;

        //! State vector.
        Vector<double> state_;
        //! Full state vector.
        Vector<double> full_state_;

        //! State upper bound.
        state upper_bound_;
        //! State lower bound.
        state lower_bound_;

        //! Adjoint state.
        state state_adjoint_;

        //! State error variance.
        state_error_variance state_error_variance_;
        //! State error variance inverse.
        state_error_variance state_error_variance_inverse_;

        //! Number of the row of state_error_variance_ currently stored.
        int current_row_;
        //! Value of the row of state_error_variance_ currently stored.
        state_error_variance_row state_error_variance_row_;

        /*! \brief Projector matrix L in the decomposition of the
          background error covariance matrix (\f$B\f$) as a product LUL^T */
        state_error_variance state_error_variance_projector_;
        /*! \brief Reduced matrix U in the decomposition of the
          background error covariance matrix (\f$B\f$) as a product LUL^T */
        state_error_variance_reduced state_error_variance_reduced_;

        //! Tangent linear operator (H).
        tangent_linear_operator tangent_linear_operator_;

        //! Uncertain parameter.
        uncertain_parameter uncertain_parameter_;
        //! Correlations between the parameters.
        Vector<double> parameter_correlation_;
        //! Name of the probability distribution.
        string parameter_pdf_;
        //! Covariance matrix.
        Matrix<double> parameter_variance_;
        //! PDF parameters.
        Vector<double> parameter_parameter_;
        //! Perturbation option.
        string parameter_option_;

    public:

        // Constructors and destructor.
        PythonModel();
        PythonModel(string configuration_file);
        ~PythonModel();
        // Initializations.
        void Initialize(string configuration_file);
        void InitializeStep();

        // Processing.
        void Forward();
        void BackwardAdjoint(state& observation_term);
        bool HasFinished() const;
        void Save();
        void FinalizeStep();
        void Finalize();

        // Operators.
        double ApplyOperator(state& x, bool preserve_state = true);
        double ApplyTangentLinearOperator(state& x);
        tangent_linear_operator& GetTangentLinearOperator();

        // Access methods.
        double GetTime() const;
        void SetTime(double time);
        int GetNstate() const;
        int GetNfull_state() const;
        state& GetState();
        void StateUpdated();
        state& GetStateLowerBound();
        state& GetStateUpperBound();
        state& GetFullState();
        void FullStateUpdated();
        state& GetAdjointState();
        void AdjointStateUpdated();

        // Uncertainty.
        int GetNparameter();
        uncertain_parameter& GetParameter(int i);
        void SetParameter(int i, uncertain_parameter& parameter);
        Vector<double>& GetParameterCorrelation(int i);
        string GetParameterPDF(int i);
        Matrix<double>& GetParameterVariance(int i);
        Vector<double>& GetParameterParameter(int i);
        string GetParameterOption(int i);

        // Errors.
        state_error_variance_row& GetStateErrorVarianceRow(int row);
        state_error_variance& GetStateErrorVariance();
        state_error_variance& GetStateErrorVarianceProjector();
        state_error_variance_reduced& GetStateErrorVarianceReduced();
        const state_error_variance& GetStateErrorVarianceInverse();

        string GetName() const;
        void Message(string message);

    private:
        string ErrorMessageNotContiguous(string function_name) const;

    };
} //namespace Verdandi

#define VERDANDI_FILE_MODEL_PYTHONMODEL_HXX
#endif
