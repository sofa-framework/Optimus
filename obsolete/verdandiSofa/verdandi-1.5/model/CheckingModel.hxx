// Copyright (C) 2012 INRIA
// Author(s): Marc Fragu
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


#ifndef VERDANDI_FILE_MODEL_CHECKINGMODEL_HXX


namespace Verdandi
{


    ////////////////////
    // CHECKING MODEL //
    ////////////////////


    //! This class is a model template.
    template <class Model>
    class CheckingModel: public VerdandiBase
    {
    public:
        //! Type of a row of the background error variance.
        typedef typename Model::state_error_variance_row
        state_error_variance_row;
        //! Type of the model state vector.
        typedef typename Model::state state;
        //! Type of the background error variance.
        typedef typename Model::state_error_variance
        state_error_variance;
        //! Type of the tangent linear model.
        typedef typename Model::tangent_linear_operator
        tangent_linear_operator;
        //! Type of the model/observation crossed matrix.
        typedef typename Model::matrix_state_observation
        matrix_state_observation;
        /*! \brief Type of the reduced matrix \f$U\f$ in the \f$LUL^T\f$
          decomposition of the background error covariance matrix. */
        typedef typename Model::state_error_variance_reduced
        state_error_variance_reduced;

        //! The model to be tested.
        Model model_;


    public:

        // Constructor and destructor.
        CheckingModel();
        ~CheckingModel();
        void Initialize(string configuration_file);
        void InitializeFirstStep();
        void InitializeStep();

        // Processing.
        void Forward();
        void BackwardAdjoint(state& observation_term);
        bool HasFinished() const;

        void FinalizeStep();
        void Finalize();

        // Operators.
        double ApplyOperator(state& x, bool preserve_state = true);
        double ApplyTangentLinearOperator(state& x);
        tangent_linear_operator& GetTangentLinearOperator();

        // Access methods.
        double GetTime();
        void SetTime(double time);
        int GetNstate();
        int GetNfull_state();
        state& GetState();
        void StateUpdated();
        state& GetFullState();
        void FullStateUpdated();
        state& GetStateLowerBound();
        state& GetStateUpperBound();
        state& GetAdjointState();
        void AdjointStateUpdated();

        // Errors.
        state_error_variance_row& GetStateErrorVarianceRow(int row);
        state_error_variance& GetStateErrorVariance();
        state_error_variance& GetStateErrorVarianceProjector();
        state_error_variance_reduced& GetStateErrorVarianceReduced();
        const state_error_variance& GetStateErrorVarianceInverse() const;

        string GetName() const;
        void Message(string message);
    };


} // namespace Verdandi.


#define VERDANDI_FILE_MODEL_CHECKINGMODEL_HXX
#endif
