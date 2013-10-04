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
// For more information, visit the Ver340dandi web site:
//      http://verdandi.gforge.inria.fr/


#ifndef VERDANDI_FILE_MODEL_CHECKINGMODEL_CXX


#include "CheckingModel.hxx"


namespace Verdandi
{


    ////////////////////////////////
    // CONSTRUCTOR AND DESTRUCTOR //
    ////////////////////////////////


    //! Constructor.
    template <class Model>
    CheckingModel<Model>::CheckingModel()
    {
    }


    //! Destructor.
    template <class Model>
    CheckingModel<Model>::~CheckingModel()
    {
    }


    ////////////////
    // INITIALIZE //
    ////////////////


    //! Initializes the model.
    /*!
      \param[in] configuration_file configuration file.
    */
    template <class Model>
    void CheckingModel<Model>::Initialize(string configuration_file)
    {
        model_.Initialize(configuration_file);
    }


    //! Initializes the current time step for the model.
    template <class Model>
    void CheckingModel<Model>::InitializeFirstStep()
    {
        model_.InitializeFirstStep();
    }


    //! Initializes the current time step for the model.
    template <class Model>
    void CheckingModel<Model>::InitializeStep()
    {
        model_.InitializeStep();
    }


    ////////////////
    // PROCESSING //
    ////////////////


    //! Advances one step forward in time.
    /*! \f[x^f_{h+1} = \mathcal{M}_h(x^a_h, p_h)\,.\f] */
    template <class Model>
    void CheckingModel<Model>::Forward()
    {
        model_.Forward();
    }


    //! Performs one step backward in adjoint model.
    /*!
      \param[in] observation_term \f$ H^T R^{-1}(y - Hx) \f$.
    */
    template <class Model>
    void CheckingModel<Model>::BackwardAdjoint(state& observation_term)
    {
        model_.BackwardAdjoint(observation_term);
    }


    //! Checks whether the model has finished.
    /*!
      \return True if the simulation is done, false otherwise.
    */
    template <class Model>
    bool CheckingModel<Model>::HasFinished() const
    {
        return model_.HasFinished();
    }


    //! Finalizes the current time step for the model.
    template <class Model>
    void CheckingModel<Model>::FinalizeStep()
    {
        model_.FinalizeStep();
    }


    //! Finalizes the model.
    template <class Model>
    void CheckingModel<Model>::Finalize()
    {
        model_.Finalize();
    }


    ///////////////
    // OPERATORS //
    ///////////////


    //! Applies the model to a given vector.
    /*! The current state of the model is modified.
      \param[in] x a vector.
      \param[in] preserve_state Boolean to indicate if the model state has to
      be preserved.
    */
    template <class Model>
    double CheckingModel<Model>::ApplyOperator(state& x, bool preserve_state)
    {
        // Checks that the full model configuration is preserved.
        {
            state x1(x), x2(x), x3(x);
            double time, time1, time2, time3;
            time = model_.GetTime();
            model_.ApplyOperator(x1, false);
            time1 = model_.GetTime();
            model_.ApplyOperator(x2, false);
            time2 = model_.GetTime();
            model_.ApplyOperator(x3, false);
            time3 = model_.GetTime();
            for(int i = 0; i < x1.GetM(); i++)
                if (x1(i) != x2(i))
                    throw ErrorProcessing("CheckingModel<" + model_.GetName()
                                          + ">::ApplyOperator"
                                          "(state& x, false,"
                                          " bool preserve_state)",
                                          "x1 = x2 but ApplyOperator(x1) != "
                                          "ApplyOperator(x2)\n"
                                          "x1 = x2 = " + to_str(x1) + "\n"
                                          "ApplyOperator(x1)("
                                          + to_str(i) +") = " + to_str(x1(i))
                                          + "\n ApplyOperator(x2)("
                                          + to_str(i) +
                                          ") = " + to_str(x2(i)) + ".");
            for(int i = 0; i < x1.GetM(); i++)
                if (x1(i) != x3(i))
                    throw ErrorProcessing("CheckingModel<" + model_.GetName()
                                          + ">::ApplyOperator"
                                          "(state& x, false,"
                                          " bool preserve_state)",
                                          "x1 = x3 but ApplyOperator(x1) != "
                                          "ApplyOperator(x3)\n"
                                          "x1 = x3 = " + to_str(x1) + "\n"
                                          "ApplyOperator(x1)("
                                          + to_str(i) +") = " + to_str(x1(i))
                                          + "\n ApplyOperator(x2)("
                                          + to_str(i) +
                                          ") = " + to_str(x3(i)) + ".");
            if (time != time1 || time != time2 || time != time3)
                throw ErrorProcessing("CheckingModel<" + model_.GetName() +
                                      ">::ApplyOperator(state& x, false,"
                                      " bool preserve_state)",
                                      "The implementation of Model:"
                                      "ApplyOperator has to preserve time.");
        }

        return model_.ApplyOperator(x, preserve_state);
    }


    //! Applies the tangent linear model to a given vector.
    /*!
      \param[in] x a vector.
    */
    template <class Model>
    double CheckingModel<Model>::ApplyTangentLinearOperator(state& x)
    {
        // Checks that the model state is preserved.
        {
            state x1(x), previous_state(model_.GetNstate());
            Copy(model_.GetState(), previous_state);
            model_.ApplyTangentLinearOperator(x1);
            state& current_state = model_.GetState();
            for(int i = 0; i < current_state.GetM(); i++)
                if (previous_state(i)!= current_state(i))
                    throw ErrorProcessing("CheckingModel<" + model_.GetName()
                                          + ">::ApplyTangentLinearOperator"
                                          "(state& x)",
                                          "ApplyTangentLinearOperator does "
                                          "not preserve state:\n"
                                          "previous_state = " +
                                          to_str(previous_state) + "\n"
                                          "current_state = " +
                                          to_str(previous_state) + "\n.");
        }
        // Checks that the full model configuration is preserved.
        {
            state x1(x), x2(x), x3(x);
            double time, time1, time2, time3;
            time = model_.GetTime();
            model_.ApplyTangentLinearOperator(x1);
            time1 = model_.GetTime();
            model_.ApplyTangentLinearOperator(x2);
            time2 = model_.GetTime();
            model_.ApplyTangentLinearOperator(x3);
            time3 = model_.GetTime();
            for(int i = 0; i < x1.GetM(); i++)
                if (x1(i) != x2(i))
                    throw ErrorProcessing("CheckingModel<" + model_.GetName()
                                          + ">::ApplyTangentLinearOperator"
                                          "(state& x)", "x1 = x2 but "
                                          "ApplyTangentLinearOperator(x1) != "
                                          "ApplyTangentLinearOperator(x2)\n"
                                          "x1 = x2 = " + to_str(x1) + "\n"
                                          "ApplyTangentLinearOperator(x1)(" +
                                          to_str(i) +") = " + to_str(x1(i)) +
                                          "\n ApplyTangentLinearOperator(x2)("
                                          + to_str(i) +
                                          ") = " + to_str(x3(i)) + ".");
            for(int i = 0; i < x1.GetM(); i++)
                if (x1(i) != x3(i))
                    throw ErrorProcessing("CheckingModel<" + model_.GetName()
                                          + ">::ApplyTangentLinearOperator"
                                          "(state& x)", "x1 = x3 but "
                                          "ApplyTangentLinearOperator(x1) != "
                                          "ApplyTangentLinearOperator(x3)\n"
                                          "x1 = x3 = " + to_str(x1) + "\n"
                                          "ApplyTangentLinearOperator(x1)(" +
                                          to_str(i) +") = " + to_str(x1(i)) +
                                          "\n ApplyTangentLinearOperator(x2)("
                                          + to_str(i) +
                                          ") = " + to_str(x3(i)) + ".");
            if (time != time1 || time != time2 || time != time3)
                throw ErrorProcessing("CheckingModel<" + model_.GetName() +
                                      ">::ApplyTangentLinearOperator(state&)",
                                      "The implementation of Model::"
                                      "ApplyTangentLinearOperator has to "
                                      "preserve time.");
        }
        return model_.ApplyTangentLinearOperator(x);
    }


    //! Gets the tangent linear model.
    /*!
      \param[out] A the matrix of the tangent linear model.
    */
    template <class Model>
    typename CheckingModel<Model>::tangent_linear_operator&
    CheckingModel<Model>::GetTangentLinearOperator()
    {
        return model_.GetTangentLinearOperator();
    }


    ////////////////////
    // ACCESS METHODS //
    ////////////////////


    //! Returns the current time.
    /*!
      \return The current time.
    */
    template <class Model>
    double CheckingModel<Model>::GetTime()
    {
        return model_.GetTime();
    }


    //! Sets the time of the model to a given time.
    /*!
      \param[in] time a given time.
    */
    template <class Model>
    void CheckingModel<Model>::SetTime(double time)
    {
        model_.SetTime(time);
        if (model_.GetTime() != time)
            throw ErrorProcessing("CheckingModel<" + model_.GetName() +
                                  ">::SetTime(double time)",
                                  "The model was set to time "
                                  + to_str(time) + " but model_.GetTime() = "
                                  + to_str(model_.GetTime()) + ".");
    }


    //! Returns the state vector size.
    /*!
      \return The state vector size.
    */
    template <class Model>
    int CheckingModel<Model>::GetNstate()
    {
        if (model_.GetNstate() != model_.GetState().GetM())
            throw ErrorProcessing("CheckingModel<" + model_.GetName() +
                                  ">::GetNstate()",
                                  "model_.GetNstate()  = "
                                  + to_str(model_.GetNstate()) + " should be "
                                  "equal to model_.GetState().GetM() = " +
                                  to_str(model_.GetState().GetM()) + ".");
        return model_.GetNstate();
    }


    //! Returns the size of the full state vector.
    /*!
      \return The size of the full state vector.
    */
    template <class Model>
    int CheckingModel<Model>::GetNfull_state()
    {
        if (model_.GetNfull_state() != model_.GetFullState().GetM())
            throw ErrorProcessing("CheckingModel<" + model_.GetName() +
                                  ">::GetNfull_state()",
                                  "model_.GetNstate()  = "
                                  + to_str(model_.GetNfull_state()) +
                                  " should be equal to model_.GetFullState()"
                                  ".GetM() = " +
                                  to_str(model_.GetFullState().GetM()) + ".");

        return model_.GetNfull_state();
    }


    //! Provides the state vector.
    /*!
      \param[out] state the reduced state vector.
    */
    template <class Model>
    typename CheckingModel<Model>::state& CheckingModel<Model>::GetState()
    {
        return model_.GetState();
    }


    //! Sets the state vector.
    /*! Before setting the reduced state vector, special requirements can be
      enforced; e.g. positivity requirement or inferior and superior limits.
      \param[in] state the reduced state vector.
    */
    template <class Model>
    void CheckingModel<Model>::StateUpdated()
    {
        model_.StateUpdated();
    }


    //! Provides the state lower bound.
    /*!
      \param[out] lower_bound the state lower bound (componentwise).
    */
    template <class Model>
    typename CheckingModel<Model>::state& CheckingModel<Model>
    ::GetStateLowerBound()
    {
        return model_.GetStateLowerBound();
    }


    //! Provides the state upper bound.
    /*!
      \param[out] upper_bound the state upper bound (componentwise).
    */
    template <class Model>
    typename CheckingModel<Model>::state& CheckingModel<Model>
    ::GetStateUpperBound()
    {
        return model_.GetStateUpperBound();
    }


    //! Provides the full state vector.
    /*!
      \param[out] state the full state vector.
    */
    template <class Model>
    typename CheckingModel<Model>::state& CheckingModel<Model>::GetFullState()
    {
        return model_.GetFullState();
    }


    //! Sets the full state vector.
    /*!
      \param[in] state the full state vector.
    */
    template <class Model>
    void CheckingModel<Model>::FullStateUpdated()
    {
        model_.FullStateUpdated();
    }


    //! Returns the adjoint state vector.
    /*!
      \param[out] state_adjoint the adjoint state vector.
    */
    template <class Model>
    typename CheckingModel<Model>::state&
    CheckingModel<Model>::GetAdjointState()
    {
        return model_.GetAdjointState();
    }


    //! Sets the adjoint state vector.
    /*!
      \param[out] state_adjoint the adjoint state vector.
    */
    template <class Model>
    void CheckingModel<Model>::AdjointStateUpdated()
    {
        model_.SetAdjointStateUpdated();
    }


    ////////////
    // ERRORS //
    ////////////


    //! Computes a row of the variance of the state error.
    /*!
      \param[in] row row index.
      \param[out] P_row the row with index \a row in the state error variance.
    */
    template <class Model>
    typename CheckingModel<Model>::state_error_variance_row&
    CheckingModel<Model>::GetStateErrorVarianceRow(int row)
    {
        return model_.GetStateErrorVarianceRow(row);
    }


    //! Returns the state error variance.
    /*!
      \return The state error variance.
    */
    template <class Model>
    typename CheckingModel<Model>::state_error_variance&
    CheckingModel<Model>::GetStateErrorVariance()
    {
        return model_.GetStateErrorVariance();
    }


    /*! Returns the matrix L in the decomposition of the
      state error covariance matrix (\f$B\f$) as a product \f$LUL^T\f$.
    */
    /*!
      \return The matrix \f$L\f$.
    */
    template <class Model>
    typename CheckingModel<Model>::state_error_variance&
    CheckingModel<Model>::GetStateErrorVarianceProjector()
    {
        int ml = model_.GetStateErrorVarianceProjector().GetM();
        int nl = model_.GetStateErrorVarianceProjector().GetN();
        int nx = model_.GetNstate();
        int mu = model_.GetStateErrorVarianceReduced().GetM();
        if (ml != nx)
            throw ErrorArgument("CheckingModel<" + model_.GetName() + ">::"
                                "GetStateErrorVarianceProjector()",
                                "The matrix L (in the decomposition of the"
                                " state error covariance matrix as a product"
                                " LUL^T) has " +  to_str(ml) + " rows while "
                                "the state vector has " + to_str(nx) +
                                " elements.");
        if (nl != mu)
            throw ErrorArgument("CheckingModel<" + model_.GetName() + ">::"
                                "GetStateErrorVarianceProjector()",
                                "The matrix L (in the decomposition of the"
                                " state error covariance matrix as a product"
                                " LUL^T) has " +  to_str(nl) + " columns "
                                "while the matrix U has " + to_str(mu) +
                                " rows.");

        return model_.GetStateErrorVarianceProjector();
    }


    /*! Returns the matrix U in the decomposition of the
      state error covariance matrix (\f$B\f$) as a product \f$LUL^T\f$.
    */
    /*!
      \return The matrix \f$U\f$.
    */
    template <class Model>
    typename CheckingModel<Model>::state_error_variance_reduced&
    CheckingModel<Model>::GetStateErrorVarianceReduced()
    {
        int mu = model_.GetStateErrorVarianceReduced().GetM();
        int nu = model_.GetStateErrorVarianceReduced().GetN();
        if (nu != mu)
            throw ErrorArgument("CheckingModel<" + model_.GetName() + ">::"
                                "GetStateErrorVarianceReduced()",
                                "The matrix U (in the decomposition of the"
                                " state error covariance matrix as a product"
                                " LUL^T) has " +  to_str(mu) + " rows  and "
                                + to_str(nu) + " columns while U should be a "
                                "square matrix.");

        return model_.GetStateErrorVarianceReduced();
    }


    //! Returns the name of the class.
    /*!
      \return The name of the class.
    */
    template <class Model>
    string CheckingModel<Model>::GetName() const
    {
        return "CheckingModel";
    }


    //! Receives and handles a message.
    /*
      \param[in] message the received message.
    */
    template <class Model>
    void CheckingModel<Model>::Message(string message)
    {
        model_.Message(message);
    }


}

#define VERDANDI_FILE_MODEL_CHECKINGMODEL_CXX
#endif
