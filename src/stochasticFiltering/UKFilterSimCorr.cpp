        #include <sofa/core/ObjectFactory.h>
        #include "UKFilterSimCorr.inl"
        //#include <sofa/helper/accessor.h>

        namespace sofa
        {

        namespace component
        {

        namespace stochastic
        {

        using namespace defaulttype;



        SOFA_DECL_CLASS(UKFilterSimCorr)

        // Register in the Factory
        int UKFilterSimCorrClass = core::RegisterObject("UKFilterSimCorr")
                #ifndef SOFA_FLOAT
                .add< UKFilterSimCorr<double> >()

                #endif
                #ifndef SOFA_DOUBLE
                //.add< UKFilterSimCorr<float> >()
                #endif
                ;

        #ifndef SOFA_FLOAT
        template class SOFA_STOCHASTIC_API UKFilterSimCorr<double>;
        #endif
        #ifndef SOFA_DOUBLE
        //template class SOFA_STOCHASTIC_API UKFilterSimCorr<float>;
        #endif


        } // namespace stochastic
        } // namespace component
        } // namespace sofa




