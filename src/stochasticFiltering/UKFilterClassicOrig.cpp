        #include <sofa/core/ObjectFactory.h>
        #include "UKFilterClassicOrig.inl"
        //#include <sofa/helper/accessor.h>

        namespace sofa
        {

        namespace component
        {

        namespace stochastic
        {

        using namespace defaulttype;



        SOFA_DECL_CLASS(UKFilterClassicOrig)

        // Register in the Factory
        int UKFilterClassicOrigClass = core::RegisterObject("UKFilterClassicOrig")
                #ifndef SOFA_FLOAT
                .add< UKFilterClassicOrig<double> >()

                #endif
                #ifndef SOFA_DOUBLE
                //.add< UKFilterClassicOrig<float> >()
                #endif
                ;

        #ifndef SOFA_FLOAT
        template class SOFA_STOCHASTIC_API UKFilterClassicOrig<double>;
        #endif
        #ifndef SOFA_DOUBLE
        //template class SOFA_STOCHASTIC_API UKFilterClassicOrig<float>;
        #endif


        } // namespace stochastic
        } // namespace component
        } // namespace sofa




