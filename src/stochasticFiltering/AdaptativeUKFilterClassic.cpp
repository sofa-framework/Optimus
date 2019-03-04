        #include <sofa/core/ObjectFactory.h>
        #include "AdaptativeUKFilterClassic.inl"
        //#include <sofa/helper/accessor.h>

        namespace sofa
        {

        namespace component
        {

        namespace stochastic
        {

        using namespace defaulttype;



        SOFA_DECL_CLASS(AdaptativeUKFilterClassic)

        // Register in the Factory
        int AdaptativeUKFilterClassicClass = core::RegisterObject("AdaptativeUKFilterClassic")
                #ifndef SOFA_FLOAT
                .add< AdaptativeUKFilterClassic<double> >()

                #endif
                #ifndef SOFA_DOUBLE
                //.add< AdaptativeUKFilterClassic<float> >()
                #endif
                ;

        #ifndef SOFA_FLOAT
        template class SOFA_STOCHASTIC_API AdaptativeUKFilterClassic<double>;
        #endif
        #ifndef SOFA_DOUBLE
        //template class SOFA_STOCHASTIC_API AdaptativeUKFilterClassic<float>;
        #endif


        } // namespace stochastic
        } // namespace component
        } // namespace sofa




