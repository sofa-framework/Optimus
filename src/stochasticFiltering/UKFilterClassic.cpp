        #include <sofa/core/ObjectFactory.h>
        #include "UKFilterClassic.inl"
        //#include <sofa/helper/accessor.h>

        namespace sofa
        {

        namespace component
        {

        namespace stochastic
        {

        using namespace defaulttype;



        SOFA_DECL_CLASS(UKFilterClassic)

        // Register in the Factory
        int UKFilterClassicClass = core::RegisterObject("UKFilterClassic")
                #ifndef SOFA_FLOAT
                .add< UKFilterClassic<double> >()

                #endif
                #ifndef SOFA_DOUBLE
                //.add< UKFilterClassic<float> >()
                #endif
                ;

        #ifndef SOFA_FLOAT
        template class SOFA_STOCHASTIC_API UKFilterClassic<double>;
        #endif
        #ifndef SOFA_DOUBLE
        //template class SOFA_STOCHASTIC_API UKFilterClassic<float>;
        #endif


        } // namespace stochastic
        } // namespace component
        } // namespace sofa




