        #include <sofa/core/ObjectFactory.h>
        #include "UKFilter.inl"
        //#include <sofa/helper/accessor.h>

        namespace sofa
        {

        namespace component
        {

        namespace stochastic
        {

        using namespace defaulttype;



        SOFA_DECL_CLASS(UKFilter)

        // Register in the Factory
        int UKFilterClass = core::RegisterObject("UKFilter")
                #ifndef SOFA_FLOAT
                .add< UKFilter<double, Vec3dTypes> >()
                .add< UKFilter<double, Rigid3dTypes> >(true)

                #endif
                #ifndef SOFA_DOUBLE
                //.add< UKFilter<float> >()
                #endif
                ;

        #ifndef SOFA_FLOAT
        template class SOFA_STOCHASTIC_API UKFilter<double,Vec3dTypes>;
        template class SOFA_STOCHASTIC_API UKFilter<double,Rigid3dTypes>;
        #endif
        #ifndef SOFA_DOUBLE
        //template class SOFA_STOCHASTIC_API UKFilter<float>;
        #endif


        } // namespace stochastic
        } // namespace component
        } // namespace sofa




