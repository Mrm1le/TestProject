#include "gtest/gtest.h"
#include "common/refline/reference_line_provider2.h"
#include "common/refline.h"

using namespace msquare;

TEST(PNC_TEST, PATH_PLANNER_TEST) {
    std::unique_ptr<msquare::ReferenceLineProvider> refline_provider_ = 
                        std::make_unique<msquare::ReferenceLineProvider>();
    std::vector<ReferenceLinePointDerived> segment_line;
    std::vector<double> x{
    23.809666, 22.117211,  20.424757, 18.732302, 18.133288, 17.039848, 15.347395, 13.654941, 
    13.360188, 11.962488, 10.270036, 8.577583, 6.885131, 5.192679, 3.500228, 2.529421, 
    1.807777, 0.115326, -1.567616,-3.250558,-3.640392,-4.943008,-6.625949,
    -8.308890, -9.991831, -11.684279, -13.367219,-15.050159,-16.733098,-17.759976,
    -18.425545,-20.108484,-21.791422,-23.474360,-25.166806,-26.849743,-27.258754,
    -28.532680,-30.215617,-31.908061,-33.590997,-35.273933,-36.956869,-38.649312,
    -40.332247,-42.015181,-43.593526,-45.171870,-46.284318,-46.750215,-48.328559,
    -49.783297,-49.906902,-51.485246,-53.073097,-54.651440,-56.229782,-57.808124,
    -59.386466,-60.964808,-62.543149};

    std::vector<double> y{
    -14.673022, -13.641870, -12.599630,-11.568478,-11.202585,-10.526237,-9.495084,-8.452843,-8.275441,-7.421690,-6.379448,
    -5.348294,-4.306052,-3.274898,-2.232655,-1.641174,-1.201500,-0.159256,0.882987,1.914143,
    2.158073,2.956387,3.998632,5.029788,6.072033,7.114278,8.145436,9.187682,10.229928,10.861928,11.261087,
    12.303333,13.345580,14.387827,15.418987,16.461234,16.714536,17.503482,18.534643,19.576891,
    20.619140,21.650302,22.692551,23.734801,24.765963,25.808213,26.772849,27.748573,28.424927,
    28.713209,29.688934,30.575956,30.653571,31.618208,32.593933,33.558571,34.523209,
    35.498935,36.463573,37.439300,38.403939};

    for (int i = 0; i < x.size(); ++i) {
        ReferenceLinePointDerived p;
        p.enu_point.x = x[i];
        p.enu_point.y = y[i];
        segment_line.push_back(p);
    }


    int loop = 100;
    for (int i = 0; i < loop; i++) {
        std::vector<double> smoother_vx_m, smoother_vy_m;
        double smooth_bound = 0.01;
        refline_provider_->SetLatBound(smooth_bound);
        refline_provider_->SetTrajectoryPoints(segment_line);
        refline_provider_->GetReferenceLine(true);
        refline_provider_->GetSmoothTrajectory(smoother_vx_m, smoother_vy_m);

        std::cout << "smooth x : " << smoother_vx_m.back() << " y : " << smoother_vy_m.back() <<
        " " << smoother_vx_m[smoother_vx_m.size()-2] << " " << smoother_vy_m[smoother_vy_m.size()-2] <<
        " " << smoother_vx_m[smoother_vx_m.size()-3] << " " << smoother_vy_m[smoother_vy_m.size()-3] <<
        std::endl;
    }
}