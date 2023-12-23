#include "common/config/vehicle_param.h"
#include "has2_get_next_state_offline/next_state_generator.h"
#include "has2_get_next_state_offline/next_state_visualizer.h"
#include "slog_macro_impl.h"

using namespace msquare;
using namespace std;


bool loadPlanParams(const std::string &apa_file, const std::string &vehicle_param_file){
    
    bool load_success = true;

    load_success &= msquare::VehicleParam::Instance()->loadFile(vehicle_param_file);
  
    load_success &= msquare::HybridAstarConfig::GetInstance()->loadFile(apa_file);
    load_success &= msquare::TrajectoryOptimizerConfig::GetInstance()->loadFile(apa_file);
    load_success &= msquare::CarParams::GetInstance()->loadFile(vehicle_param_file);
    load_success &= msquare::CarParams::GetInstance()->loadFile4Plan(apa_file);
    load_success &= msquare::StrategyParams::GetInstance()->loadFile(apa_file);

    return load_success;
}


int main()
{
    //assuming we are running in test/bag_recurrent/build directory
    //the resource file is in bag_recurrent/resource/
    string strategy_config_filename = "../resources/apa_vertical.yaml";
    string vehicle_config_filename = "../resources/vehicle_param_l.yaml";

    if(loadPlanParams(strategy_config_filename, vehicle_config_filename))
    {
        SLOG_INFO("Load param successful at [strategy]%s [vehicle]%s", strategy_config_filename.c_str(), vehicle_config_filename.c_str());
    }
    else
    {
        SLOG_ERROR("Load param failed at [strategy]%s [vehicle]%s", strategy_config_filename.c_str(), vehicle_config_filename.c_str());
        return -1;
    }

    NextStateGenerator nsg;

    nsg.setParamsFromConfig();

    //set some params manually
    nsg.step_size_ = 1.0;
    //
    if(!nsg.checkParamsConsistencyConfig())
    {
        SLOG_WARN("Not all params are consistent with yaml, manual set configs are only for develop usages");
    } 
    nsg.printAllParams();



    nsg.generateAllOriginNextStates();

    SLOG_INFO("origin_next_states_forward_.size() = %d", (int)nsg.origin_next_states_forward_.size());



    NextStateVisualizer nsv;
    nsv.init(6000,6000, 0.0005);
    nsv.drawGrid(0.1);
    
    for(int i=0; i<(int)nsg.origin_next_states_forward_.size() ; i++)
    {
        SLOG_DEBUG("  origin_next_states_forward_.[%d] = %s",i, nsg.origin_next_states_forward_[i]->getNodeInfoStr().c_str());

        nsv.drawStatePose(nsg.origin_next_states_forward_[i]);
        //nsv.drawStateInfo(i, nsg.origin_next_states_forward_[i]);
    }

    cv::imwrite("./dump.png", nsv.getCanvas());
    //cv::resizeWindow("disiplay",1024,1024);

    SLOG_INFO("Image dumped to ./dump.png");

    return 0;

};