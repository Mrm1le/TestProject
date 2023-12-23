#include <iostream>
#include <vector>
#include <chrono>

#include "parking_scenario/threadpool.h"
#include "parking_scenario/parking_scenario.h"


int main()
{
    
    ThreadPool pool(12);
    std::vector< std::future<int> > results;

    for(int i = 0; i < 8; ++i) {
        results.emplace_back(
            pool.enqueue([i] {
                std::cout << "hello " << i << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                std::cout << "world " << i << std::endl;
                return i*i;
            })
        );
    }

    for(auto && result: results)
        std::cout << result.get() << ' ';
    std::cout << std::endl;

    parking_scenario::PlanParams params;
    params.vehicle_param_file = "../resources/vehicle_param_rx5.yaml";
    params.channle_width = 5.5;
    params.slot_margin = 1.8;
    params.step = 3.0;
    params.scenario_type = parking_scenario::ScenarioType::PARALLEL;

    auto t1=std::chrono::steady_clock::now();

    parking_scenario::scenarioPlan(params, true);        
    auto t2=std::chrono::steady_clock::now();
    std::chrono::duration<double> chrono_s=t2-t1;
    std::cout<<"time usage:"<<chrono_s.count()<<std::endl;
    
    return 0;
}