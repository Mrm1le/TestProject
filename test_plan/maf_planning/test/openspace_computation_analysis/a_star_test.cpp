#include "a_star_test.h"


void printHelp();
int main(int argc, char const *argv[]) {
    initConfig();
  
    if(argc <3){
        printHelp();
    }
    
    switch (argv[1][1])
    {
        case 'p':
            testPlanner(argc, argv);       
            break;
        case 's':
            displayScene(argc, argv);
        default:
            break;
    }

    // testPointSplit(argc, argv);  // time cost for point
    // testPLB(argc, argv);
    return 0;
}

void printHelp(){
    std::cout<<"\n****************** USAGE INSTRUCTION ********************"<<std::endl;
    std::cout<<"@argv[1]: -p, for planning"<<std::endl;
    std::cout<<"          -s, for the scene display"<<std::endl;
    std::cout<<"@argv[2]: file/folder, the config yaml file or folder"<<std::endl;
    std::cout<<"@argv[3]: -s, <optional>, if this is specified, planning will not plot the planning result"<<std::endl;
    std::cout<<"***********************************************************\n"<<std::endl;
}
