#include "localization/localization.h"

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv,"localization");
    Localization loc;
    ros::spin();
    return 0;
}