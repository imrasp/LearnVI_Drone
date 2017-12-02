#include <iostream>

#include "MAVControl/mavlink_control.h"
#include "SLAMInterface/mono_live_viorb.h"
#include "SLAMInterface/mono_offline_viorb.h"
#include "Utility/systemConfigParam.h"

using namespace std;

int main(int argc, char **argv);


int main(int argc, char **argv) {
    try {
        SystemConfigParam configParam(argc, argv);
        configParam.readParams();
        cout << "Starting main in " << configParam.getMode() << " mode " << endl;

        System_Log system_log(configParam.getRecord_path());

        if (configParam.isBLiveSLAM()) {
            // if (std::string(getResult) == "something") to compare char need to make one to be string
            Mono_Live_VIORB mono_live_viorb(&system_log, &configParam);
            Location_Manager location_manager(&system_log, &mono_live_viorb);
            mono_live_viorb.setLocationManager(&location_manager);
            Mavlink_Control mavlink_control(&system_log, &location_manager, &configParam);
            location_manager.setMavlinkControl(&mavlink_control);

            cout << "Start SLAM thread,..." << endl;
            mono_live_viorb.start();
            location_manager.activateSLAM();
            cout << "Start Mavlink thread,..." << endl;
            mavlink_control.start();

            //stop all thread in order
            mono_live_viorb.stop();
            mavlink_control.stop();
        } else if (configParam.isBMAVonly()) {
            Location_Manager location_manager(&system_log, nullptr);
            Mavlink_Control mavlink_control(&system_log, &location_manager, &configParam);
            location_manager.setMavlinkControl(&mavlink_control);

            cout << "Start Mavlink thread,..." << endl;
            mavlink_control.start();
            mavlink_control.stop();
        } else if (configParam.isBOffline()) {
            Mono_Offline_VIORB mono_offline_viorb(&system_log, &configParam);

            cout << "Start Mavlink thread,..." << endl;
            mono_offline_viorb.start();
        }

        return 0;
    }
    catch (int error) {
        fprintf(stderr, "threw exception %i \n", error);
        return error;
    }
}