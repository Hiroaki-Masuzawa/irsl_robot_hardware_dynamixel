#include "irsl/shm_controller.h"
#include "irsl/realtime_task.h"
#include "irsl/simple_yaml_parser.hpp"
#include <iostream>

using namespace irsl_common_utils;
using namespace irsl_shm_controller;
using namespace irsl_realtime_task;

#include "DynamixelInterface.h"
#include "common.h"

#include <unordered_map>

static const std::unordered_map<std::string, int> jointTypeMap = {
    {"PositionCommand", ShmSettings::JointType::PositionCommand},
    {"PositionGains", ShmSettings::JointType::PositionGains},
    {"VelocityCommand", ShmSettings::JointType::VelocityCommand},
    {"VelocityGains", ShmSettings::JointType::VelocityGains},
    {"TorqueCommand", ShmSettings::JointType::TorqueCommand},
    {"TorqueGains", ShmSettings::JointType::TorqueGains},
    {"MotorTemperature", ShmSettings::JointType::MotorTemperature},
    {"MotorCurrent", ShmSettings::JointType::MotorCurrent},
};

int main(int argc, char **argv)
{

    std::string fname = "test.yaml";
    if (argc >= 2)
    {
        fname = std::string(argv[1]);
    }

    YAML::Node n;
    try
    {
        // check fname
        n = YAML::LoadFile(fname);
    }
    catch (const std::exception &)
    {
        std::cerr << "parameter file [" << fname << "] can not open" << std::endl;
        return false;
    }

    DynamixelInterface di;
    bool ret;

    ret = di.initialize(n);
    if (!ret)
    {
        std::cerr << "error initialize" << std::endl;
    }

    ret = di.loadDynamixels();
    if (!ret)
    {
        std::cerr << "error loadDynamixels" << std::endl;
    }

    ret = di.initDynamixels();
    if (!ret)
    {
        std::cerr << "error initDynamixels" << std::endl;
    }

    ret = di.initControlItems();
    if (!ret)
    {
        std::cerr << "error initControlItems" << std::endl;
    }

    ret = di.initSDKHandlers();
    if (!ret)
    {
        std::cerr << "error initSDKHandlers" << std::endl;
    }

    ShmSettings ss;
    ss.hash = n["SHMSettings"]["hash"].as<int32_t>();
    ss.shm_key = n["SHMSettings"]["shm_key"].as<int32_t>();

    ss.numJoints = di.get_dx_info_size();
    ss.numForceSensors = 0;
    ss.numImuSensors = 0;
    ss.jointType = 0;

    for (const auto &jtype : n["SHMSettings"]["jointType"])
    {
        auto it = jointTypeMap.find(jtype.as<std::string>());
        if (it != jointTypeMap.end())
        {
            ss.jointType |= it->second;
        }
    }
    std::cout << "jointType : " << ss.jointType << std::endl;

    ShmManager sm(ss);
    bool res;
    res = sm.openSharedMemory(true);
    std::cout << "open: " << res << std::endl;

    res = sm.writeHeader();
    std::cout << "writeHeader: " << res << std::endl;

    std::cout << "isOpen: " << sm.isOpen() << std::endl;

    sm.resetFrame();
    unsigned long interval_us = (unsigned long)(n["HardwareIFSettings"]["period"].as<double>() * 1000000);
    unsigned long interval_ns = (unsigned long)(n["HardwareIFSettings"]["period"].as<double>() * 1000000000);
    IntervalStatistics tm(interval_us);

    int cntr = 0;
    tm.start();

    size_t joint_num = di.get_dx_info_size();
    std::vector<int32_t> cur_pos_vec(joint_num);
    std::vector<int32_t> cur_vel_vec(joint_num);
    std::vector<int32_t> cur_cur_vec(joint_num);
    std::vector<irsl_float_type> cur_pos_float_vec(joint_num);
    std::vector<irsl_float_type> cur_vel_float_vec(joint_num);
    // std::vector<irsl_float_type> cur_float_vec(joint_num);
    std::vector<irsl_float_type> cur_torque_float_vec(joint_num);

    std::vector<irsl_float_type> cmd_pos_float_vec(joint_num);

    di.getDynamixelStatus(cur_pos_vec, cur_vel_vec, cur_cur_vec);

    di.convertPosition(cur_pos_vec, cur_pos_float_vec);
    di.convertVelocity(cur_vel_vec, cur_vel_float_vec);
    di.convertTorque(cur_cur_vec, cur_torque_float_vec);

    sm.writePositionCurrent(cur_pos_float_vec);
    sm.writeVelocityCurrent(cur_vel_float_vec);
    sm.writeTorqueCurrent(cur_torque_float_vec);

    sm.writePositionCommand(cur_pos_float_vec);
    // sm.writeVelocityCommand(cur_vel_float_vec);
    // sm.writeTorqueCommand(cur_torque_float_vec);
    for (int i = 0; i < joint_num; i++)
    {
        // std::cout << i << " " << cur_pos_float_vec[i] << " " << cur_vel_float_vec[i] << " " << cur_torque_float_vec[i] << std::endl;
        std::cout << i << " " << cur_pos_float_vec[i] << " " << cmd_pos_float_vec[i] << std::endl;
    }
    while (true)
    {
        tm.sleepUntil(interval_ns);
        tm.sync();

        di.getDynamixelStatus(cur_pos_vec, cur_vel_vec, cur_cur_vec);
        di.convertPosition(cur_pos_vec, cur_pos_float_vec);
        di.convertVelocity(cur_vel_vec, cur_vel_float_vec);
        // di.convertCurrent(cur_cur_vec, cur_cur_float_vec);
        di.convertTorque(cur_cur_vec, cur_torque_float_vec);

        sm.writePositionCurrent(cur_pos_float_vec);
        sm.writeVelocityCurrent(cur_vel_float_vec);
        sm.writeTorqueCurrent(cur_torque_float_vec);

        sm.readPositionCommand(cmd_pos_float_vec);
        di.writePosition(cmd_pos_float_vec);

#ifdef DEBUG
        for (int i = 0; i < joint_num; i++)
        {
            // std::cout << i << " " << cur_pos_float_vec[i] << " " << cur_vel_float_vec[i] << " " << cur_torque_float_vec[i] << std::endl;
            std::cout << i << " " << cur_pos_float_vec[i] << " " << cmd_pos_float_vec[i] << std::endl;
        }
        std::cout << "--------------------" << std::endl;
#endif

        cntr++;

        sm.incrementFrame();
        if (cntr > 100)
        {
            std::cout << "max: " << tm.getMaxInterval() << std::endl;
            tm.reset();
            cntr = 0;
        }
    }
    // polling

    return 0;
}