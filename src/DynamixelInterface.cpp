#include "DynamixelInterface.h"

DynamixelInterface::DynamixelInterface()
    : dxl_wb_(new DynamixelWorkbench)
{
}

DynamixelInterface::~DynamixelInterface()
{
    delete dxl_wb_;
}

bool DynamixelInterface::initialize(YAML::Node &n)
{
    // YAML::Node n;
    // try
    // {
    //     // check fname
    //     n = YAML::LoadFile(yaml_file);
    // }
    // catch (const std::exception &)
    // {
    //     std::cerr << "parameter file [" << yaml_file << "] can not open" << std::endl;
    //     return false;
    // }

    bool res;

    // res = readValue<HardwareIFSettings>(n, "HardwareIFSettings", h_settings);
    // if (res)
    // {
    //     // std::cerr << "HardwareIFSettings: " << std::endl;
    //     // // std::cerr << "  dxl_read_period: " << h_settings.dxl_read_period << std::endl;
    //     // // std::cerr << "  dxl_write_period: " << h_settings.dxl_write_period << std::endl;
    //     // std::cerr << "  period: " << h_settings.period << std::endl;
    // }
    // else
    // {
    //     std::cerr << "fail :struct:" << std::endl;
    //     return false;
    // }

    std::string port_name = n["HardwareIFSettings"]["port_name"].as<std::string>();
    int32_t baud_rate = n["HardwareIFSettings"]["baud_rate"].as<int32_t>();

    dx_info.clear();
    for (auto it_j = n["Joint"].begin(); it_j != n["Joint"].end(); ++it_j)
    {
        DynamixelInfo info;
        std::string name = it_j->first.as<std::string>();
        auto values = it_j->second;

        info.name = name;
        info.id = -1;
        for (auto it = values.begin(); it != values.end(); ++it)
        {
            std::string key = it->first.as<std::string>();
            if (key == "ID")
            {
                info.id = (int8_t)(it->second.as<int32_t>());
            }
            else
            {
                int32_t value = it->second.as<int32_t>();
                ItemValue item_value = {key, value};
                info.dxl_setting.push_back(item_value);
            }
        }
        dx_info.push_back(info);
        // std::cout << info.name << " " << (int32_t)info.id << std::endl;
    }
    return this->initWorkbench(port_name, baud_rate);
}

bool DynamixelInterface::initWorkbench(const std::string port_name, const uint32_t baud_rate)
{
    bool result = false;
    const char *log;

    result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
    if (result == false)
    {
        std::cerr << log << std::endl;
    }

    return result;
}

bool DynamixelInterface::initDynamixels()
{
    const char *log;

    for (auto info : dx_info)
    {
        uint8_t id = info.id;
        dxl_wb_->torqueOff(id, &log);
        for (auto const &setting : info.dxl_setting)
        {
            // std::cout << (int32_t)id << " " <<setting.item_name.c_str() << " " << setting.value << std::endl;
            bool result = dxl_wb_->itemWrite(id, setting.item_name.c_str(), setting.value, &log);
            if (result == false)
            {
                std::cerr << log << std::endl;
                std::cerr << "Failed to write value[" << setting.value << "] on items[" << setting.item_name << "] to Dynamixel[Name : " << info.name << ", ID : " << id << "]" << std::endl;
                return false;
            }
        }

        dxl_wb_->torqueOn(id, &log);
    }

    return true;
}

bool DynamixelInterface::loadDynamixels(void)
{
    bool result = false;
    const char *log;

    for (auto const &dxl : dx_info)
    {
        uint16_t model_number = 0;
        uint8_t id = (uint8_t)dxl.id;
        result = dxl_wb_->ping(id, &model_number, &log);
        if (result == false)
        {
            std::cerr << log << std::endl;
            std::cerr << "Can't find Dynamixel ID " << id << std::endl;
            return result;
        }
        else
        {
            std::cout << "Name : " << dxl.name << ", ID : " << (int32_t)dxl.id << ", Model Number : " << model_number << std::endl;
        }
    }

    return result;
}

bool DynamixelInterface::initControlItems(void)
{
    if (dx_info.empty())
    {
        std::cerr << "No Dynamixel info available." << std::endl;
        return false;
    }

    const char *log = nullptr;

    // 最初の1つから取得（共通前提）
    uint8_t sample_id = dx_info.front().id;

    std::vector<std::string> keys = {
        "Goal_Position", "Goal_Velocity",
        "Present_Position", "Present_Velocity", "Present_Current"};

    for (const auto &key : keys)
    {
        const ControlItem *item = dxl_wb_->getItemInfo(sample_id, key.c_str());

        // 代替キーも考慮
        if (item == nullptr)
        {
            if (key == "Goal_Velocity")
                item = dxl_wb_->getItemInfo(sample_id, "Moving_Speed");
            else if (key == "Present_Velocity")
                item = dxl_wb_->getItemInfo(sample_id, "Present_Speed");
            else if (key == "Present_Current")
                item = dxl_wb_->getItemInfo(sample_id, "Present_Load");
        }

        if (item == nullptr)
        {
            std::cerr << "Failed to get ControlItem: " << key << std::endl;
            return false;
        }

        control_items_[key] = item;
    }

    return true;
}

bool DynamixelInterface::initSDKHandlers(void)
{
    bool result = false;
    const char *log = NULL;

    //   auto it = dynamixel_.begin();

    result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address, control_items_["Goal_Position"]->data_length, &log);
    if (result == false)
    {
        std::cerr << log << std::endl;
        return result;
    }
    else
    {
        std::cout << log << std::endl;
    }

    result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length, &log);
    if (result == false)
    {
        std::cerr << log << std::endl;
        return result;
    }
    else
    {
        std::cout << log << std::endl;
    }

    if (dxl_wb_->getProtocolVersion() == 2.0f)
    {
        uint16_t start_address = std::min(control_items_["Present_Position"]->address, control_items_["Present_Current"]->address);

        /*
          As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.
        */
        // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
        uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length + 2;

        result = dxl_wb_->addSyncReadHandler(start_address,
                                             read_length,
                                             &log);
        if (result == false)
        {
            std::cerr << log << std::endl;
            return result;
        }
    }

    return result;
}

size_t DynamixelInterface::get_dx_info_size()
{
    return dx_info.size();
}

void DynamixelInterface::convertPosition(
    const std::vector<int32_t> &pos_vec,
    std::vector<irsl_float_type> &pos_float_vec)
{
    if (pos_vec.size() != pos_float_vec.size())
    {
        pos_float_vec.resize(pos_vec.size());
    }
    for (int i = 0; i < get_dx_info_size(); i++)
    {
        irsl_float_type angle = dxl_wb_->convertValue2Radian(dx_info[i].id, pos_vec[i]);
        pos_float_vec[i] = angle;
    }
}

void DynamixelInterface::convertVelocity(
    const std::vector<int32_t> &vel_vec,
    std::vector<irsl_float_type> &vel_float_vec)
{
    if (vel_vec.size() != vel_float_vec.size())
    {
        vel_float_vec.resize(vel_vec.size());
    }
    for (int i = 0; i < get_dx_info_size(); i++)
    {
        irsl_float_type vel = dxl_wb_->convertValue2Velocity(dx_info[i].id, vel_vec[i]);
        vel_float_vec[i] = vel;
    }
}

void DynamixelInterface::convertCurrent(
    const std::vector<int32_t> &cur_vec,
    std::vector<irsl_float_type> &cur_float_vec)
{
    if (cur_vec.size() != cur_float_vec.size())
    {
        cur_float_vec.resize(cur_vec.size());
    }
    for (int i = 0; i < get_dx_info_size(); i++)
    {
        irsl_float_type cur = dxl_wb_->convertValue2Current(dx_info[i].id, cur_vec[i]);
        cur_float_vec[i] = cur;
    }
}

void DynamixelInterface::convertTorque(
    const std::vector<int32_t> &cur_vec,
    std::vector<irsl_float_type> &torque_float_vec)
{
    if (cur_vec.size() != torque_float_vec.size())
    {
        torque_float_vec.resize(cur_vec.size());
    }
    for (int i = 0; i < get_dx_info_size(); i++)
    {
        irsl_float_type tor = dxl_wb_->convertValue2Current(dx_info[i].id, cur_vec[i]);
        torque_float_vec[i] = tor;
    }
}

bool DynamixelInterface::writePosition(
    const std::vector<irsl_float_type> &pos_float_vec)
{
    bool result = false;
    const char *log = NULL;

    uint8_t id_vec_size = get_dx_info_size();
    std::vector<uint8_t> id_vec(id_vec_size);
    std::vector<int32_t> dynamixel_position(id_vec_size);
    for (int i = 0; i < id_vec_size; i++)
    {
        id_vec[i] = dx_info[i].id;
        dynamixel_position[i] = dxl_wb_->convertRadian2Value(id_vec[i], pos_float_vec[i]);
    }

    result = dxl_wb_->syncWrite(
        SYNC_WRITE_HANDLER_FOR_GOAL_POSITION,
        id_vec.data(), id_vec_size,
        dynamixel_position.data(), 1, &log);
    if (result == false)
    {
        std::cerr << log << std::endl;
    }
    return result;
}

void DynamixelInterface::getDynamixelStatus(
    std::vector<int32_t> &pos_vec,
    std::vector<int32_t> &vel_vec,
    std::vector<int32_t> &cur_vec)
{
    bool result = false;
    const char *log = NULL;

    uint8_t id_vec_size = get_dx_info_size();
    std::vector<uint8_t> id_vec(id_vec_size);
    for (int i = 0; i < id_vec_size; i++)
    {
        id_vec[i] = dx_info[i].id;
    }

    if (pos_vec.size() != id_vec_size)
    {
        pos_vec.resize(id_vec_size);
    }
    if (vel_vec.size() != id_vec_size)
    {
        vel_vec.resize(id_vec_size);
    }
    if (cur_vec.size() != id_vec_size)
    {
        cur_vec.resize(id_vec_size);
    }

    result = dxl_wb_->syncRead(
        SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
        id_vec.data(), id_vec_size,
        &log);
    if (!result)
    {
        //   ROS_ERROR("syncRead failed for group %s: %s", group_name.c_str(), log);
        std::cerr << "syncRead failed " << log << std::endl;
        // continue; // グループ単位でスキップ
        return;
    }

    result = dxl_wb_->getSyncReadData(
        SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
        id_vec.data(), id_vec_size,
        control_items_["Present_Position"]->address,
        control_items_["Present_Position"]->data_length,
        pos_vec.data(),
        &log);
    if (!result)
    {
        //   ROS_ERROR("getSyncReadData position failed for group %s: %s", group_name.c_str(), log);
        std::cerr << "getSyncReadData position failed " << log << std::endl;
    }

    result = dxl_wb_->getSyncReadData(
        SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
        id_vec.data(), id_vec_size,
        control_items_["Present_Velocity"]->address,
        control_items_["Present_Velocity"]->data_length,
        vel_vec.data(),
        &log);
    if (result == false)
    {
        // ROS_ERROR("getSyncReadData velocity failed for group %s: %s", group_name.c_str(), log);
        std::cerr << "getSyncReadData velocity failed " << log << std::endl;
    }

    result = dxl_wb_->getSyncReadData(
        SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
        id_vec.data(), id_vec_size,
        control_items_["Present_Current"]->address,
        control_items_["Present_Current"]->data_length,
        cur_vec.data(),
        &log);
    if (result == false)
    {
        // ROS_ERROR("getSyncReadData current failed for group %s: %s", group_name.c_str(), log);
        std::cerr << "getSyncReadData current failed " << log << std::endl;
    }
}
