#include "irsl/shm_controller.h"
#include "irsl/realtime_task.h"
#include "irsl/simple_yaml_parser.hpp"
#include <iostream>

using namespace irsl_common_utils;
using namespace irsl_shm_controller;
using namespace irsl_realtime_task;

/**
 * @brief ポート・周期などのインタフェース設定
 */
struct HardwareIFSettings
{
    double period;           ///< 制御周期（秒）
    std::string port_name;   ///< デバイス名（例: "/dev/ttyUSB0"）
    int32_t baud_rate;       ///< 通信速度（例: 1000000）
};
/**
 * @brief Dynamixelの設定項目（Control Itemと値）
 */
typedef struct
{
    std::string item_name;
    int32_t value;
} ItemValue;

/**
 * @brief 個々のDynamixelに関する情報
 */
struct DynamixelInfo
{
    std::string name;
    int8_t id;
    std::vector<ItemValue> dxl_setting;
};

namespace YAML
{
    //// YAML auto conversion
    template <>
    struct convert<HardwareIFSettings>
    {
        static bool decode(const Node &node, HardwareIFSettings &cType)
        {
            cType.period = node["period"].as<double>();
            cType.port_name = node["port_name"].as<std::string>();
            cType.baud_rate = node["baud_rate"].as<int32_t>();
            return true;
        }
    };

}

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

class DynamixelInterface
{
public:
    /**
     * @brief インタフェースクラスのコンストラクタ
     */
    DynamixelInterface();

    /**
     * @brief デストラクタ
     */
    ~DynamixelInterface();

    /**
     * @brief YAML設定に基づき、Dynamixelに初期設定値を書き込む
     *
     * @return true 全設定成功
     * @return false 一部失敗
     */
    bool initDynamixels();

    /**
     * @brief YAMLファイルを読み込み、Dynamixelの設定と初期化を行う
     *
     * @param yaml_file 設定ファイル（YAML形式）のパス
     * @return true 成功
     * @return false 失敗（ファイル読み込みやパースエラー）
     */
    bool initialize(const std::string yaml_file);

    /**
     * @brief Dynamixel Workbench を初期化する
     *
     * @param port_name デバイス名（例: "/dev/ttyUSB0"）
     * @param baud_rate 通信速度（例: 1000000）
     * @return true 初期化成功
     * @return false 初期化失敗
     */
    bool initWorkbench(const std::string port_name, const uint32_t baud_rate);

    /**
     * @brief 接続されているDynamixelを検出し、モデル情報を出力する
     *
     * @return true 全てのDynamixelと通信成功
     * @return false 一部通信失敗
     */
    bool loadDynamixels();

    /**
     * @brief 必要なControl Item（制御項目）を取得して記録する
     *
     * @return true 成功
     * @return false 必要な項目が取得できなかった
     */
    bool initControlItems();

    /**
     * @brief SDKのSyncRead/SyncWriteハンドラを登録する
     *
     * @return true 成功
     * @return false 登録失敗
     */
    bool initSDKHandlers();

    /**
     * @brief Dynamixelの登録情報の数を取得する
     *
     * @return size_t 登録されたDynamixelの個数
     */
    size_t get_dx_info_size();

    /**
     * @brief 現在のDynamixelのステータスを取得する
     *
     * @param pos_vec 出力: 角度データ（生値）
     * @param vel_vec 出力: 速度データ（生値）
     * @param cur_vec 出力: 電流データ（生値）
     */
    void getDynamixelStatus(
        std::vector<int32_t> &pos_vec,
        std::vector<int32_t> &vel_vec,
        std::vector<int32_t> &cur_vec);

    /**
     * @brief 角度データ（生値）をラジアンに変換する
     *
     * @param pos_vec 入力の角度値（Dynamixel生値）
     * @param pos_float_vec 出力の角度（ラジアン）
     */
    void convertPosition(
        const std::vector<int32_t> &pos_vec,
        std::vector<irsl_float_type> &pos_float_vec);

    /**
     * @brief 速度データ（生値）を変換する
     *
     * @param vel_vec 入力の速度値（Dynamixel生値）
     * @param vel_float_vec 出力の速度（ユーザ定義単位）
     */
    void convertVelocity(
        const std::vector<int32_t> &vel_vec,
        std::vector<irsl_float_type> &vel_float_vec);

    /**
     * @brief 電流データ（生値）を変換する
     *
     * @param cur_vec 入力の電流値（Dynamixel生値）
     * @param cur_float_vec 出力の電流（mAなど）
     */
    void convertCurrent(
        const std::vector<int32_t> &cur_vec,
        std::vector<irsl_float_type> &cur_float_vec);

    /**
     * @brief トルク（= 電流）データを変換する
     *
     * @note 現時点ではconvertCurrentと同様の変換を行う
     *
     * @param cur_vec 入力の電流値
     * @param torque_float_vec 出力のトルク値（単位未定義）
     */
    void convertTorque(
        const std::vector<int32_t> &cur_vec,
        std::vector<irsl_float_type> &torque_float_vec);

    /**
     * @brief ラジアン単位でDynamixelに角度指令を送信する
     *
     * @param pos_float_vec 入力: 指令角度（ラジアン）
     * @return true 成功
     * @return false 送信失敗
     */
    bool writePosition(
        const std::vector<irsl_float_type> &pos_float_vec);

private:
    DynamixelWorkbench *dxl_wb_;
    HardwareIFSettings h_settings;
    std::vector<DynamixelInfo> dx_info;
    std::map<std::string, const ControlItem *> control_items_;
};

DynamixelInterface::DynamixelInterface()
    : dxl_wb_(new DynamixelWorkbench)
{
}

DynamixelInterface::~DynamixelInterface()
{
    delete dxl_wb_;
}

bool DynamixelInterface::initialize(const std::string yaml_file)
{
    YAML::Node n;
    try
    {
        // check fname
        n = YAML::LoadFile(yaml_file);
    }
    catch (const std::exception &)
    {
        std::cerr << "parameter file [" << yaml_file << "] can not open" << std::endl;
        return false;
    }

    bool res;

    res = readValue<HardwareIFSettings>(n, "HardwareIFSettings", h_settings);
    if (res)
    {
        // std::cerr << "HardwareIFSettings: " << std::endl;
        // // std::cerr << "  dxl_read_period: " << h_settings.dxl_read_period << std::endl;
        // // std::cerr << "  dxl_write_period: " << h_settings.dxl_write_period << std::endl;
        // std::cerr << "  period: " << h_settings.period << std::endl;
    }
    else
    {
        std::cerr << "fail :struct:" << std::endl;
        return false;
    }

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
    return this->initWorkbench(h_settings.port_name, h_settings.baud_rate);
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

int main(int argc, char **argv)
{

    std::string fname = "test.yaml";
    if (argc >= 2)
    {
        fname = std::string(argv[1]);
    }

    DynamixelInterface di;
    bool ret;

    ret = di.initialize(fname);
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

    ShmSettings ss;
    ss.hash = n["SHMSettings"]["hash"].as<int32_t>();
    ss.shm_key = n["SHMSettings"]["shm_key"].as<int32_t>();

    ss.numJoints = di.get_dx_info_size();
    ss.numForceSensors = 0;
    ss.numImuSensors = 0;
    ss.jointType = ShmSettings::PositionCommand; //| ShmSettings::PositionGains;

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
    sm.writeVelocityCommand(cur_vel_float_vec);
    sm.writeTorqueCommand(cur_torque_float_vec);

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
            std::cout << i << " " << cur_pos_float_vec[i] << " " << cur_vel_float_vec[i] << " " << cur_torque_float_vec[i] << std::endl;
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