#pragma once

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <iostream>
#include "irsl/shm_controller.h"
// #include "irsl/simple_yaml_parser.hpp"
// #include "common.h"
#include <yaml-cpp/yaml.h>

using namespace irsl_shm_controller;
// using namespace irsl_common_utils;

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

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
     * @param n YAMLのノードデータ
     * @return true 成功
     * @return false 失敗（ファイル読み込みやパースエラー）
     */
    bool initialize(YAML::Node& n);

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
    // HardwareIFSettings h_settings;
    std::vector<DynamixelInfo> dx_info;
    std::map<std::string, const ControlItem *> control_items_;
};
