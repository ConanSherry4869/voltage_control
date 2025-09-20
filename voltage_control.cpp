/*
 * 文件：voltage_control.cpp
 * 功能：台区储能系统双向PI电压调节控制器
 * 作者：LWF
 * 日期：2025/9/19
 * 版本：1.0
 *
 * 功能描述：
 * 1. 基于电压偏差自动调节储能充放电功率
 * 2. 具备过压充电和欠压放电双向调节能力
 * 3. 集成SOC保护功能防止电池过充过放
 * 4. 支持JSON配置文件动态加载参数
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <synchapi.h>
#include "cJSON.h"

/* ---------- 系统配置参数(从json文件读取) ---------- */
typedef struct {
    // 电压相关参数
    float V_ref_upper;      // 电压上限设定值，如241.0
    float V_ref_lower;      // 电压下限设定值，如198.0
    float Deadband_upper;   // 上限控制死区，如2.0
    float Deadband_lower;   // 下限控制死区，如2.0
    float V_enter_lower;    // 电压进入门槛，如160.0

    // PI控制器参数
    float Kp_upper;         // 过压控制比例系数
    float Ki_upper;         // 过压控制积分系数
    float Kp_lower;         // 欠压控制比例系数
    float Ki_lower;         // 欠压控制积分系数

    // 功率限制参数
    float P_step_max;       // 功率需求最大步长，如10.0 (kW)
    float P_charge_max;     // PCS最大充电功率，如125.0 (kW)
    float P_discharge_max;  // PCS最大放电功率，如125.0 (kW)
    float SOC_max;          // SOC安全上限，如0.95 (95%)
    float SOC_min;          // SOC安全下限，如0.15 (15%)
} SystemConfig_Cfg;


/* ---------- 系统实时状态 ---------- */
typedef struct {
    float V_meas;           // 实时电压测量值 (来自智能电表)
    float SOC;              // 储能当前SOC (来自BMS)
    float P_meas;           // PCS当前功率 (来自PCS) 正为充电，负为放电
    float P_soc_charge_limit;   // SOC计算出的当前最大允许充电功率 (基于SOC)
    float P_soc_discharge_limit;// SOC计算出的当前最大允许放电功率 (基于SOC)
} SystemStatus_RealTime;


/* ---------- 控制器内部状态 ---------- */
typedef struct {
    int Ctrl_Mode;          // 控制模式状态: 0-正常, 1-过压, 2-欠压
    float integral_upper;   // 过压PI控制器的积分项累积值
    float integral_lower;   // 欠压PI控制器的积分项累积值
} ControllerState;

// 定义全局变量
SystemConfig_Cfg sys_cfg;
SystemStatus_RealTime realtime_status;
ControllerState ctrl_state;

// 模式判断函数
int Determine_CtrlMode(float V_meas, SystemConfig_Cfg cfg) {
    if (V_meas > (cfg.V_ref_upper + cfg.Deadband_upper)) {
        return 1; // 过压状态
    } else if (V_meas < (cfg.V_ref_lower - cfg.Deadband_lower) && V_meas > cfg.V_enter_lower) {
        return 2; // 欠压状态
    } else {
        return 0; // 正常状态
    }
}

// 过压控制计算函数
float Calculate_OverVoltage_Control(void) {
    float effective_error;
    float P_calc;
    float P_cmd_final;

    // 1. 计算有效偏差
    effective_error = realtime_status.V_meas - (sys_cfg.V_ref_upper + sys_cfg.Deadband_upper);
    if (effective_error < 0) {
        effective_error = 0; // 如果误差为负，说明已在死区内，无需动作
    }

    // 2. PI计算 (比例项 + 积分项)
    ctrl_state.integral_upper += effective_error * sys_cfg.Ki_upper; // 积分累积
    P_calc = effective_error * sys_cfg.Kp_upper + ctrl_state.integral_upper;

    // 3. 功率步长限制
    if (P_calc > sys_cfg.P_step_max) {
        P_calc = sys_cfg.P_step_max;
    }

    // 4. 计算最终指令：P_cmd = min(P_calc + P_meas, P_charge_max, P_soc_charge_limit)
    // P_calc是“需要增加的充电功率”，所以要加上当前功率P_meas
    P_cmd_final = P_calc + realtime_status.P_meas;

    // 进行三重最小值的限幅
    if (P_cmd_final > realtime_status.P_soc_charge_limit) {
        P_cmd_final = realtime_status.P_soc_charge_limit;
    }
    if (P_cmd_final > sys_cfg.P_charge_max) {
        P_cmd_final = sys_cfg.P_charge_max;
    }
    // 确保指令是正的（充电）
    if (P_cmd_final < 0) {
        P_cmd_final = 0;
    }

    return P_cmd_final;
}

// 欠压控制计算函数
float Calculate_UnderVoltage_Control(void) {
    float effective_error;
    float P_calc; // PI计算出的需要“增加”的放电功率（恒为正值）
    float P_discharge_capacity; // 当前系统最大允许的放电功率（正值）
    float P_cmd_target; // PI计算出的目标总功率
    float P_cmd_final; // 经过所有限制后的最终指令

    // 1. 计算有效偏差 (注意方向)
    effective_error = (sys_cfg.V_ref_lower - sys_cfg.Deadband_lower) - realtime_status.V_meas;
    if (effective_error < 0) {
        effective_error = 0; // 如果误差为负，说明已在死区内，无需动作
    }

    // 2. PI计算 (比例项 + 积分项)
    ctrl_state.integral_lower += effective_error * sys_cfg.Ki_lower;

    P_calc = effective_error * sys_cfg.Kp_lower + ctrl_state.integral_lower;

    // 3. 功率步长限制 (P_calc是本次计算出的功率增量，需限制其最大变化幅度)
    if (P_calc > sys_cfg.P_step_max) {
        P_calc = sys_cfg.P_step_max;
    }

    // 4. 计算PI控制器期望的总功率目标
    // P_calc是“需要增加的放电功率”（正值），所以要从当前功率（负值）中减去。
    P_cmd_target = realtime_status.P_meas - P_calc;

    // 5. 计算当前系统最大允许放电能力
    P_discharge_capacity = sys_cfg.P_discharge_max; // 先取PCS的限制
    if (realtime_status.P_soc_discharge_limit < P_discharge_capacity) {
        P_discharge_capacity = realtime_status.P_soc_discharge_limit; // SOC限制更严格
    }

    // 将其转化为负值，作为指令的下限。
    float P_cmd_lower_limit = -P_discharge_capacity;

    // 6. 对目标指令进行最终限幅
    // 确保指令不会要求充电（即限制上限为0）
    if (P_cmd_target > 0.0) {
        P_cmd_final = 0.0;
    }
    // 确保指令不会超过最大放电能力（即限制下限为-P_discharge_capacity）
    else if (P_cmd_target < P_cmd_lower_limit) {
        P_cmd_final = P_cmd_lower_limit;
    }
    // 如果目标值在合理范围内，则采用目标值
    else {
        P_cmd_final = P_cmd_target;
    }


    return P_cmd_final;
}


/**
 * @brief 计算基于SOC的充放电功率限制,在过渡区间内使用平滑的S形曲线
 * @param soc 当前电池SOC（0.0-1.0）
 * @param cfg 系统配置参数
 * @param charge_limit [输出] 计算出的最大允许充电功率
 * @param discharge_limit [输出] 计算出的最大允许放电功率
 */
void Calculate_SOC_Power_Limits(float soc, SystemConfig_Cfg cfg,
                                float* charge_limit, float* discharge_limit) {
    // 使用平滑的S形曲线（sigmoid函数）过渡，避免功率突变
    const float charge_transition_width = 0.05f;  // 充电过渡区间宽度
    const float discharge_transition_width = 0.05f; // 放电过渡区间宽度

    // 充电限制：使用S形曲线在SOC_max附近平滑过渡到0
    float charge_factor;
    if (soc >= cfg.SOC_max) {
        charge_factor = 0.0;
    } else if (soc <= (cfg.SOC_max - charge_transition_width)) {
        charge_factor = 1.0;
    } else {
        // 在过渡区间内使用平滑的S形曲线
        float x = (soc - (cfg.SOC_max - charge_transition_width)) / charge_transition_width;
        charge_factor = 0.5 * (1.0 + cos(M_PI * x)); // 使用余弦函数实现平滑过渡
    }
    *charge_limit = cfg.P_charge_max * charge_factor;

    // 放电限制：使用S形曲线在SOC_min附近平滑过渡到0
    float discharge_factor;
    if (soc <= cfg.SOC_min) {
        discharge_factor = 0.0f;
    } else if (soc >= (cfg.SOC_min + discharge_transition_width)) {
        discharge_factor = 1.0f;
    } else {
        // 在过渡区间内使用平滑的S形曲线
        float x = (soc - cfg.SOC_min) / discharge_transition_width;
        discharge_factor = 0.5f * (1.0f - cos(M_PI * x)); // 使用余弦函数实现平滑过渡
    }
    *discharge_limit = cfg.P_discharge_max * discharge_factor;

    // 确保限制值合理（非负）
    if (*charge_limit < 0.0f) *charge_limit = 0.0f;
    if (*discharge_limit < 0.0f) *discharge_limit = 0.0f;
}

// 模拟实时数据函数，
void Simulate_RealTimeData(SystemStatus_RealTime *status) {
    static int simulation_step = 0;
    simulation_step++;

    // 模拟电压变化：在190V-250V之间正弦波动，周期约30秒（加快变化）
    float base_voltage = 220.0f;
    float voltage_variation = 30.0f * sin(2 * M_PI * simulation_step / 30.0f);
    status->V_meas = base_voltage + voltage_variation;

    // 模拟SOC变化：加快变化速度
    static float simulated_soc = 0.7f; // 初始SOC为70%

    // 根据电压情况模拟SOC变化（增加变化幅度）
    if (status->V_meas > 235.0f) {
        simulated_soc += 0.02f; // 过压时充电，SOC快速增加
    } else if (status->V_meas < 205.0f) {
        simulated_soc -= 0.02f; // 欠压时放电，SOC快速减少
    } else {
        simulated_soc -= 0.005f; // 正常时缓慢放电
    }

    // 添加随机扰动，使SOC变化更明显
    float random_perturbation = (rand() % 100 - 50) / 1000.0f; // -0.05到+0.05的随机变化
    simulated_soc += random_perturbation;

    // 限制SOC在合理范围内
    if (simulated_soc > 0.95f) simulated_soc = 0.95f;
    if (simulated_soc < 0.15f) simulated_soc = 0.15f;

    status->SOC = simulated_soc;

    // 模拟当前功率（基于电压偏差）
    status->P_meas = (status->V_meas - 220.0f) * 2.0f;

}

// 主控制循环
void Main_VoltageControlLoop(void) {
    // 1. 读取实时数据 (需要您实现硬件接口通信)
    Simulate_RealTimeData(&realtime_status);
    // SOC高时，充电功率受限；SOC低时，放电功率受限
    Calculate_SOC_Power_Limits(realtime_status.SOC,
                               sys_cfg,
                               &realtime_status.P_soc_charge_limit,
                               &realtime_status.P_soc_discharge_limit
    );
    printf("模拟数据: V_meas=%.2fV, SOC=%.1f%%, P_meas=%.2fkW, P_soc_charge_limit=%.2fkW, P_soc_discharge_limit=%.2fkW\n",
           realtime_status.V_meas, realtime_status.SOC * 100, realtime_status.P_meas,
           realtime_status.P_soc_charge_limit, realtime_status.P_soc_discharge_limit);

    // 2. 判断当前工作模式
    ctrl_state.Ctrl_Mode = Determine_CtrlMode(realtime_status.V_meas, sys_cfg);

    // 3. 根据模式执行相应的控制逻辑
    float P_cmd = 0.0; // 最终要发送给PCS的功率指令

    switch (ctrl_state.Ctrl_Mode) {
        case 0: // 正常模式
            P_cmd = 0.0f; // 或执行其他调度计划
            // 退出控制模式，清零积分器防止下次进入时冲击
            ctrl_state.integral_upper = 0.0f;
            ctrl_state.integral_lower = 0.0f;
            break;

        case 1: // 过压控制模式
            P_cmd = Calculate_OverVoltage_Control();
            break;

        case 2: // 欠压控制模式
            P_cmd = Calculate_UnderVoltage_Control();
            break;

        default:
            P_cmd = 0.0f;
            break;
    }

    // 4. 发送指令给PCS
    printf("控制模式状态=%d,有功功率指令=%f\n",ctrl_state.Ctrl_Mode,P_cmd);
    printf("******************************************************\n");
    fflush(stdout); // 强制刷新输出缓冲区

}

/**
 * @brief 从指定的JSON文件中加载配置
 * @param filename 配置文件名（"config.json"）
 * @return int 成功返回0，失败返回-1
 */
int load_configuration(const char *filename) {
    FILE *fp = NULL;
    long file_size;
    char *file_content = NULL;
    cJSON *root_json = NULL;
    cJSON *voltage_json = NULL;
    cJSON *pi_json = NULL;
    cJSON *power_json = NULL;

    // 1. 打开文件
    fp = fopen(filename, "r");
    if (!fp) {
        fprintf(stderr, "错误: 无法打开配置文件 %s\n", filename);
        return -1;
    }

    // 2. 获取文件大小并读取内容到内存
    fseek(fp, 0, SEEK_END);
    file_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    file_content = (char *)malloc(file_size + 1);
    if (!file_content) {
        fclose(fp);
        fprintf(stderr, "错误: 内存分配失败\n");
        return -1;
    }
    fread(file_content, 1, file_size, fp);
    file_content[file_size] = '\0'; // 添加字符串结束符
    fclose(fp);

    // 3. 解析JSON字符串
    root_json = cJSON_Parse(file_content);
    free(file_content); // 释放原始文件内容内存
    if (!root_json) {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr) {
            fprintf(stderr, "JSON解析错误: %s\n", error_ptr);
        }
        return -1;
    }

    // 4. 逐项提取配置
    voltage_json = cJSON_GetObjectItemCaseSensitive(root_json, "voltage_settings");
    pi_json = cJSON_GetObjectItemCaseSensitive(root_json, "pi_controller");
    power_json = cJSON_GetObjectItemCaseSensitive(root_json, "power_limits");

    // 检查各个对象是否存在
    if (!cJSON_IsObject(voltage_json) || !cJSON_IsObject(pi_json) || !cJSON_IsObject(power_json)) {
        fprintf(stderr, "错误: 配置文件格式不正确，缺少必要的字段\n");
        cJSON_Delete(root_json);
        return -1;
    }

    // 4.1 读取电压相关参数
    sys_cfg.V_ref_upper = cJSON_GetObjectItemCaseSensitive(voltage_json, "V_ref_upper")->valuedouble;
    sys_cfg.V_ref_lower = cJSON_GetObjectItemCaseSensitive(voltage_json, "V_ref_lower")->valuedouble;
    sys_cfg.Deadband_upper = cJSON_GetObjectItemCaseSensitive(voltage_json, "Deadband_upper")->valuedouble;
    sys_cfg.Deadband_lower = cJSON_GetObjectItemCaseSensitive(voltage_json, "Deadband_lower")->valuedouble;
    sys_cfg.V_enter_lower = cJSON_GetObjectItemCaseSensitive(voltage_json, "V_enter_lower")->valuedouble;

    // 4.2 读取PI控制器参数
    sys_cfg.Kp_upper = cJSON_GetObjectItemCaseSensitive(pi_json, "Kp_upper")->valuedouble;
    sys_cfg.Ki_upper = cJSON_GetObjectItemCaseSensitive(pi_json, "Ki_upper")->valuedouble;
    sys_cfg.Kp_lower = cJSON_GetObjectItemCaseSensitive(pi_json, "Kp_lower")->valuedouble;
    sys_cfg.Ki_lower = cJSON_GetObjectItemCaseSensitive(pi_json, "Ki_lower")->valuedouble;

    // 4.3 读取功率限制参数
    sys_cfg.P_step_max = cJSON_GetObjectItemCaseSensitive(power_json, "P_step_max")->valuedouble;
    sys_cfg.P_charge_max = cJSON_GetObjectItemCaseSensitive(power_json, "P_charge_max")->valuedouble;
    sys_cfg.P_discharge_max = cJSON_GetObjectItemCaseSensitive(power_json, "P_discharge_max")->valuedouble;
    sys_cfg.SOC_max = cJSON_GetObjectItemCaseSensitive(power_json, "SOC_max")->valuedouble;
    sys_cfg.SOC_min = cJSON_GetObjectItemCaseSensitive(power_json, "SOC_min")->valuedouble;

    // 5. 清理cJSON对象树
    cJSON_Delete(root_json);
    printf("配置加载成功!\n");
    return 0;
}



int main(int argc, char *argv[])
{

    // 加载配置文件
    if (load_configuration("config.json") != 0) {
        fprintf(stderr, "程序启动失败：配置文件错误。\n");
        return EXIT_FAILURE;
    }

    // 查看部分读取信息
    printf("V_ref_upper=%f\n", sys_cfg.V_ref_upper);
    printf("V_ref_lower=%f\n",sys_cfg.V_ref_lower);
    printf("Deadband_upper=%f\n", sys_cfg.Deadband_upper);
    printf("Deadband_lower=%f\n", sys_cfg.Deadband_lower);
    printf("V_enter_lower=%f\n", sys_cfg.V_enter_lower);
    printf("Kp_upper=%f\n", sys_cfg.Kp_upper);
    printf("Ki_upper=%f\n", sys_cfg.Ki_upper);
    printf("Kp_lower=%f\n", sys_cfg.Kp_lower);
    printf("Ki_lower=%f\n", sys_cfg.Ki_lower);
    printf("P_step_max=%f\n", sys_cfg.P_step_max);
    printf("P_charge_max=%f\n", sys_cfg.P_charge_max);
    printf("P_discharge_max=%f\n", sys_cfg.P_discharge_max);
    printf("SOC_max=%f\n", sys_cfg.SOC_max);
    printf("SOC_min=%f\n", sys_cfg.SOC_min);

    printf("=== 台区储能双向PI电压调节模拟 ===\n");
    // 初始化控制器状态
    ctrl_state.Ctrl_Mode = 0;
    ctrl_state.integral_upper = 0.0f;
    ctrl_state.integral_lower = 0.0f;

    // 进入主控制循环
    while(1)
    {
        Main_VoltageControlLoop();
        // 休眠1秒，模拟定时循环 1s=1000ms
        Sleep(1000);
    }

    return 0;
}

