//
// Created by HMQ on 2025/9/20.
//
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
/**
 * @brief 从CSV文件中加载配置
 * @param filename CSV文件名
 * @return int 成功返回0，失败返回-1
 */

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
// 定义全局变量
SystemConfig_Cfg sys_cfg;

int load_configuration_from_csv(const char *filename) {
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        fprintf(stderr, "错误: 无法打开CSV文件 %s\n", filename);
        return -1;
    }

    char line[256];
    int line_num = 0;

    while (fgets(line, sizeof(line), fp)) {
        line_num++;

        // 跳过空行和注释行（以#开头的行）
        if (line[0] == '\n' || line[0] == '#' || line[0] == '\r') {
            continue;
        }

        // 移除行尾的换行符
        line[strcspn(line, "\n\r")] = '\0';

        // 分割键值对
        char *key = strtok(line, ",");
        char *value_str = strtok(NULL, ",");

        if (key && value_str) {
            float value = atof(value_str);

            // 根据键名设置对应的配置参数
            if (strcmp(key, "V_ref_upper") == 0) {
                sys_cfg.V_ref_upper = value;
            } else if (strcmp(key, "V_ref_lower") == 0) {
                sys_cfg.V_ref_lower = value;
            } else if (strcmp(key, "Deadband_upper") == 0) {
                sys_cfg.Deadband_upper = value;
            } else if (strcmp(key, "Deadband_lower") == 0) {
                sys_cfg.Deadband_lower = value;
            } else if (strcmp(key, "V_enter_lower") == 0) {
                sys_cfg.V_enter_lower = value;
            } else if (strcmp(key, "Kp_upper") == 0) {
                sys_cfg.Kp_upper = value;
            } else if (strcmp(key, "Ki_upper") == 0) {
                sys_cfg.Ki_upper = value;
            } else if (strcmp(key, "Kp_lower") == 0) {
                sys_cfg.Kp_lower = value;
            } else if (strcmp(key, "Ki_lower") == 0) {
                sys_cfg.Ki_lower = value;
            } else if (strcmp(key, "P_step_max") == 0) {
                sys_cfg.P_step_max = value;
            } else if (strcmp(key, "P_charge_max") == 0) {
                sys_cfg.P_charge_max = value;
            } else if (strcmp(key, "P_discharge_max") == 0) {
                sys_cfg.P_discharge_max = value;
            } else if (strcmp(key, "SOC_max") == 0) {
                sys_cfg.SOC_max = value;
            } else if (strcmp(key, "SOC_min") == 0) {
                sys_cfg.SOC_min = value;
            } else {
                fprintf(stderr, "警告: 第%d行未知的配置项: %s\n", line_num, key);
            }
        } else {
            fprintf(stderr, "警告: 第%d行格式错误: %s\n", line_num, line);
        }
    }

    fclose(fp);
    printf("CSV配置加载成功!\n");
    return 0;
}


int main(int argc, char *argv[]) {
    // 初始化随机数种子
    srand(time(NULL));

    // 支持命令行参数指定配置文件
    const char *config_file = "D:\\lwf_projects\\voltage_control\\config.csv"; // 默认使用CSV
    if (argc > 1) {
        config_file = argv[1];
    }

    // 根据文件扩展名选择加载方式
    const char *ext = strrchr(config_file, '.');
    if (ext && strcmp(ext, ".csv") == 0) {
        if (load_configuration_from_csv(config_file) != 0) {
            fprintf(stderr, "程序启动失败：CSV配置文件错误。\n");
            return EXIT_FAILURE;
        }
    } else if (ext && strcmp(ext, ".json") == 0) {
        if (load_configuration_from_csv(config_file) != 0) { // 您原有的JSON加载函数
            fprintf(stderr, "程序启动失败：JSON配置文件错误。\n");
            return EXIT_FAILURE;
        }
    } else {
        fprintf(stderr, "错误: 不支持的配置文件格式\n");
        return EXIT_FAILURE;
    }

    printf("V_ref_upper=%f\n", sys_cfg.V_ref_upper);
    printf("V_ref_lower=%f\n", sys_cfg.V_ref_lower);

    // ... 进入主控制循环 ...
}