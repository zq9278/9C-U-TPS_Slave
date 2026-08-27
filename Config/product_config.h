#ifndef CONFIG_PRODUCT_CONFIG_H
#define CONFIG_PRODUCT_CONFIG_H

/*
 * 全局产品配置
 *
 * 不同产品型号、固件版本和功能差异统一在本文件配置，业务模块不要
 * 再单独定义同类产品开关。
 */

/* 产品标识。 */
#define PRODUCT_MODEL_NAME                              "9C-TPS100-Slave"
#define PRODUCT_VARIANT_ID                              0U

/*
 * 固件版本占位。发布版本确定后在这里统一填写。
 * 当前 0.0.0 表示尚未设置正式发布版本。
 */
#define PRODUCT_FIRMWARE_VERSION_MAJOR                  0U
#define PRODUCT_FIRMWARE_VERSION_MINOR                  0U
#define PRODUCT_FIRMWARE_VERSION_PATCH                  0U
#define PRODUCT_FIRMWARE_VERSION_STRING                 "0.0.0"

/*
 * 产品功能开关：是否允许输出耗材熔断脉冲。
 * 0U：禁用熔断，收到左右熔断命令时不驱动 GPIO。
 * 1U：启用熔断，按 EYE_SHIELD_FUSE_BLOW_PULSE_MS 输出脉冲。
 */
#define PRODUCT_FEATURE_CONSUMABLE_FUSE_BLOW_ENABLE     0U

#endif /* CONFIG_PRODUCT_CONFIG_H */
