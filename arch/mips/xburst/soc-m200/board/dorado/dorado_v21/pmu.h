#ifndef __PMU_H__
#define __PMU_H__
#ifdef CONFIG_REGULATOR_RICOH619
/* ****************************PMU DC/LDO NAME******************************* */
#define DC1_NAME "cpu_core"
#define DC2_NAME "cpu_vmema"
#define DC3_NAME "cpu_mem12"
#define DC4_NAME "cpu_vddio"
#define LDO1_NAME "wifi_vddio_18"
#define LDO2_NAME "cpu_vdll1"
#define LDO3_NAME "cpu_avdd"
#define LDO4_NAME "cpu_vddio2"
#define LDO5_NAME "cpu_2.5v"
#define LDO6_NAME "v33"
#define LDO7_NAME "vcc_sensor1v8"
#define LDO8_NAME "vcc_sensor3v3"
#define LDO9_NAME "lcd_1.8v"
#define LDO10_NAME "ldo10"
#define LDORTC1_NAME "rtc_1.8V"
#define LDORTC2_NAME "rtc_1.1V"
/* ****************************PMU DC/LDO NAME END*************************** */

/* ****************************PMU DC/LDO DEFAULT V************************** */
#define DC1_INIT_UV     1100
#define DC2_INIT_UV     1200
#define DC3_INIT_UV     1200
#ifdef CONFIG_JZ_EPD_V12
#define DC4_INIT_UV     3300
#else
#define DC4_INIT_UV     1800
#endif
#define LDO1_INIT_UV    1800
#define LDO2_INIT_UV    3300
#define LDO3_INIT_UV    2800
#define LDO4_INIT_UV    1800
#define LDO5_INIT_UV    2500
#define LDO6_INIT_UV    3300
#define LDO7_INIT_UV    1800
#define LDO8_INIT_UV    3300
#define LDO9_INIT_UV    1800
#define LDO10_INIT_UV   1800
#define LDORTC1_INIT_UV 1800
#define LDORTC2_INIT_UV 1100
/* ****************************PMU DC/LDO DEFAULT V END********************** */
/* ****************************PMU LDO INIT ENABLE*************************** */
#define LDO1_INIT_ENABLE    0
#define LDO2_INIT_ENABLE    1
#define LDO3_INIT_ENABLE    0
#define LDO4_INIT_ENABLE    1
#define LDO5_INIT_ENABLE    1
#define LDO6_INIT_ENABLE    1
#define LDO7_INIT_ENABLE    1
#define LDO8_INIT_ENABLE    0
#define LDO9_INIT_ENABLE    0
#define LDO10_INIT_ENABLE   0
#define LDORTC1_INIT_ENABLE 1
#define LDORTC2_INIT_ENABLE 1
/* ****************************PMU DC/LDO DEFAULT V END********************** */
#endif	/* CONFIG_REGULATOR_RICOH619 */
#endif
