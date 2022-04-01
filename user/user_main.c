#include "osapi.h"
#include "user_interface.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"

#include "user_config.h"
#include "user_main.h"

const int32_t pin_name[GPIOMAX] = {
    PERIPHS_IO_MUX_GPIO0_U,     /* 0 -  GPIO0   */
    PERIPHS_IO_MUX_U0TXD_U,     /* 1 -  UART    */
    PERIPHS_IO_MUX_GPIO2_U,     /* 2 -  GPIO2   */
    PERIPHS_IO_MUX_U0RXD_U,     /* 3 -  UART    */
    PERIPHS_IO_MUX_GPIO4_U,     /* 4 -  GPIO4   */
    PERIPHS_IO_MUX_GPIO5_U,     /* 5 -  GPIO5   */
    -1,                         /* 6            */
    -1,                         /* 7            */
    -1,                         /* 8            */
    PERIPHS_IO_MUX_SD_DATA2_U,  /* 9 -  GPIO9   */
    PERIPHS_IO_MUX_SD_DATA3_U,  /* 10 - GPIO10  */
    -1,                         /* 11           */
    PERIPHS_IO_MUX_MTDI_U,      /* 12 - GPIO12  */
    PERIPHS_IO_MUX_MTCK_U,      /* 13 - GPIO13  */
    PERIPHS_IO_MUX_MTMS_U,      /* 14 - GPIO14  */
    PERIPHS_IO_MUX_MTDO_U,      /* 15 - GPIO15  */
    PAD_XPD_DCDC_CONF           /* 16 - GPIO16  */
};

const int32_t func[GPIOMAX] = {
    FUNC_GPIO0,
    FUNC_U0TXD,
    FUNC_GPIO2,
    FUNC_GPIO3,
    FUNC_GPIO4,
    FUNC_GPIO5,
    -1,
    -1,
    -1,
    FUNC_GPIO9,
    FUNC_GPIO10,
    -1,
    FUNC_GPIO12,
    FUNC_GPIO13,
    FUNC_GPIO14,
    FUNC_GPIO15,
    0
};

#if ((SPI_FLASH_SIZE_MAP == 0) || (SPI_FLASH_SIZE_MAP == 1))
#error "The flash map is not supported"
#elif (SPI_FLASH_SIZE_MAP == 2)
#define SYSTEM_PARTITION_OTA_SIZE                           0x6A000
#define SYSTEM_PARTITION_OTA_2_ADDR                         0x81000
#define SYSTEM_PARTITION_RF_CAL_ADDR                        0xfb000
#define SYSTEM_PARTITION_PHY_DATA_ADDR                      0xfc000
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR              0xfd000
#define SYSTEM_PARTITION_CUSTOMER_PRIV_PARAM_ADDR           0x7c000
#elif (SPI_FLASH_SIZE_MAP == 3)
#define SYSTEM_PARTITION_OTA_SIZE                           0x6A000
#define SYSTEM_PARTITION_OTA_2_ADDR                         0x81000
#define SYSTEM_PARTITION_RF_CAL_ADDR                        0x1fb000
#define SYSTEM_PARTITION_PHY_DATA_ADDR                      0x1fc000
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR              0x1fd000
#define SYSTEM_PARTITION_CUSTOMER_PRIV_PARAM_ADDR           0x7c000
#elif (SPI_FLASH_SIZE_MAP == 4)
#define SYSTEM_PARTITION_OTA_SIZE                           0x6A000
#define SYSTEM_PARTITION_OTA_2_ADDR                         0x81000
#define SYSTEM_PARTITION_RF_CAL_ADDR                        0x3fb000
#define SYSTEM_PARTITION_PHY_DATA_ADDR                      0x3fc000
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR              0x3fd000
#define SYSTEM_PARTITION_CUSTOMER_PRIV_PARAM_ADDR           0x7c000
#elif (SPI_FLASH_SIZE_MAP == 5)
#define SYSTEM_PARTITION_OTA_SIZE                           0x6A000
#define SYSTEM_PARTITION_OTA_2_ADDR                         0x101000
#define SYSTEM_PARTITION_RF_CAL_ADDR                        0x1fb000
#define SYSTEM_PARTITION_PHY_DATA_ADDR                      0x1fc000
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR              0x1fd000
#define SYSTEM_PARTITION_CUSTOMER_PRIV_PARAM_ADDR           0xfc000
#elif (SPI_FLASH_SIZE_MAP == 6)
#define SYSTEM_PARTITION_OTA_SIZE                           0x6A000
#define SYSTEM_PARTITION_OTA_2_ADDR                         0x101000
#define SYSTEM_PARTITION_RF_CAL_ADDR                        0x3fb000
#define SYSTEM_PARTITION_PHY_DATA_ADDR                      0x3fc000
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR              0x3fd000
#define SYSTEM_PARTITION_CUSTOMER_PRIV_PARAM_ADDR           0xfc000
#else
#error "The flash map is not supported"
#endif

#define SYSTEM_PARTITION_CUSTOMER_PRIV_PARAM                SYSTEM_PARTITION_CUSTOMER_BEGIN

uint32 priv_param_start_sec;


static const partition_item_t at_partition_table[] = {
    { SYSTEM_PARTITION_BOOTLOADER,          0x0,                                        0x1000},
    { SYSTEM_PARTITION_OTA_1,               0x1000,                                     SYSTEM_PARTITION_OTA_SIZE},
    { SYSTEM_PARTITION_OTA_2,               SYSTEM_PARTITION_OTA_2_ADDR,                SYSTEM_PARTITION_OTA_SIZE},
    { SYSTEM_PARTITION_RF_CAL,              SYSTEM_PARTITION_RF_CAL_ADDR,               0x1000},
    { SYSTEM_PARTITION_PHY_DATA,            SYSTEM_PARTITION_PHY_DATA_ADDR,             0x1000},
    { SYSTEM_PARTITION_SYSTEM_PARAMETER,    SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR,     0x3000},
    { SYSTEM_PARTITION_CUSTOMER_PRIV_PARAM, SYSTEM_PARTITION_CUSTOMER_PRIV_PARAM_ADDR,  0x1000},
};

void ICACHE_FLASH_ATTR user_pre_init(void)
{
    if(!system_partition_table_regist(at_partition_table, sizeof(at_partition_table)/sizeof(at_partition_table[0]),SPI_FLASH_SIZE_MAP)) {
        os_printf("system_partition_table_regist fail\r\n");
        while(1);
    }
}


#define KEY_GPIO GPIO5

static os_timer_t pulse_timer, main_timer, sleep_timer;
static bool p_pressed = false;
static bool p_state = false;
static uint16_t p_bit = 1;
static uint32_t p_count;

static void ICACHE_FLASH_ATTR pulse_timer_cb(void *arg) {

    os_timer_disarm(&pulse_timer);

    if (!GPIO_INPUT_GET(KEY_GPIO)) {
        if (p_bit == 0x1000) {
            p_state = true;
        } else {
            p_bit <<= 1;
        }
    } else {
        if (p_bit == 1) {
            p_state = false;
        } else {
            p_bit >>= 1;
        }
    }
    if (p_state == true) {

        if (p_pressed == false) {
            os_printf("key pressed %u\n", p_count++);
            p_pressed = true;
        }
    } else {
        p_pressed = false;
    }

    os_timer_arm(&pulse_timer, 10, false);
}


static void ICACHE_FLASH_ATTR wakeup_from_motion(void) {
    wifi_fpm_close();
    os_printf("Wake up from sleep.\n");
}

static void ICACHE_FLASH_ATTR sleep_start() {
    os_printf("Light sleep start\n");
    wifi_station_disconnect();
    wifi_set_opmode_current(NULL_MODE);
//  wifi_set_opmode(NULL_MODE);
    os_delay_us(50000);
    wifi_fpm_set_sleep_type(LIGHT_SLEEP_T); //light sleep mode
    wifi_enable_gpio_wakeup(GPIO_ID_PIN(KEY_GPIO), GPIO_PIN_INTR_LOLEVEL);
    wifi_fpm_open();
    wifi_fpm_set_wakeup_cb(wakeup_from_motion); //wakeup callback
    wifi_fpm_do_sleep(0xFFFFFFF);
}



void ICACHE_FLASH_ATTR user_init(void) {

    uart_init(BIT_RATE_115200, BIT_RATE_115200);
    os_delay_us(65535);

    wifi_set_opmode(NULL_MODE);

    os_printf("\n\nSDK version: %s\n", system_get_sdk_version());


    PIN_FUNC_SELECT(pin_name[KEY_GPIO], func[KEY_GPIO]);
    GPIO_DIS_OUTPUT(pin_name[KEY_GPIO]);
    PIN_PULLUP_EN(pin_name[KEY_GPIO]);

    p_count = 0;

    os_timer_disarm(&pulse_timer);
    os_timer_setfn(&pulse_timer, (os_timer_func_t *)&pulse_timer_cb, NULL);
    os_timer_arm(&pulse_timer, 10, false);


    os_timer_disarm(&sleep_timer);
    os_timer_setfn(&sleep_timer, (os_timer_func_t *)&sleep_start, NULL);
    os_timer_arm(&sleep_timer, 10000, true);

//    i2c_master_gpio_init();
}

