#include <string.h> // memcpy
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "stm32/gpio.h"
#include "autoconf.h"
#include "ota_data.h"

#define APP1_ADDRESS CONFIG_FLASH_APP1_ADDRESS
#define APP2_ADDRESS CONFIG_FLASH_APP2_ADDRESS
#define APP3_ADDRESS CONFIG_FLASH_FACTORY_ADDRESS
#define BOOT_ADDRESS CONFIG_FLASH_BOOT_ADDRESS 
#define OTA_DATA_ADDRESS CONFIG_FLASH_OTA_DATA_ADDRESS


uint32_t get_app_addr(void){
    return APP2_ADDRESS;
}

// static uint8_t number_to_char(uint8_t num,char *str){
//     int count = 0;
//     while (num != 0) {
//         int remainder = num % 10;
//         num /= 10;
//         str[count++] = remainder + '0';
//     }
//     return count;
// }

uint32_t ota_data_get_app_addr(struct ota_data *data){
    return get_app_addr();
}

void ota_data_set_app_addr(struct ota_data *data,uint8_t flag){
    if(flag > APP_FACTORY){
        data->app_flag = 0;
        return;
    }
    data->app_flag = flag;
}

static int _atoi(const char *str) {
    if(str == NULL)
    {
        return -1;
    }
    int result = 0;
    int sign = 1;
    int i = 0;

    if(str[0] == '-') {
        sign = -1;
        i++; 
    }

    for(; str[i] != '\0'; ++i) {
        if(str[i] < '0' || str[i] > '9') {
        break;
        }
        result = result*10 + str[i] - '0';
    }
  
    return sign * result;
}

int string_number_check(const char *number){

    const char *v = number;
    for (size_t i = 0; i < strlen(v); i++)
    {
        if(v[i] < '0' || v[i] > '9'){
            return -1;
        }
    }
    return 0;
}

int8_t ota_data_get_version(struct ota_data *data,const char *version){
    
    char *start = (char *)version;
    char *end = NULL;

    char tmp[8] = {0};
    uint8_t len = 0;

    //检查并跳过‘v’
    if(version[0] != 'v'){
        return -1;
    }

    start ++;
    end = strstr(start,".");
    if (end == NULL){
        return -1;
    }
    len = end - start;
    strncpy(tmp,start,len);
    tmp[len] = '\0';

    if(string_number_check(tmp)){
        return -1;
    }
    data->version_major = (uint8_t)_atoi(tmp);

    //检查并跳过第一个‘.’
    start = end + 1;
    end = strstr(start,".");
    if (end == NULL){
        return -1;
    }
    len = end - start;
    strncpy(tmp,start,len);
    tmp[len] = '\0';
 
    if(string_number_check(tmp)){
        return -1;
    }
    data->version_minor = (uint8_t)_atoi(tmp);
    
    //检查并跳过第二个‘.’
    start = end + 1;
    strcpy(tmp,start);
    if(string_number_check(tmp)){
        return -1;
    }
    data->version_patch = (uint8_t)_atoi(tmp);
    return 0;
}

int ota_data_check(struct ota_data *data){
    
    if(data->app_flag <= 0 && data->app_flag > APP_FACTORY) {
        return -1;
    }
    
    if(data->crc32 == 0) {
        return -1;
    }
    return 0;
}

uint32_t ota_data_get_factory_addr(struct ota_data *data){
    
    return APP3_ADDRESS;
}

int ota_data_save(struct ota_data *data,int force_erase){
    // uint32_t flag = data->app_flag << 24 | data->version_major << 16 | data->version_minor << 8 |data->version_patch;
    struct flash_config cfg = {0};
    int res = 0;

    if(force_erase){
        flash_erase(&cfg,OTA_DATA_ADDRESS);
    }

    uint32_t tmp_data[] = {
        data->app_flag << 24 | data->version_major << 16 | data->version_minor << 8 |data->version_patch,
        data->crc32,
        data->offset,
        0xFFFFFFFF
    };
    
    for (size_t i = 0; i < sizeof(tmp_data) / sizeof(tmp_data[0]); i++)
    {
        res = flash_write_byte(&cfg,OTA_DATA_ADDRESS + i * 4,tmp_data[i]);
        if(res != 0){
            res = flash_write_byte(&cfg,OTA_DATA_ADDRESS + i * 4,tmp_data[i]);
            if(res != 0){
                return -1;
            }
        }
    }
    return res;
}

struct ota_data ota_data_alloc(void){
    uint32_t* flag = (uint32_t*)OTA_DATA_ADDRESS;
    uint32_t* crc = (uint32_t*)(OTA_DATA_ADDRESS + 4);
    // default ota_data value
    if (((*flag) & 0xff) == 0xff){
        struct flash_config cfg = {0};
        flash_erase(&cfg,OTA_DATA_ADDRESS);

        struct ota_data tmp = {
            .app_flag = 0,
            .version_major = 0,
            .version_minor = 0,
            .version_patch = 1,
            .crc32 = 0x12345678
        };

        ota_data_get_version(&tmp,CONFIG_FIRMWARE_VERSION);
        ota_data_save(&tmp,1);
        return tmp;
    }
    
    struct ota_data tmp = {
        .app_flag = (*flag >> 24) & 0xff,
        .version_major = (*flag >> 16) & 0xff,
        .version_minor = (*flag >> 8) & 0xff,
        .version_patch = *flag & 0xff,
        .crc32 = *crc
    };
    return tmp;
}

void
ota_data_init(void)
{
    ota_data_alloc();
}
DECL_INIT(ota_data_init);