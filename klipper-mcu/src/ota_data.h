#ifndef __OTA_DATA_H
#define __OTA_DATA_H

#include "sched.h" // DECL_TASK

/*############################################
#             OTA_DATA 64bit                 #                           
#|APP_flag|Version 1|Version 2|Version 3|crc32|
#|0/1     |0-255    |0-255    |0-255    | 0   |
#############################################*/

enum{
    APP_FLAG1 = 0,APP_FLAG2 = 1,APP_FACTORY = 2,OTA_DATA_FLAG = 3,
};

struct ota_data
{
    uint8_t app_flag;
    uint8_t version_major;
    uint8_t version_minor;
    uint8_t version_patch;
    uint32_t crc32;
    uint32_t offset;
    uint32_t reserve;
};

uint32_t ota_data_get_app_addr(struct ota_data *data);
void ota_data_set_app_addr(struct ota_data *data,uint8_t flag);
int8_t ota_data_get_version(struct ota_data *data,const char *version);
int ota_data_check(struct ota_data *data);
struct ota_data ota_data_alloc();
uint32_t ota_data_get_factory_addr(struct ota_data *data);
int ota_data_save(struct ota_data *data,int force_erase);
#endif