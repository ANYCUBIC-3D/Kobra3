#include <string.h> // memcpy
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "board/gpio.h"
#include "autoconf.h"
#include "ota_data.h"
#include "string.h"

#define OTA_SEND_MAX_BYTE   48
#define OTA_FLASH_SECTOR    2048

struct ota_cmd{
    struct timer timer;
    uint8_t status;
    struct flash_config config;
    struct ota_data ota_data;
    uint16_t offset;
    uint32_t flash_offset;
};

enum {
    OTA_START = 1<<0,OTA_ERASEING = 1<<1,OTA_ERASED = 1<<2, 
    OTA_WRITING = 1<<3,OTA_WRITED = 1<<4,OTA_FINISH = 1<<5,
    OTA_ERROR = 1<<6,OTA_RESERVE = 1<<7,
};

enum{
    EC_NONE = 0,
    EC_OD_CHECK_ERR = 20,
    EC_OD_WRITE_ERR,
    EC_WRITE_ERR,
    EC_ERASE_ERR,
    EC_CRC_ERR,
};
static struct task_wake ota_wake;

static uint_fast8_t
ota_event(struct timer *timer)
{
    //Debug_sendf("ota_event\n");
    sched_wake_task(&ota_wake);
    return SF_DONE;
}

static void
ota_status(struct ota_cmd *ota, uint_fast8_t oid,uint_fast8_t status,uint_fast8_t err_code)
{
    ota->status = status;
    sendf("ota_status oid=%c offset=%hu status=%c err_code=%c",
        oid,ota->offset, ota->status,err_code);
}

static void
ota_reschedule_timer(struct ota_cmd *ota,uint32_t ticks)
{
    irq_disable();
    ota->timer.waketime = timer_read_time() + ticks;//timer_from_us(1500);
    sched_add_timer(&ota->timer);
    irq_enable();
}

void
command_config_ota(uint32_t *args)
{
    struct ota_cmd *ota = oid_alloc(args[0], command_config_ota, sizeof(*ota));
    struct ota_data tmp = ota_data_alloc();
    if(ota_data_check(&tmp) == 0){
        ota->ota_data = tmp;
    }else{
        ota_status(ota,args[0],OTA_ERROR,EC_OD_CHECK_ERR);
    }
    ota->timer.func = ota_event;
    //Debug_sendf("command_config_ota\n");
}
DECL_COMMAND(command_config_ota, "config_ota oid=%c");

void
command_query_ota_status(uint32_t *args)
{
    struct ota_cmd *ota = oid_lookup(args[0], command_config_ota);
    ota_status(ota,args[0],ota->status,EC_NONE);
}
DECL_COMMAND(command_query_ota_status, "query_ota_status oid=%c");

void
command_query_ota_local_info(uint32_t *args)
{
    struct ota_cmd *ota = oid_lookup(args[0], command_config_ota);
    sendf("ota_local_info oid=%c flag=%c crc32=%u version_major=%c version_minor=%c version_patch=%c",
        args[0],
        ota->ota_data.app_flag,
        ota->ota_data.crc32,
        ota->ota_data.version_major,
        ota->ota_data.version_minor,
        ota->ota_data.version_patch);
}
DECL_COMMAND(command_query_ota_local_info, "query_ota_local_info oid=%c");

void 
ota_request_report(struct ota_cmd *ota,uint8_t oid)
{
    ota->status = OTA_WRITING;
    sendf("ota_transfer oid=%c offset=%hu count=%c",
            oid,ota->offset, OTA_SEND_MAX_BYTE);
}

void
command_ota_start(uint32_t *args)
{   
    struct ota_cmd *ota = oid_lookup(args[0], command_config_ota);
    ota->status = OTA_START;
    ota->offset = 0;
    ota->ota_data.app_flag = 0;
    ota->ota_data.crc32 = args[1];
    ota->ota_data.version_major = args[2];
    ota->ota_data.version_minor = args[3];
    ota->ota_data.version_patch = args[4];
    ota_status(ota,args[0],OTA_START,EC_NONE);
}
DECL_COMMAND(command_ota_start, "ota_start oid=%c crc32=%u version_major=%c version_minor=%c version_patch=%c" );

void
command_ota_erase(uint32_t *args)
{   
    struct ota_cmd *ota = oid_lookup(args[0], command_config_ota);
    uint8_t is_transfer = args[2];
    //Debug_sendf("command_ota_erase:%d,offset:%x\n",is_transfer,args[1]);
    if(is_transfer){
        ota_request_report(ota,args[0]);
    }else{
        ota_status(ota,args[0],OTA_ERASEING,EC_NONE);
        ota->flash_offset = args[1];
        udelay(10000);
        sched_wake_task(&ota_wake);
        //ota_reschedule_timer(ota,timer_from_us(50000));
    }
}
DECL_COMMAND(command_ota_erase, "ota_erase oid=%c offset=%u is_transfer=%c" );

uint8_t ascii_to_hex(uint8_t ascii){
    if(ascii >= 0x61 && ascii <= 0x66) {
        return ascii - 0x57;
    }else if(ascii >= 0x41 && ascii <= 0x46){
        return ascii - 0x37;
    }else if(ascii >= 0x30 && ascii <= 0x39){
        return ascii - 0x30;
    }
    return 0x0F;
}

void
command_ota_transfer_response(uint32_t *args)
{
    struct ota_cmd *ota = oid_lookup(args[0], command_config_ota);
    uint8_t data_len = args[2];
    uint8_t *data = command_decode_ptr(args[3]);
    int res = 0;
    if(ota->status == OTA_WRITING){
        irq_disable();
        if(data_len > 0){
            for(int i = 0;i < data_len;i += 4){
                uint32_t dat = 0x12345678;
                if(i % 4 == 0 && i == data_len - 1 && data[i] == '\n'){ //最后一帧数据且最后一个数据是换行符
                    break;
                }else{
                    dat = data[i + 3] << 24 | data[i + 2] << 16 | data[i + 1] << 8 | data[i];
                }
                uint32_t addr = ota_data_get_app_addr(&ota->ota_data) + i + ota->offset;
                res = flash_write_byte(&ota->config,addr,dat);
                if(res){
                    res = flash_write_byte(&ota->config,addr,dat);
                    if(res){
                        ota_status(ota,args[0],OTA_ERROR,EC_WRITE_ERR);
                        return;
                    }
                }
            }
            //Debug_sendf("len:%d\n",data_len);
            ota->offset += data_len;
            ota_request_report(ota,args[0]);
            irq_enable();
        }else{ 
            //crc check
            irq_disable();
            //Debug_sendf("len:%d\n",data_len);
            uint32_t crc32 = 0;
            if(ota->offset > 0){
                uint32_t * app_addr = (uint32_t *)(ota_data_get_app_addr(&ota->ota_data) + ota->offset - 1); 
                uint8_t crc[8] = { 
                    *(app_addr - 2) & 0xff, (*(app_addr - 2) >> 8) & 0xff, (*(app_addr - 2) >> 16)& 0xff, (*(app_addr - 2) >> 24)& 0xff,
                    *(app_addr - 1) & 0xff, (*(app_addr - 1) >> 8) & 0xff, (*(app_addr - 1) >> 16)& 0xff, (*(app_addr - 1) >> 24)& 0xff,
                    };

                crc32 = (ascii_to_hex(crc[0]) << 4 | ascii_to_hex(crc[1])) << 24 | 
                        (ascii_to_hex(crc[2]) << 4 | ascii_to_hex(crc[3])) << 16 | 
                        (ascii_to_hex(crc[4]) << 4 | ascii_to_hex(crc[5])) << 8  | 
                        (ascii_to_hex(crc[6]) << 4 | ascii_to_hex(crc[7])); 
            }
            sendf("ota_crc_check oid=%c flag=%c v1=%c v2=%c v3=%c crc_fw=%u crc_calu=%u offset=%u",
                    args[0],ota->ota_data.app_flag,ota->ota_data.version_major,ota->ota_data.version_minor,
                    ota->ota_data.version_patch,crc32,ota->ota_data.crc32,ota->ota_data.offset);
            Debug_sendf("rx finshed\n");        
            if(crc32 > 0 && ota->ota_data.crc32 == crc32){
                ota->ota_data.app_flag = 1;
                ota->ota_data.offset = ota->offset;
                int res = ota_data_save(&ota->ota_data,1);
                uint32_t* flag = (uint32_t*)CONFIG_FLASH_OTA_DATA_ADDRESS;
                uint32_t* crc = (uint32_t*)(CONFIG_FLASH_OTA_DATA_ADDRESS + 4);
                uint32_t* offset = (uint32_t*)(CONFIG_FLASH_OTA_DATA_ADDRESS + 8);
                sendf("ota_save_otadata oid=%c flag=%c v1=%c v2=%c v3=%c crc_fw=%u crc_calu=%u offset=%u",
                    args[0],(*flag >> 24)&0xff,(*flag >> 16)&0xff,(*flag >> 8)&0xff,(*flag)&0xff,*crc,crc32,*offset);

                if (res == 0){
                    ota_status(ota,args[0],OTA_FINISH,EC_NONE); 
                }else{
                    ota_status(ota,args[0],OTA_ERROR,EC_OD_WRITE_ERR);
                }
                       
            }else{
                ota_status(ota,args[0],OTA_ERROR,EC_CRC_ERR);
            }
            irq_enable();
        }
    }
}
DECL_COMMAND(command_ota_transfer_response, "ota_transfer_response oid=%c offset=%hu data=%*s");


void
ota_task(void)
{
    if (!sched_check_wake(&ota_wake))
        return;
    uint8_t oid;
    struct ota_cmd *ota;
    int res = 0;
    uint32_t app_addr = 0;
    irq_disable();
    foreach_oid(oid, ota, command_config_ota) {
        app_addr = ota_data_get_app_addr(&ota->ota_data);
        uint_fast8_t status = ota->status;
        if(status == OTA_ERASEING){
            //res = flash_erase(&ota->config,app_addr + ota->flash_offset * OTA_FLASH_SECTOR);
            //Debug_sendf("%x,%d\n",app_addr+ota->flash_offset * OTA_FLASH_SECTOR,res);
            if(res){
                ota_status(ota,oid,OTA_ERROR,EC_ERASE_ERR);
                return;
            }
            ota_status(ota,oid,OTA_ERASED,EC_NONE);
            //sched_del_timer(&ota->timer);           
        }
    }
    irq_enable();
}
DECL_TASK(ota_task);