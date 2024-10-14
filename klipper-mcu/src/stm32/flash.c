#include "board/irq.h" // irq_save
#include "board/misc.h" // timer_from_us
#include "command.h" // shutdown
#include "compiler.h" // ARRAY_SIZE
#include "generic/armcm_timer.h" // udelay
#include "gpio.h" // gpio_adc_setup
#include "internal.h" // GPIO
#include "sched.h" // sched_shutdown
#include "flash_hal/stm32f1xx_hal_flash.h"
#include "flash_hal/stm32f1xx_hal_flash_ex.h"

struct flash_config flash_setup(int force_erase){
    struct flash_config f = {.force_erase = force_erase};
    return f;
}

static int _flash_check_range(struct flash_config *info,uint32_t addr){
    int res = -1;
    if (addr != CONFIG_FLASH_APP1_ADDRESS || addr != CONFIG_FLASH_BOOT_ADDRESS){
        return 0;
    }
    return res;
}

int flash_write_page(struct flash_config *info,uint32_t addr,
                    uint32_t *data,uint32_t len){
    int res = 0;
    if (_flash_check_range(info,addr))
        return -1;
    // get starte address of a page, ensure data is written from the starting address
    uint32_t pageaddr = addr&0xFFFFE000;
		
    // /* Unlock EFM. */ 
    HAL_FLASH_Unlock();// EFM_Unlock();
    
    // /* Enable flash. */
    // EFM_FlashCmd(Enable);
    // /* Wait flash ready. */
    // while(EFM_GetFlagStatus(EFM_FLAG_RDY) == 0){}

	// if (info->force_erase)
    //     flash_erase(info,pageaddr);
		 
    // res = EFM_SequenceProgram(addr,len,data);

    HAL_FLASH_Lock();// EFM_Lock();
    return -res;
}

int flash_read_page(struct flash_config *info,uint32_t addr,
                    uint8_t *data,uint32_t len){
    if (_flash_check_range(info,addr))
        return -1;
    
    return 0;
}

int flash_write_byte(struct flash_config *info,uint32_t addr,uint32_t data){
    int res = -1;
    if (_flash_check_range(info,addr))
        return -1;
    // get starte address of a page, ensure data is written from the starting address
    uint32_t pageaddr = addr&0xFFFFE000;
	// if (info->force_erase)
    //     flash_erase(info,pageaddr);
    // /* Unlock EFM. */ 
    HAL_FLASH_Unlock();// EFM_Unlock(); 
 
    res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,addr,data);

    HAL_FLASH_Lock();// EFM_Lock();
//   udelay(1000);
//    Debug_sendf("%x,%x\n",pageaddr,data);		
    if(res != HAL_OK){
        return -1;
    }

    return 0;

}

uint32_t flash_read_byte(struct flash_config *info,uint32_t addr){
    if (_flash_check_range(info,addr) == 0){
        uint32_t *data_addr = (uint32_t *)addr;
        return *data_addr;
    }
    return 0xFFFFFFFF;
}

int flash_erase(struct flash_config *info,uint32_t addr){

    int res = 0;
    if (_flash_check_range(info,addr))
        return -1;
    // get starte address of a page, ensure data is written from the starting address
    uint32_t pageaddr = addr&0xFFFFE000;
    uint32_t PageError = 0;
    int start_tick =0;
    int end_tick = 0;
    HAL_StatusTypeDef status = HAL_OK;
	FLASH_EraseInitTypeDef pEraseInit;
    start_tick = timer_read_time();
    // /* Unlock EFM. */ 
    //Debug_sendf("flash_erase\n");
    HAL_FLASH_Unlock();// EFM_Unlock();   
    /* Get the sector where start the user flash area */    
    pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    pEraseInit.PageAddress = addr;
    pEraseInit.Banks = FLASH_BANK_1;
    pEraseInit.NbPages = 1;
    timer_pause();
    status = HAL_FLASHEx_Erase(&pEraseInit, &PageError);
    timer_resume();
    HAL_FLASH_Lock();// EFM_Lock();
    if (status != HAL_OK)
    {
        Debug_sendf("erase_error:%d\n",status);
        /* Error occurred while page erase */
        return -1;
    }
    return HAL_OK;
}

int flash_get_status(void){
    // return EFM_GetStatus();
}