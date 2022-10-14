/*
 * bootloader_command_code.h
 *
 *  Created on: 4 Eki 2022
 *      Author: SAMEDBASKİN
 */

#ifndef INC_BOOTLOADER_COMMAND_CODE_H_
#define INC_BOOTLOADER_COMMAND_CODE_H_

#define BL_GET_VER 				 0x51
#define BL_GET_HELP 			 0x52
#define BL_GET_CID				 0x53
#define BL_GET_RDP_STATUS		 0x54
#define BL_GO_TO_ADDR			 0x55
#define BL_FLASH_ERASE			 0x56
#define BL_MEM_WRITE 			 0x57
#define BL_EN_RW_PROTECT	     0x58
#define BL_MEM_READ				 0x59
#define BL_READ_SECTOR_P_STATUS	 0x5A
#define BL_OTP_READ 			 0x5B
#define BL_DIS_R_W_PROTECT 	     0x5C

uint8_t supported_command[]={
	BL_GET_VER,BL_GET_HELP,BL_GET_CID,	BL_GET_RDP_STATUS,BL_GO_TO_ADDR,BL_FLASH_ERASE,
	BL_MEM_WRITE,BL_EN_RW_PROTECT,BL_MEM_READ,BL_READ_SECTOR_P_STATUS,BL_OTP_READ,BL_DIS_R_W_PROTECT
};
#endif /* INC_BOOTLOADER_COMMAND_CODE_H_ */
