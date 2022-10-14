/*
 * bootloader_command_app.h
 *
 *  Created on: 5 Eki 2022
 *      Author: SAMEDBASKİN
 */

#ifndef INC_BOOTLOADER_COMMAND_APP_H_
#define INC_BOOTLOADER_COMMAND_APP_H_
#include "main.h"

#define BL_VER 0x10
#define CRC_FAIL 1
#define CRC_SUCCESS 1

#define  BL_NACK_VALUE 0x7F
#define  BL_ACK_VALUE 0xA5

#define ADDR_VALID 0x00
#define ADDR_INVALID 0x01

#define SRAM1_SIZE 112*1024
#define SRAM1_END (SRAM1_BASE+SRAM1_SIZE)
#define SRAM2_SIZE 16*1024
#define SRAM2_END (SRAM2_BASE+SRAM2_SIZE)

#define INVALID_SECTOR 0x04

void bootloader_get_ver_cmd(uint8_t *bl_rx_data);
void bootloader_get_help(uint8_t *bl_rx_data);
void bootloader_get_rdp_cmd(uint8_t *bl_rx_data);
void bootloader_go_to_addr(uint8_t *bl_rx_data);
void bootloader_flash_erase(uint8_t *bl_rx_data);
void bootloader_mem_write_cmd(uint8_t * bl_rx_data);
void bootloader_enable_rw_protect(uint8_t * bl_rx_data);
void bootloader_disable_rw_protect(uint8_t * bl_rx_data);



uint8_t configure_flash_sector_rw_protection(uint8_t sectorDetails,uint8_t protesctionModes,uint8_t enableOrDisable);
uint8_t execute_memory_write(uint8_t * buffer,uint32_t memAddress,uint32_t len);
uint8_t  bootloader_verifiy_crc(uint8_t * buffer, uint32_t len,uint32_t crc);
void bootloader_send_ack(uint8_t followLenght);
void bootloader_send_nack();
void bootloader_uart_write_data(uint8_t *dataBuff, uint8_t len);
uint8_t bootloader_verify_address(uint32_t goAddress);
uint8_t execute_flash_erase(uint8_t sectorNumber,uint8_t numberOfSector);
#endif /* INC_BOOTLOADER_COMMAND_APP_H_ */
