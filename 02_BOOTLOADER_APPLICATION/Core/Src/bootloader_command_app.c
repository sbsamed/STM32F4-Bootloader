/*
 * bootloader_command_app.c
 *
 *  Created on: 5 Eki 2022
 *      Author: SAMEDBASKİN
 */

#include "bootloader_command_app.h"
#include "string.h"
extern uint8_t supported_command[];

uint8_t get_flash_rdp_levell(void);

void bootloader_get_ver_cmd(uint8_t *bl_rx_data)
 {

	uint8_t bl_version;
	printMessage("BL_GET_VER_CMD is run\n");

	uint32_t command_packet_lenght=1+bl_rx_data[0];

	uint32_t host_crc= *((uint32_t*)bl_rx_data+command_packet_lenght-4);
	//crc control
	if(bootloader_verifiy_crc(bl_rx_data[0], command_packet_lenght-4, host_crc)){
		printMessage("BL_DEBUG_MESSAGE: Checksum succes \n");
	    bootloader_send_ack(1);
	}else{

		bootloader_send_nack();
	}

}
void bootloader_get_help(uint8_t *bl_rx_data){

	printMessage("BL_GET_HELP is run\n");

	uint32_t command_packet_lenght=bl_rx_data[0]+1;

	uint32_t host_crc= *((uint32_t*)bl_rx_data+command_packet_lenght-4);

	//crc control

	if(bootloader_verifiy_crc(bl_rx_data[0], command_packet_lenght-4, host_crc)){
		printMessage("BL_DEBUG_MESSAGE: Checksum succes \n");
	    bootloader_send_ack( strlen(supported_command));
		bootloader_uart_write_data(supported_command, strlen(supported_command));

	}else{

		bootloader_send_nack();
	}
}
void bootloader_get_cid(uint8_t *bl_rx_data){

	uint16_t cID=0;

	printMessage("BL_GET_CID is run\n");
	uint32_t command_packet_lenght=bl_rx_data[0]+1;
	uint32_t host_crc= *((uint32_t*)bl_rx_data+command_packet_lenght-4);
	//crc control
	if(bootloader_verifiy_crc(bl_rx_data[0], command_packet_lenght-4, host_crc)){
		printMessage("BL_DEBUG_MESSAGE: Checksum succes \n");
	    bootloader_send_ack(strlen(2));
	    cID= (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;
	    printMessage("BL_DEBUG_MESSAGE CHIP ID : %d",cID);
	    bootloader_uart_write_data((uint8_t*)cID, 2);

	}else{

		bootloader_send_nack();
	}
}

void bootloader_get_rdp_cmd(uint8_t *bl_rx_data){

	printMessage("BL_GET_RDP is run\n");
	uint8_t rdpLevel=0;
	uint32_t command_packet_lenght=bl_rx_data[0]+1;

		uint32_t host_crc= *((uint32_t*)bl_rx_data+command_packet_lenght-4);

		//crc control

		if(bootloader_verifiy_crc(bl_rx_data[0], command_packet_lenght-4, host_crc)){
			printMessage("BL_DEBUG_MESSAGE: Checksum succes \n");
		    bootloader_send_ack(1);
		    rdpLevel=get_flash_rdp_levell();
		    printMessage("RDP Level = %d\n",rdpLevel );
		   bootloader_uart_read_data(&rdpLevel,1);

		}else{

			bootloader_send_nack();
		}
}

void bootloader_go_to_addr(uint8_t *bl_rx_data){

	printMessage("BL_GO_TO_ADDRESS is run\n");

	uint32_t go_to_address=0;
	uint8_t add_invalid= ADDR_INVALID;
	uint8_t add_valid= ADDR_VALID;

	uint32_t command_packet_lenght=bl_rx_data[0]+1;

		uint32_t host_crc= *((uint32_t*)bl_rx_data+command_packet_lenght-4);

		//crc control

		if(bootloader_verifiy_crc(bl_rx_data[0], command_packet_lenght-4, host_crc)){
			printMessage("BL_DEBUG_MESSAGE: Checksum succes \n");
		    bootloader_send_ack(1);
		    go_to_address = ((uint32_t*)&bl_rx_data[2]);
		    printMessage("GO TO ADDRESS :%d",go_to_address);

		    if(bootloader_verify_address(go_to_address)== ADDR_VALID){
		    	bootloader_uart_write_data(add_valid, 1);
		    	go_to_address+=1; //t bit is 1;

		    	void (*lets_go_to_address)(void)=(void*)go_to_address;
		    	printMessage("JUMPING GO TO ADDRESS");
		    	lets_go_to_address();

		    }else{
		    	bootloader_uart_write_data(&add_invalid, 1);
		    	printMessage("GO ADDRESS INVALID ERROR");

		    }
		}
		else{
			bootloader_send_nack();
		}

}

void bootloader_flash_erase(uint8_t *bl_rx_data){

	uint8_t eraseStatus = 0;

	printMessage("BL_bootloader_flash_erase is run\n");
	uint32_t command_packet_lenght=bl_rx_data[0]+1;
	uint32_t host_crc= *((uint32_t*)bl_rx_data+command_packet_lenght-4);
	//crc control
	if(bootloader_verifiy_crc(bl_rx_data[0], command_packet_lenght-4, host_crc)){

		printMessage("BL_DEBUG_MESSAGE: Checksum succes \n");
		bootloader_send_ack(1);
		eraseStatus=execute_flash_erase(bl_rx_data[2],bl_rx_data[3]);
		printMessage("BL_DEBUG_MESSAGE: Flash erase status %d \n  \n",eraseStatus);

		}else{
		bootloader_send_nack();
		}
}
void bootloader_mem_write_cmd(uint8_t * bl_rx_data)
 {
	uint8_t addrValid=0;
	uint8_t write_status=0;
	uint8_t checkSum=0,lenght=0;

	lenght=bl_rx_data[0];

	uint8_t payloadLenght=bl_rx_data[6];

	uint32_t memAddress=*((uint32_t*)(&bl_rx_data[2]));

	checkSum=bl_rx_data[lenght];

	printMessage("BL_memn_write_cmd is run\n");

	uint32_t command_packet_lenght=bl_rx_data[0]+1;

	uint32_t host_crc= *((uint32_t*)bl_rx_data+command_packet_lenght-4);

	if(bootloader_verifiy_crc(bl_rx_data[0], command_packet_lenght-4, host_crc)){

		printMessage("BL_DEBUG_MESSAGE: Checksum succes \n");
		bootloader_send_ack(1);

		printMessage("Memory write address %x \n",memAddress);

		if(bootloader_verify_address(memAddress)==ADDR_VALID){
			printMessage("BL_DEBUG_MSG: VALID MEMEORY ADDRESS");

			write_status=execute_memory_write(bl_rx_data[7],memAddress,payloadLenght);
			bootloader_uart_write_data(&write_status, 1);

		}else{
			printMessage("BL_DEBUG_MSG: INVALID MEMEORY ADDRESS");
			bootloader_uart_write_data(&write_status, 1);

		}

	}else{
		bootloader_send_nack();
		}

 }
void bootloader_enable_rw_protect(uint8_t * bl_rx_data){

    uint8_t status=0;

	printMessage("BL_enable_rw_protect is run\n");
	uint32_t command_packet_lenght=bl_rx_data[0]+1;
	uint32_t host_crc= *((uint32_t*)bl_rx_data+command_packet_lenght-4);

	if(bootloader_verifiy_crc(bl_rx_data[0], command_packet_lenght-4, host_crc))
	{

		printMessage("BL_DEBUG_MESSAGE: Checksum succes \n");
		bootloader_send_ack(1);
		status= configure_flash_sector_rw_protection(bl_rx_data[2],bl_rx_data[3],0);
		printMessage("BL_DEBUG_MESSAGE: Flash protect status : %d \n",status);
		bootloader_uart_write_data(&status, 1);
	}
	else{
		bootloader_send_nack();
		}



}
void bootloader_disable_rw_protect(uint8_t * bl_rx_data){

	 uint8_t status=0;

		printMessage("BL_enable_rw_protect is run\n");
		uint32_t command_packet_lenght=bl_rx_data[0]+1;
		uint32_t host_crc= *((uint32_t*)bl_rx_data+command_packet_lenght-4);

		if(bootloader_verifiy_crc(bl_rx_data[0], command_packet_lenght-4, host_crc))
		{

			printMessage("BL_DEBUG_MESSAGE: Checksum succes \n");
			bootloader_send_ack(1);
			status= configure_flash_sector_rw_protection(0,0,1);
			printMessage("BL_DEBUG_MESSAGE: Flash protect status : %d \n",status);
			bootloader_uart_write_data(&status, 1);
		}
		else{
			bootloader_send_nack();
			}
}

uint8_t configure_flash_sector_rw_protection(uint8_t sectorDetails,uint8_t protesctionModes,uint8_t enableOrDisable){

	volatile uint32_t *pOPTCR=(uint32_t*)0x40023C14;

	//enableorisable==0 -> en-wr-protect | enableordisable==1, en-wr-protect | disableordisable==1,
if(enableOrDisable==1){
	HAL_FLASH_OB_Unlock();

	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);

	*pOPTCR |= (0xFF <<16 );

	*pOPTCR |= (1<<1);

	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);

	HAL_FLASH_OB_Lock();

}else{
	if(protesctionModes==1){
			//write protection

			HAL_FLASH_OB_Unlock();

			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);

			*pOPTCR &= ~(sectorDetails <<16 );

			*pOPTCR |= (1<<1);

			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);

			HAL_FLASH_OB_Lock();

		}else if(protesctionModes==2){
			//read write protection

			HAL_FLASH_OB_Unlock();

			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);

			*pOPTCR &= ~(0xFF <<16 );

			*pOPTCR |= (sectorDetails<<16);

			*pOPTCR |= (1<<1);

			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)!=RESET);

			HAL_FLASH_OB_Lock();
		}
}

	return 0;
}
uint8_t execute_flash_erase(uint8_t sectorNumber,uint8_t numberOfSector){

	FLASH_EraseInitTypeDef FLASH_EraseInitStruct={0};
	uint32_t sectorError=0;
	HAL_StatusTypeDef status={0};

	if(numberOfSector>11){
		return INVALID_SECTOR;

	}if((numberOfSector<=11) || sectorNumber==0xFF ){

		if(sectorNumber==0xFF ){
			FLASH_EraseInitStruct.TypeErase=FLASH_TYPEERASE_MASSERASE;
		}
		else{//sector erase

			uint8_t remainingSector =11 -sectorNumber;
			if(sectorNumber>remainingSector)
				sectorNumber=remainingSector;
			FLASH_EraseInitStruct.TypeErase=FLASH_TYPEERASE_SECTORS;
			FLASH_EraseInitStruct.Sector=sectorNumber;
			FLASH_EraseInitStruct.NbSectors=numberOfSector;
		}
		FLASH_EraseInitStruct.Banks=FLASH_BANK_1;
		HAL_FLASH_Unlock();
		FLASH_EraseInitStruct.VoltageRange=FLASH_VOLTAGE_RANGE_3;
		status=(uint8_t) HAL_FLASHEx_Erase(&FLASH_EraseInitStruct, &sectorError);
		HAL_FLASH_Lock();
		return status;
	}

}
uint8_t execute_memory_write(uint8_t * buffer,uint32_t memAddress,uint32_t len){

	uint8_t status=0;
	HAL_FLASH_Unlock();

	for (int i = 0; i < len; i++) {
		status=HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,memAddress + i,buffer[i]);
	}

	HAL_FLASH_Lock();
	return status;
}

uint8_t bootloader_verify_address(uint32_t goAddress){

	if(goAddress >= FLASH_BASE && goAddress <= FLASH_END  ){
		return ADDR_VALID;
	}else if(goAddress >= SRAM1_BASE && goAddress <= SRAM1_END ){
		return ADDR_VALID;
	}else if(goAddress >= SRAM2_BASE && goAddress <= SRAM2_END ){
		return ADDR_VALID;
	}else{
		return ADDR_INVALID;
	}

}

uint8_t  bootloader_verifiy_crc(uint8_t *buffer, uint32_t len,uint32_t crc)
{

 uint32_t crcValue=0xFF;
 uint32_t data=0;

 for(uint32_t i=0;i<len; i++ ){
		data=buffer[i];
		crcValue=HAL_CRC_Accumulate(&hcrc, &data, 1);

 }
 __HAL_CRC_RESET_HANDLE_STATE(&hcrc);
 if(crcValue==crc){
	 return CRC_SUCCESS;
 }
 else{
	 return CRC_FAIL;
 }
}

void bootloader_send_ack(uint8_t followLenght){
	uint8_t ackBuffer[2];
	ackBuffer[0]=BL_ACK_VALUE;
	ackBuffer[1]=followLenght;
	HAL_UART_Transmit(&huart3, &ackBuffer, 2, HAL_MAX_DELAY);

}
void bootloader_send_nack(){
	uint8_t nackValue= BL_NACK_VALUE;
	HAL_UART_Transmit(&huart3, &nackValue, 1, HAL_MAX_DELAY);
}

void bootloader_uart_write_data(uint8_t *dataBuff, uint8_t len){
	HAL_UART_Transmit(&huart3, dataBuff, len, HAL_MAX_DELAY);
}
uint8_t bootloader_Get_version(){
	return BL_VER;
}
/*uint16_t get_mcu_chipid(void){
	return (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;

}*/

uint8_t get_flash_rdp_levell(void){
	uint8_t rdp_level=0;
#if 0
	volatile (uint32_t)OB_Address=(uint32_t*)0x1FFFC000;
	rdp_level=(uint8_t)(*OB_Address>>8);
	return rdp_level;
#else
	FLASH_OBProgramInitTypeDef OBProgramInitTStruct;
	HAL_FLASHEx_OBGetConfig(&OBProgramInitTStruct);
	rdp_level=(uint8_t)OBProgramInitTStruct.RDPLevel;
	return rdp_level;
#endif
}










