/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "app_debug.h"
/* USER CODE END Header */
#include "fatfs.h"

uint8_t retUSER;    /* Return value for USER */
char USERPath[4];   /* USER logical drive path */
FATFS USERFatFS;    /* File system object for USER logical drive */
FIL USERFile;       /* File object for USER */

/* USER CODE BEGIN Variables */
FRESULT fresult;
/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the USER driver ###########################*/
  retUSER = FATFS_LinkDriver(&USER_Driver, USERPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */
uint32_t fatfs_read_file(const char *file, uint8_t *data, uint32_t size)
{
    UINT byte_read = 0;

    fresult = f_open(&USERFile, file, FA_READ);
    if (fresult != FR_OK)
    {
        DEBUG_ERROR("Open file %s failed %d\r\n", file, fresult);
        goto end;
    }

    fresult = f_lseek(&USERFile, SEEK_SET);
    if (FR_OK != fresult)
    {
        DEBUG_ERROR("Seek file %s failed\r\n", file);
        f_close(&USERFile);
        goto end;
    }

    fresult = f_read(&USERFile, data, size, &byte_read);
    if (FR_OK != fresult)
    {
        DEBUG_ERROR("Read file %s failed %d\r\n", file, fresult);
        f_close(&USERFile);
        goto end;
    }

    f_close(&USERFile);

end:
    return byte_read;
}

int32_t fatfs_read_file_at_pos(const char *file, uint8_t *data, uint32_t size, uint32_t pos)
{
    UINT byte_read = 0;

    fresult = f_open(&USERFile, file, FA_READ);
    if (fresult != FR_OK)
    {
        DEBUG_ERROR("Open file %s failed %d\r\n", file, fresult);
        goto end;
    }
    DEBUG_VERBOSE("File %s opened\r\n", file);

    fresult = f_lseek(&USERFile, SEEK_SET);
    if (FR_OK != fresult)
    {
        DEBUG_ERROR("[0] Seek file %s failed\r\n", file);
        f_close(&USERFile);
        goto end;
    }

    fresult = f_lseek(&USERFile, pos);
    if (FR_OK != fresult)
    {
        DEBUG_ERROR("[1] Seek file %s failed, %d\r\n", file, fresult);
        HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
        f_close(&USERFile);
        goto end;
    }

    fresult = f_read(&USERFile, data, size, &byte_read);
    if (FR_OK != fresult)
    {
    	HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
        DEBUG_ERROR("Read file %s failed %d\r\n", file, fresult);
        f_close(&USERFile);
        goto end;
    }

    f_close(&USERFile);
    DEBUG_VERBOSE("File %s closed\r\n", file);
end:
    return byte_read;
}

int32_t fatfs_get_file_size(const char *file)
{
    int32_t size = -1;
//    if (!m_sdcard_is_mounted)
//    {
//        goto end;
//    }
    fresult = f_open(&USERFile, file, FA_READ);
    if (fresult != FR_OK)
    {
    	if (fresult == 4) size = 0;
        DEBUG_ERROR("Open file %s failed %d\r\n", file, fresult);
        goto end;
    }

    fresult = f_lseek(&USERFile, SEEK_SET);
    if (FR_OK != fresult)
    {
        f_close(&USERFile);
        DEBUG_ERROR("Seek file %s failed %d\r\n", file, fresult);
        goto end;
    }

    size = f_size(&USERFile);
    f_close(&USERFile);

end:
    return size;
}

uint32_t fatfs_write_to_a_file_at_pos (const char* file, char* buff, uint32_t size, uint32_t pos)
{
	UINT byte_write = 0;
	// step1 : check co file hay ko
	// neu co thi xoa file
	// ghi vao file
	fresult = f_open(&USERFile, file, FA_CREATE_ALWAYS);
	if (fresult != FR_OK)
	{
		DEBUG_ERROR("Open file %s failed %d\r\n", file, fresult);
		goto end;
	}
	f_close(&USERFile);
	fresult = f_open(&USERFile, file, 	FA_OPEN_APPEND | FA_WRITE);
	if (fresult != FR_OK)
	{
		DEBUG_ERROR("Open file %s failed %d\r\n", file, fresult);
		goto end;
	}

	fresult = f_lseek(&USERFile, (FSIZE_t)pos);
	if (FR_OK != fresult)
	{
		DEBUG_ERROR(" Seek file %s at 0 failed\r\n", file);
		f_close(&USERFile);
		goto end;
	}

	fresult = f_write (&USERFile, buff, size, &byte_write);
	if (fresult != FR_OK)
	{
		DEBUG_INFO ("ERROR %d", fresult);
		DEBUG_ERROR ("WRITE FILE %s FAIL", file);
		f_close(&USERFile);
		goto end;

	}
	f_close(&USERFile);
	end:
	    return byte_write;
}
uint8_t check_file (const char* file)
{
	FRESULT fr;
	FILINFO fno;
	fr = f_stat (file, &fno);
	switch (fr)
	{
	case FR_OK:
		DEBUG_INFO ("THERE IS FILE %s\r\n", file);
		return 0;

	case FR_NO_FILE:
//		DEBUG_INFO ("NO FILE %s\r\n", file);
		return 1;
	default:
		DEBUG_ERROR ("AN ERROR OCCURED\r\n");
		return 2;
	}
}
void delete_a_file (const char * file)
{
	fresult = f_unlink (file);
}
//FILINFO scan_files (char* pat)
//{
//    DIR dir;
//    UINT i;
//    FILINFO fno;
//    static char buffer[128];
//    char path[20];
//    sprintf (path, "%s",pat);
//
//    fresult = f_opendir(&dir, path);                       /* Open the directory */
//    if (fresult == FR_OK)
//    {
//        for (;;)
//        {
//            fresult = f_readdir(&dir, &fno);                   /* Read a directory item */
//            if (fresult != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
//            if (fno.fattrib & AM_DIR)     /* It is a directory */
//            {
//
//            	if (!(strcmp ("SYSTEM~1", fno.fname))) continue;
//            	DEBUG_INFO ("FIND A DIR \r\n");
////            	sprintf (buffer, "Dir: %s\r\n", fno.fname);
////            	send_uart(buffer);
////                i = strlen(path);
////                sprintf(&path[i], "/%s", fno.fname);
//                fresult = scan_files(path);                     /* Enter the directory */
//                if (fresult != FR_OK) break;
//                path[i] = 0;
//            }
//            else
//            {                                       /* It is a file. */
//               sprintf(buffer,"File: %s/%s\n", path, fno.fname);
////               send_uart(buffer);
//               DEBUG_INFO ("%s\r\n", buffer);
//
//            }
//        }
//        f_closedir(&dir);
//    }
//    return fresult;
//}
FRESULT create_a_dir (const char * path)
{
	fresult = f_mkdir (path);
	if (fresult != FR_OK)
	{
		DEBUG_ERROR ("ERROR IN CREAT DIR\r\n");
	}
	return fresult;
}

/* USER CODE END Application */
