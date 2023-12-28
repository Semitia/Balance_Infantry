/**********************************************************************************************************
 * @文件     SDCardTask.c
 * @说明     读写SD卡，记录日志
 * @版本  	 V1.0
 * @作者     段康晟
 * @日期     2022.7
 **********************************************************************************************************/
#include "SDCardTask.h"

// BYTE ReadBuffer[1024] = {0};         /* 读缓冲区 */
char WriteBuffer[256] = "";              /* 写缓冲区*/
TCHAR DataFile[] = "0:infantry3-12.txt"; //文件名
SDStatus sd_status;

void SDCard_task(void *pvParameters)
{
    // SD卡初始化
    while ((sd_status.Status = SD_Init()) != SD_RESPONSE_NO_ERROR)
        sd_status.SD_init_result = 0; // printf("SD卡初始化失败，请确保SD卡已正确接入开发板，或换一张SD卡测试！\n");
    sd_status.SD_init_result = 1;     // printf("SD卡初始化成功！\n");

    //挂载文件系统
    sd_status.res_sd = f_mount(&sd_status.fs, "0:", 1); //在外部SPI Flash挂载文件系统，文件系统挂载时会对SPI设备初始化

    if (sd_status.res_sd != FR_OK)
        sd_status.SD_FS_Mount_result = 0; //挂载失败
    else
        sd_status.SD_FS_Mount_result = 1; //挂载成功

    while (1)
    {
        vTaskDelay(1000);

        sd_status.count++;

        if (sd_status.count % 2 == 0) // 500hz
        {
        }

        if (sd_status.count % 5 == 0) // 200hz
        {
        }

        if (sd_status.count % 200 == 0) // 5HZ
        {
        }

        if (sd_status.count % 1000 == 0) // 1hz
        {
        }

        //打开文件（追加写模式）
        sd_status.res_sd = f_open(&sd_status.fnew, (const TCHAR *)DataFile, FA_OPEN_EXISTING | FA_READ); //打开已经存在的文件
        uint32_t file_size = 0;

        if (sd_status.res_sd == FR_OK)
            file_size = f_size(&sd_status.fnew); //获得文件已经写入的长度
        f_close(&sd_status.fnew);                //关闭文件

        sd_status.res_sd = f_open(&sd_status.fnew, (const TCHAR *)DataFile, FA_CREATE_ALWAYS | FA_WRITE); //打开文件，如果文件不存在则创建它
        if (sd_status.res_sd == FR_OK)
        {
            sd_status.SD_FS_Open_result = 1; //打开文件成功
            if (file_size > 0)               //判断文件非空（大于零的数），并寻找添加头（即文件尾）
                sd_status.res_sd = f_lseek(&sd_status.fnew, file_size);
            memset(WriteBuffer, '\0', sizeof(WriteBuffer)); //清空缓存区
            if (sd_status.SDCard_task_init == 0)
            {
                sd_status.SDCard_task_init = 1;
                sprintf(WriteBuffer, "******************Start"
                                     "******************\n");
            }
            else
            {
            }
            sd_status.res_sd = f_write(&sd_status.fnew, WriteBuffer, strlen(WriteBuffer), &sd_status.fnum);
            if (sd_status.res_sd == FR_OK)
                sd_status.SD_FS_Write_result = 1; //写入成功
            else
                sd_status.SD_FS_Write_result = 0;
            f_close(&sd_status.fnew); //关闭文件
        }
        else
            sd_status.SD_FS_Open_result = 0;
    }
    // f_mount(NULL, "0:", 1); /* 不再使用文件系统，取消挂载文件系统 */
}
