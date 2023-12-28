/**********************************************************************************************************
 * @�ļ�     SDCardTask.c
 * @˵��     ��дSD������¼��־
 * @�汾  	 V1.0
 * @����     �ο���
 * @����     2022.7
 **********************************************************************************************************/
#include "SDCardTask.h"

// BYTE ReadBuffer[1024] = {0};         /* �������� */
char WriteBuffer[256] = "";              /* д������*/
TCHAR DataFile[] = "0:infantry3-12.txt"; //�ļ���
SDStatus sd_status;

void SDCard_task(void *pvParameters)
{
    // SD����ʼ��
    while ((sd_status.Status = SD_Init()) != SD_RESPONSE_NO_ERROR)
        sd_status.SD_init_result = 0; // printf("SD����ʼ��ʧ�ܣ���ȷ��SD������ȷ���뿪���壬��һ��SD�����ԣ�\n");
    sd_status.SD_init_result = 1;     // printf("SD����ʼ���ɹ���\n");

    //�����ļ�ϵͳ
    sd_status.res_sd = f_mount(&sd_status.fs, "0:", 1); //���ⲿSPI Flash�����ļ�ϵͳ���ļ�ϵͳ����ʱ���SPI�豸��ʼ��

    if (sd_status.res_sd != FR_OK)
        sd_status.SD_FS_Mount_result = 0; //����ʧ��
    else
        sd_status.SD_FS_Mount_result = 1; //���سɹ�

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

        //���ļ���׷��дģʽ��
        sd_status.res_sd = f_open(&sd_status.fnew, (const TCHAR *)DataFile, FA_OPEN_EXISTING | FA_READ); //���Ѿ����ڵ��ļ�
        uint32_t file_size = 0;

        if (sd_status.res_sd == FR_OK)
            file_size = f_size(&sd_status.fnew); //����ļ��Ѿ�д��ĳ���
        f_close(&sd_status.fnew);                //�ر��ļ�

        sd_status.res_sd = f_open(&sd_status.fnew, (const TCHAR *)DataFile, FA_CREATE_ALWAYS | FA_WRITE); //���ļ�������ļ��������򴴽���
        if (sd_status.res_sd == FR_OK)
        {
            sd_status.SD_FS_Open_result = 1; //���ļ��ɹ�
            if (file_size > 0)               //�ж��ļ��ǿգ����������������Ѱ�����ͷ�����ļ�β��
                sd_status.res_sd = f_lseek(&sd_status.fnew, file_size);
            memset(WriteBuffer, '\0', sizeof(WriteBuffer)); //��ջ�����
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
                sd_status.SD_FS_Write_result = 1; //д��ɹ�
            else
                sd_status.SD_FS_Write_result = 0;
            f_close(&sd_status.fnew); //�ر��ļ�
        }
        else
            sd_status.SD_FS_Open_result = 0;
    }
    // f_mount(NULL, "0:", 1); /* ����ʹ���ļ�ϵͳ��ȡ�������ļ�ϵͳ */
}
