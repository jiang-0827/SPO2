/*
 * ������������Ӳ�������������չ����Ӳ�����Ϲ���ȫ����Դ
 * �����������www.lckfb.com
 * ����֧�ֳ�פ��̳���κμ������⻶ӭ��ʱ����ѧϰ
 * ������̳��club.szlcsc.com
 * ��עbilibili�˺ţ������������塿���������ǵ����¶�̬��
 * ��������׬Ǯ���������й�����ʦΪ����
 * 
 
 Change Logs:
 * Date           Author       Notes
 * 2024-03-07     LCKFB-LP    first version
 */
 
 #ifndef __BSP_UART_H__
 #define __BSP_UART_H__
 
 #include "stm32f4xx.h"
 
 
 
 //�ⲿ�ɵ��ú���������
void uart1_init(uint32_t __Baud);
void uart_send_buf(uint8_t* buf, uint16_t len);
void USART1_IRQHandler(void);
 
 
 
 
 
 
 
#endif
