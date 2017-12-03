#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "stdlib.h"
#include "malloc.h"
#include <jansson.h>
u8 sramx=0;
//ALIENTEK ̽����STM32F407������ ʵ��4
//����ͨ��ʵ�� -�⺯���汾
//����֧�֣�www.openedv.com
//�Ա����̣�http://eboard.taobao.com
//������������ӿƼ����޹�˾  
//���ߣ�����ԭ�� @ALIENTEK

//jansson Test
int linear[3], angular[3];

static void *mallocx(size_t size)
{
  return mymalloc(sramx, size);
}

static void freex(void *ptr)
{
  myfree(sramx, ptr);
}

void jsonEncode(void) {
  json_t *root;
  char *out;
	root = json_pack("{s:i,s:i,s:i}", "led0", 1 - LED0, "led1", 1 - LED1, "buzz", BUZZ);
  out = json_dumps(root, JSON_ENCODE_ANY);
	printf("%s", out);
  json_decref(root);
  freex(out);
}

void jsonDecode(void) {
  json_t *root;
	char *in = (char *) &USART_RX_BUF;
	root = json_loads(in, JSON_DECODE_ANY, NULL);
	json_unpack(root, "{s:{s:i,s:i,s:i},s:{s:i,s:i,s:i}}", "linear", "x", &linear[0], "y", &linear[1], "z", &linear[2], "angular", "x", &angular[0], "y", &angular[1], "z", &angular[2]);
	json_decref(root);
  freex(in);
}

void dataProcess(void) {
	if (linear[0] > 0) LED0 = 1 - LED0;
	if (linear[0] < 0) LED1 = 1 - LED1;
	if (angular[2] > 0) BUZZ = 0;
	if (angular[2] < 0) BUZZ = 1;
}

int main(void)
{  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);		//��ʱ��ʼ�� 
	uart_init(57600);	//���ڳ�ʼ��������Ϊ115200
  LED_Init();
	my_mem_init(SRAMIN);
	json_set_alloc_funcs(mallocx, freex);
	//printf("\r\nSystem started.\r\n");

	while(1)
	{
		if (my_mem_perused(SRAMIN)>80){  // ��һ���������˾����³�ʼ���ڴ����
			//printf("SRAM overflow\r\n");	
			my_mem_init(SRAMIN);
		}
		if(USART_RX_STA&0x8000)
		{
			jsonDecode();
			dataProcess();
			USART_RX_STA=0; 
      jsonEncode();
		}
    //printf("Usage: %d\r\n", my_mem_perused(SRAMIN));
	}
}

