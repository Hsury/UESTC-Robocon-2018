#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "stdlib.h"
#include "malloc.h"
#include <jansson.h>
u8 sramx=0;
//ALIENTEK 探索者STM32F407开发板 实验4
//串口通信实验 -库函数版本
//技术支持：www.openedv.com
//淘宝店铺：http://eboard.taobao.com
//广州市星翼电子科技有限公司  
//作者：正点原子 @ALIENTEK

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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);		//延时初始化 
	uart_init(57600);	//串口初始化波特率为115200
  LED_Init();
	my_mem_init(SRAMIN);
	json_set_alloc_funcs(mallocx, freex);
	//printf("\r\nSystem started.\r\n");

	while(1)
	{
		if (my_mem_perused(SRAMIN)>80){  // 万一堆意外满了就重新初始化内存分配
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

