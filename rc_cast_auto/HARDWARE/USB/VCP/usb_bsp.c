#include "usb_bsp.h"
#include "Includes.h"

#define USB_HOST_PWRCTRL PAout(15)

//USB板级初始化
void USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE *pdev)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12; //OTG FS Data + | OTG FS Data -
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; //JTDI
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure); //JTDI引脚配置后可使MCU复位后重新初始化USB
    USB_HOST_PWRCTRL = 1;
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_OTG1_FS);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_OTG1_FS);
}

//USB全局中断
void USB_OTG_BSP_EnableInterrupt(USB_OTG_CORE_HANDLE *pdev)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

//微秒级延时
void USB_OTG_BSP_uDelay(const uint32_t usec)
{
    delay_us(usec);
//  uint32_t count = 0;
//  const uint32_t utime = (120 * usec / 7);
//  while (++count <= utime);
}

//毫秒级延时
void USB_OTG_BSP_mDelay(const uint32_t msec)
{
    delay_ms(msec);
//  USB_OTG_BSP_uDelay(msec * 1000);
}
