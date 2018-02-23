1. 编译时提示 ..\SYSTEM\usart\usart.c(47): error:  #260-D: explicit type is missing ("int" assumed)
               _sys_exit(int x)
   解决方式：在正点原子的usart.c文件的第47行行首加入void
2. UTF-8编码下，常规汉字占用3Bytes
3. ST官方驱动使用环形缓冲区来存储待向主机发送的数据，逻辑上存在漏洞，暂时先写了个缓冲区动态解压的功能，足够适配大部分应用场景
4. 2018/02/03性能测试结果显示，800KB/s为目前可实现的最大速度
5. 如果要提升波特率，注意调整usbd_conf.h文件中关于APP_RX_DATA_SIZE的宏定义大小，如果过小易导致很高的误码率
6. .USB_DEVICE与.USB_DRIVER分组中为ST提供的标准库USB驱动包，文件属性设置为只读，正常情况下没有改动的必要
7. 板级初始化时，注意开启USB外设时钟：RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE);

2018/02/23 HsuRY
