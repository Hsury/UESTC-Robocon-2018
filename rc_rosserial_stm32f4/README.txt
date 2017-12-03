Arduino/ESP8266上使用ArduinoJson库+动态内存分配
STM32上使用Keil:Jansson库+原子的Malloc库
注意对json_t *类型的内存释放，使用json_decref()函数，详见https://jansson.readthedocs.io/en/2.7/apiref.html#reference-count
对于非json数据类型，将myfree()包装后进行释放

更新日志：
2017/10/12: 对原子的程序修改了堆大小，开启了MicroLib，修改串口接收策略（对RX缓冲区末尾自动补\0），修复爆堆问题