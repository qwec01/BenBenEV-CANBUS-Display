# BenBenEV-CANBUS-Display
长安奔奔EV-CAN总线显示屏+车速HUD，17款奔奔EV180测试通过  

野生程序猿，程序能跑但不好看，欢迎正规程序猿修改  
本程序为长安奔奔EV的CAN总线数据显示屏，主控为Arduino，信息包括电压、电流、功率、速度、电池组最高/低单体电压、电池组最高/低温度等等  
Libraries/TM1638为自编库，请下载后复制到Arduino IDE的库文件夹  
同时本程序使用了pierremolinaro/acan2515库  
链接：https://github.com/pierremolinaro/acan2515  
下载该库后复制到Arduino IDE的库文件夹，也可从IDE中下载库
显示屏使用X宝购得的USART串口屏  
显示屏工程文件使用了字体 Digital Counter 7  
下载地址https://en.m.fontke.com/family/431683/  
CAN控制器为MCP2515，CAN收发器为TJA1050，数字隔离芯片ADUM1201  
本程序仍不稳定  
