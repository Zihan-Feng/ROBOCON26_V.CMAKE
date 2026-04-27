待完成：底盘速度外环pid，底盘位置外环pid，电机位置环


目前是本工具链是生产elf通过ozone进行调试
现在能用jlink，也能用无线daplink，但是无线调试不能看曲线
怎么用无线dap呢：
https://bbs.robomaster.com/article/1638268?source=1

目前数据内存默认都存储在DTCMRAM里，其他SRAM基本没用上，如果以后 DTCM 接近满了，可以考虑把不敏感的大数组/缓存挪到 D1/D2，具体怎么改到时再看看
除此之外，我（fzh）把pid计算放在了ITCMRAM里，我也不知道这能够优化多少
上述说的这俩RAM的优势是什么呢
DTCMRAM
Data TCM RAM，紧耦合数据内存，速度非常快，通常给实时性高的数据/栈
ITCMRAM
Instruction TCM RAM，紧耦合指令内存，通常放极高实时性的代码

后续的开发流程非常之明了啊，
要添加外设的话：在module层调用BSP层的固件接口（目前只有串口，而且默认是开dma的）->在module层写功能函数以及封装函数接口->APP成调用你的功能函数
要添加机器人执行逻辑：在APP层调用所需的功能函数->debug任务书写功能逻辑、状态机->测试->新建个任务文件把内容装里面
<img width="850" height="857" alt="6324839ae149e8f52d0411ab866e990b" src="https://github.com/user-attachments/assets/525838a6-2a47-48ca-a4d0-c90de74be154" />

