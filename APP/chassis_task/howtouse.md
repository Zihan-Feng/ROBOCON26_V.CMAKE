solution文件是底盘解算的类文件，后续更新底盘结构在里面新建类就行
用的时候直接
MecanumChassis chassis_solver(chassis_motor1, chassis_motor2, chassis_motor3,
                chassis_motor4);
实例化应该底盘类把你几个电机丢进去就行，注意电机顺序，这里我定义的依次是左上->右上->左下->右下
然后再把pid参数加进去
chassis_solver.configureSpeedPid(kWheelPidParams);
如果要边跑边跳就在for循环里一直调用这个函数就行

