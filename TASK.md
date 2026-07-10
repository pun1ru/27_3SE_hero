# Tasks
## 目标一-规范hero_up的头文件应用-参考hero_down里面的general_task_include.h文件,把常用应用和驱动文件都放在里面,然后在各个任务的.c文件中包含general_task_include.h,减少头文件的包含数量,提高编译速度

## 目标二-参考hero_down的任务调度和任务划分,把原来的整个robot_control_task.c文件拆分成多个任务task_decision,task_control,task_estimate,依然是传递信号量方式imu->estimate->control,具体的任务周期参照hero_down

## 目标三-优化掉僵尸变量声明和注释没有用用的东西

## 目标四-参考hero_down的MainControl文件夹,把各个模块的控制分散到MainControl里面,值得注意的是有一个worldGimbal模式采用世界系坐标控制云台,你可以写一个markdown文档-gimbalControl说明这个世界系控制逻辑,你顺便把整个云台的控制逻辑和控制逻辑都写好,包括不同控制模式,控制输入量的来源,不同控制算法,不同观测量什么的,方便后续维护理解