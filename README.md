一、控制系统设计

基于老师给出的实验要求，我们总结出整个任务分为以下几个步骤：

①小车在初始位置，能够自动纠正当前角度以对准跷跷板并且自主行驶到跷跷板上。

②小车在跷跷板上找到平衡位置，并保持十秒左右的静止。

③小车沿着跷跷板自动返回起始位置。

  发挥部分要求在跷跷板的一端两次增加质量为机器人配重的10%到20%的配重，小车仍能保持平衡。在挑战部分中，需要在翘翘板上放置一个滚动的铁球并且小车仍能保持平衡。
  
  从基本要求入手，我们进行了初步的讨论。小车需满足一下几点基本的要求：
  
①要能实时感知自己的YAW角，在起始位置和行驶中能够依据给定值不断校正自己的航向以保证对准跷跷板，避免在行驶过程中跌落。

②要能实时感知自己的PITCH角，在跷跷板上能够通过PID算法调节自己的速度大小和方向，最终成功找到平衡位置。

小车控制系统框图如下：

![IMG_256](https://github.com/dragonundertheworld/module-robot/blob/main/img/control_system.jpg)

图 1-1小车控制系统框图
 

1.1机械结构设计

机械结构总共分为五个模块，分别是：控制器模块、驱动模块、电源模块、传感器模块、机架

1.1.1机架与轮子布局

​     小车的整体布局分为3轮式和4轮式，由于4轮布局转向阻力较大且安装更麻烦，因此我们决定用3轮布局，后两轮为驱动轮，前轮使用万向轮作为支撑。根据已有的各类元件的尺寸以及提

供的材料，我们决定将机架整体设计成长方形，长约20cm,宽约13cm。然后我们设计了前后轮的最佳车轴距以及后轮之间的距离。我们经过考虑与挑选后，决定用两根13cm的铝型材作为机架的

宽，两根20cm的作为机架的长。两后轮通过圆周舵机与机架间接相连，前轮通过连接件与机架相连。

1.1.2驱动模块布局

驱动模块由舵机通过联轴器与轮胎相连，并选择舵机支架使其与整体连接固定。两个舵机实现平衡车的前进和转向运动。

1.1.3电源模块布局

电源采用两节18650电池供电。由于电机在后方，为保持重心我们决定把电池盒放在机架前方。

1.1.4 接线端与稳压模块布局

稳压模块与接线端用于连接电源模块与控制器模块，为使整体的布局更加清晰以及安装更加便利，我们决定将之放在中间。

1.1.5控制器模块布局

​     平衡车整体通过stm32进行控制。安装方面，为了稳定与接线的方便，我们采用普遍使用的面包板作为线路连接的道具，stm32单片机的引脚插入宽条插孔中，传出、传入的信号通过纵向

连通插孔进行传输。为使整体的布局更加清晰，我们决定将其放在机架后方。

1.1.6 传感器模块布局

传感器使用实验室提供的GY-953九轴姿态传感器，即三轴陀螺仪，三轴加速度传感器和三轴地磁传感器，分别获得速度，加速度和磁向角等数据，通过串口通讯以数据桢的形式可以实时获取小

车的欧拉角（roll，pitch，yaw）数据。为避免电线纠缠和电源模块及执行模块的电磁干扰，将传感器远离这两者并安装在前轮上方。
 

1.2传感系统设计

1.2.1 传感器模块及功能

传感器模块包括传感器、上位机Gy Electronic V3、 TTL转接口以及控制器。其中传感器使用实验室提供的GY-953九轴姿态传感器，即三轴陀螺仪，三轴加速度传感器和三轴地磁传感器，分

别获得速度，加速度和磁向角等数据，通过串口通讯以数据帧的形式可以实时获取小车的欧拉角（roll，pitch，yaw其中Yaw用于判断小车偏离预定航向的程度，Pitch用于判断小车平衡的程

度）数据，其传输频率为50Hz且容易受到地磁影响，因此每次使用前需进行地磁航线校准；上位机用于直接得到可视化后的传感器数据，其与传感器之间通过USB转TTL转接口互相发送数据，这

种数据传输方式被用于测试传感器的好坏；控制器与传感器通讯，并将得到的数据传给电脑，这种数据传输方式用于对小车的实际控制中。实际传感系统的设计中，我们发现俯仰角会影响偏航

角，于是我们提出了将磁力角用于修正航向的方案并尝试了一下，但实际效果不佳，最终还是选择了初版方案。


1.3执行系统设计

执行器选择和功能

我们小组设计的是一个三轮车，前轮无驱动，后两轮为执行器，由提供两个舵机驱动。通过控制舵机同时同向转动控制小车的移动，通过改变转向控制小车的前进与后退，通过改变舵机的转速

控制速度，通过使舵机的产生差速控制转动。

 

1.4控制算法设计

​      跷跷板小车平衡算法的核心为PID控制算法，接下来结合实际，对程序中所使用的PID算法原理进行分析。

​        在实际中，我们使用到了九轴传感器的磁力角功能和欧拉角功能。我们将实时采集到的欧拉角滤波后通过与目标角度相减以得到对应偏差值err，将err逐次累加以得到err积分，将当

前err与上一次err作差以得到err微分，并用于PID算法的具体计算。PID算法的公式如下所示：

![IMG_256](https://github.com/dragonundertheworld/module-robot/blob/main/img/PID.jpg)

​     这里我们采用了PID算法的离散形式。这也是因为我们实际得到的传感器数据是离散的数字信号，而非连续模拟信号。其中e（k）即为当前所得err，通过对err的计算即可得出对应的修

正信号。可以看到PID算法主要分为三个部分，比例项、积分项以及微分项。这三项分别发挥着不同的作用有不同的特性，因此系数也各不相同，在实际调试中，我们需要通过理论和实验相结

合，最终得到合适的比例系数kp、积分系数ki以及微分系数kd。他们对系统的影响如下所示：

比例项的作用

当kp增大时，系统的超调量随之增大，系统的响应速度会更快，实际表现的更加灵敏。

积分项的作用

积分控制主要是为了改善系统的稳态性能，消除系统的稳态误差。当系统存在稳态误差时，积分项就会发挥作用直至稳态误差减小到0，积分控制停止。积分控制的加入会使系统的相对稳定性

变差。ki减小会导致系统的超调量增大，ki增大会导致系统响应趋于稳定值的速度减慢。

微分项的作用

微分环节的主要作用是提高系统的响应速度。当kd增大时，闭环系统响应的响应速度加快，调节时间减小。在误差变化趋势较大时，微分项会对系统施加合适的控制，但是过大的kd也会导致系

统出现问题。同时当信号变化缓慢或者无变化时，微分项不起作用。我们在调节时，将微分项看作是系统的阻尼，若系统对角度变化反应过大，则增大微分项。 

在实验的过程中我们对这几项的作用有了更加深刻的认识，并对调节过程总结如下：

对Pitch角PID控制的调节。

首先，是对kp项的调节。我们首先直接改变小车在跷跷板上的位置，大致找到了小车在跷跷板上的平衡点。然后，我们先将kp逐渐增大，并运行程序。直到小车能够在平衡点附近一定范围内做

来回振荡的运动。随后我们开始调试kd值，以减小小车振荡的幅度，并且使小车的运动逐渐收敛，最终能够停止在翘翘板上。但是这仍然会存在问题，即小车在停止不动时并非总是能够满足跷

跷板两端的高度差在3cm之内。因此我们需要增大第三个系数，即kd。若小车发现自己停止时仍存在较大误差，将会在一段时间后重新开始调节，直到满足我们设定的误差要求，即-1.6=

<Pitch_err=<1.6时，小车认为保持平衡成功，并开始计时。经过多次实验，我们发现小车在Pitch角偏差较小时，难以快速找到平衡点，有时反而出现了更为严重的振荡。为了同时满足Pitch

角偏差较大时，调节的速度以及Pitch角较小时调节的精准性，我们将Pitch的调节分为了两个阶段。当-2=<Pitch_err=<2时，ki与kd的值比-2>Pitch_err或Pitch_err>2时小一些，这样更加

符合实际的需要，即在角度变化已经比较微小时，降低小车每次调整幅度，从而使得整个调整过程更快的收敛。

对Yaw角的PID控制的调节

​     类比Pitch角的PID参数调整过程，Yaw角调整的过程更加简单一些，在经过多次实验后，我们发现只需要比例项和微分项的作用便可以使小车的方向快速对准，但是也带来了一些问题，

即小车可能会出现细微的偏差，但是这只是对于航向角来说，并不会对小车的平衡过程造成太大的影响。

​     经过多次尝试，我们将所有的参数逐一确定，并取得了不错的效果。但是我们也通过上位机观察到九轴传感器在运行过程中并不能做到十分稳定，在受到外界电磁干扰时，输出的信号极

易发生波动，曲线出现毛刺现象。故我们对采集到的信号进行了滤波处理。采用FIR滤波时，由于程序复杂和参数调整的困难，我们在实际应用中并未实现较好的效果。为了让程序更加的简

便，我们最终采用了均值滤波的方法，即将当前收集到的Yaw值与Pitch值分别存入YawTemp与PitchTemp中，在滤波函数中将多次得到的Temp值进行累加并取均值，作为PID算法中Yaw值与Pitch

值的当前值。采用这种算法时我们首先在滤波函数中进行了八次累加，但是结果却出现小车运行出错的情况，经过仔细的思考，我们发现是时序存在问题。由于传感器的工作频率为50hz。若要

采集八次数据并实现完整的八次累加则滤波函数在每一次相加后都至少需要等待传感器20ms，而我们的滤波程序单次只等待10ms，这直接导致了滤波函数的失效。最终我们将滤波函数的采集次

数改为五次并将单次等待时长延长到20ms，终于解决了外界干扰的问题。

二、控制系统的制作与调试

2.1机械结构的制作与调试

2.1.1 机械结构的制作

​     两个由舵机控制的轮胎安装在机架靠后的两侧，作支撑的轮胎安装在机架的正前方，其他模块零件统一安装在机架上方，从后向前依次是面包板、分线器以及电源，单片机安装在面包板上，传感器最后安装在单片机上方。

执行器安装和驱动

**执行器的安装**

​     舵机先通过螺钉与特定金属件连接固定，连接处安装垫片，再通过方形条安装在机架两侧，最后通过联轴器与轮胎连接驱动。前轮直接固定在机架前方，保持机架与地面的平行。

执行器驱动

​    舵机通过联轴器实现运动的运输，两个舵机的旋转速度的不同实现直线运动和旋转运动。

**传感系统的安装**

安装要求

设计时靠近重心，且安装在面包板上方，但安装时因为机架的设计形状改安装在前轮上方。为避免传感器受到电池和执行器的电磁干扰，将传感器远离这两者。

**电源模块的安装**

电机安装在小车的后半部分，为保持小车的重心，所以我们把电源模块安装在小车的前方并用扎带固定。

**驱动模块的安装**

舵机先通过螺钉与特定金属件连接固定，连接处安装垫片，再通过方形条安装在机架两侧，最后通过联轴器与轮胎连接驱动。前轮直接固定在机架前方，保持机架与地面的平行。

 

2.1.2 机械结构的调试

我们初次安装时仿照老师提供的范例连接前轮与机架，结果是前轮不能与后轮同时着地，于是我们在老师的基础上又增加了一块连接件作为过渡与延长，最终得到了满意的结果；另外我们在固定轮子时为了轮子不晃动将螺母拧得太紧导致前轮无法正常转动，转弯不顺畅，调整之后得到了一个平衡的结果，即轮子晃动幅度小且轮子能够顺畅转动；还有就是连接安装时因为所用铝型材的大小导致传感器没有地方固定，因此将传感器的位置移到前轮的上方；最后是安装时一开始没有注意安装顺序，导致有的地方先安装好后发现剩下的部分放不进去，不得不拆了重新调整顺序安装。

2.2电路系统的制作与调试

电路系统制作与搭建中，其中，不同模块之间的接线利用分线器或者面包板进行连接

![IMG_256](https://github.com/dragonundertheworld/module-robot/blob/main/img/circuit_design.jpg)

 

图2-1 电路搭建简图

 出现的问题、原因及解决：

（1）问题：传感器得到的航向角数据在每次开机时都会改变 

   原因：未进行地磁校准 

   解决：每天都要对传感器进行地磁校准，以确保其稳定性。

（2）问题：部分线路有些杂乱且连接不稳定。

   原因：在搭建时出现的问题有螺丝螺母占据位置或者公母线的长度不合适。

   解决：挑选合适长度的杜邦线重新连接，并用电胶带和扎带固定，使之整体整洁美观。 

如何分步检验的：

传感器航向角的问题但是当时没有太清楚，只知道每次都要改变程序中的航向角纠正值，将电池磁场、电机磁场、线路连接等可能的问题依次排除后也不知道为什么，后来老师讲解了之后才清楚。

2.3控制程序的编写与调试

控制程序主要分为初始化、电机控制、电机死区矫正、Yaw角以及Pitch角PID算法、传感器数据采集、滤波、串口通讯、小车控制主程序等部分组成。

初始化部分是通过stm32cubemx软件中进行设置后自动生成的；电机控制函数采用了提供的代码；电机死区矫正则是先编写了电机按键调速程序，通过实际测试，分别计算出了左右电机的大致死区。在死区矫正时分别将正负的死区跨越从而实现了死区矫正；PID算法是依据控制算法设计中所提到的原理和思路编写的；传感器数据采集程序中我们参考提供的代码，将采集得到yaw角和Pitch角分别赋值给对应的Temp变量实现了数据采样；滤波算法是采用了均值滤波的原理；串口通讯进程采用了提供的代码；小车控制进程实现的功能为：

1、小车开机后检测Yaw角err是否在（-2，2）之中，若不满足则首先利用PID算法调整Yaw角。

2、Yaw角满足条件后，小车将向前行驶六秒检测到不满足PitchError<=3 && PitchError>=-3的条件（说明小车已经驶上跷跷板）后进入Pitch角的调整过程。

3、当小车进入Pitch角的调整后，每隔4s将进行Yaw角的调整，每次调整Pitch角前也会检测Yaw角是否满足条件，避免小车掉下跷跷板。

4、小车进入平衡稳定状态后将开始计时，计时满15s小车开始倒退并在到达桌面后保持静止。

部分重要代码如下所示：

```c
// Motor Calibration 电机死区修正函数
void MotorCalib(uint8_t chn, int16_t spd)
{
 int16_t  dead_down_left, dead_down_right,dead_up_left,dead_up_right;
​    dead_down_left=-150+3*27;
​    dead_up_left=-150+3*62;
​    dead_down_right = -150+3*32;
​    dead_up_right = -150+3*67;
​    switch(chn)
​    {
​    case 0:
​         if(spd < 0)
​         {
​             spd = spd + dead_down_left;
​         }
​         else if(spd > 0)
​         {
​             spd = spd + dead_up_left;
​         }
​         break;
​    case 1:
​         if(spd < 0)
​         {
​             spd = spd + dead_down_right;
​         }
​         else if (spd > 0)
​         {
​             spd = spd + dead_up_right;
​         }
​         break;
​    default:
​         break;
​    }
 

​    MotorCtrl(chn, spd);
}
// 计算俯仰角与航向角平均值角度
void CalculateAverage(void)//以后需要改进成滤波算法
{
​    
​    int number = 5;
​    float SumOfPitch = 0;
​    float SumOfYaw = 0;
​    for(int i = 0; i < number; i++)
​    {
​         SumOfPitch += PitchTemp;
​         SumOfYaw += YawTemp;
​         osDelay(20);//这个Delay的作用：50Hz接收频率
​    }
​    PitchAverage = SumOfPitch / number; // 全局变量
​    YawAverage = SumOfYaw / number;   // 全局变量
​    osSemaphoreRelease(myBinarySem_MotorSpdChangeHandle);
}
 

//PID控制算法
struct _YawPIDParam
{
​    float ActualYaw;    // 定义实际值
​    float err;        // 定义偏差值
​    float err_last;      // 定义上一个偏差值
​    float Kp;         //定义比例系数
​    float Ki;         //定义积分系数
​    float Kd;         //定义微分系数
​    float integral;      //定义积分值
}YawPIDParam;
 

struct _PitchPIDParam
{
​    float ActualPitch;       // 定义实际值
​    float err;        // 定义偏差值
​    float err_last;      // 定义上一个偏差值
​    float Kp;         //定义比例系数
​    float Ki;         //定义积分系数
​    float Kd;         //定义微分系数
​    float integral;      //定义积分值
}PitchPIDParam;

void YawPIDInit()
{
​    YawPIDParam.ActualYaw = 0;
​    YawPIDParam.err = 0;
​    YawPIDParam.err_last = 0;
​    YawPIDParam.Kp = 1.2;//1.2-》1.4
​    YawPIDParam.Ki = 0;
​    YawPIDParam.Kd = 0.8;
​    YawPIDParam.integral = 0;
}

int YawPIDControl(float YawAverage)
{
​    YawPIDInit();
​    YawPIDParam.ActualYaw = YawAverage;
​    YawPIDParam.err = YawPIDParam.ActualYaw - YawAngleCalibration;
​    YawPIDParam.integral += YawPIDParam.err;
​    YawPIDParam.ActualYaw = YawPIDParam.Kp* YawPIDParam.err + YawPIDParam.Ki * YawPIDParam.integral + YawPIDParam.Kd * (YawPIDParam.err - YawPIDParam.err_last);
​    //UART2SendIntegerVariable(YawPIDParam.Kp);
​    YawPIDParam.err_last = YawPIDParam.err;
​    YawCorrectionSpeed = (int)(3* YawPIDParam.ActualYaw);//100*PID计算值，得到YawCorrectionSpeed
​    //UART2SendIntegerVariable(YawCorrectionSpeed);
​    osSemaphoreRelease(myBinarySem_YawCorrectionHandle);//释放Yaw信标
​    return YawCorrectionSpeed;
}

int PitchPIDControl(float PitchAverage)
{
​         PitchPIDParam.ActualPitch = PitchAverage; // 俯仰角均值传递给实际俯仰角
​         PitchPIDParam.err = PitchPIDParam.ActualPitch - PitchAngleCalibration; // 偏差
​         PitchPIDParam.integral += PitchPIDParam.err;
​         if (PitchPIDParam.integral > 100 || PitchPIDParam.integral < -100)
​         {
​             PitchPIDParam.integral = 100;
​         }
 
​         if (PitchPIDParam.err >= 2.0 || -2.0 >= PitchPIDParam.err)
​         {
​             PitchPIDParam.Kp =2.4;                // 定义比例系数
​             PitchPIDParam.Ki = 0.0015;                // 定义积分系数
​             PitchPIDParam.Kd = 21;
​         }
​         else if (PitchPIDParam.err < 2.0 && -2.0 < PitchPIDParam.err)
​         {
​             PitchPIDParam.Kp = 2.4;                // 定义比例系数
​             PitchPIDParam.Ki = 0.0003;                // 定义积分系数0.005-》0.003
​             PitchPIDParam.Kd = 20;//16-》17
​         }
​         PitchPIDParam.ActualPitch = PitchPIDParam.Kp * PitchPIDParam.err + PitchPIDParam.Ki * PitchPIDParam.integral + PitchPIDParam.Kd * (PitchPIDParam.err - PitchPIDParam.err_last);
​         PitchPIDParam.err_last = PitchPIDParam.err;
​         SPEED = (int)(6 * PitchPIDParam.ActualPitch);//更改全局变量speed
​         PitchError = PitchPIDParam.err; // 更改全局变量PitchError和SPEED
​         
​         return SPEED;
}

//Pitch调整过程中对Yaw的调整进程
void StartTaskMotorCtrl(void const * argument)
{
 /* USER CODE BEGIN StartTaskMotorCtrl */
​    //可以执行
 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);//打开时钟1
 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);//打开时钟2
 /* Infinite loop */
​    
 for(;;)
 {
//  osSemaphoreWait (myBinarySem_YawCorrectionHandle, osWaitForever);//暂不需要等待信标
​     osDelay(4000);//不只是不断断循环 = 20ms*5，行进过程中的修正间隔
​         while ((YawCorrectionSpeed < -5 || YawCorrectionSpeed > 5)&& flag==1)
​         {
​             CalculateAverage();
​             YawPIDControl(YawAverage);
​             if(YawCorrectionSpeed<-2)
​             {
​             MotorCalib(1, -YawCorrectionSpeed);//
​             MotorCalib(0, YawCorrectionSpeed);
​             }
​             if(YawCorrectionSpeed>2)
​             {
​             MotorCalib(0, YawCorrectionSpeed);//
​             MotorCalib(1, -YawCorrectionSpeed);
​             }
​         }    

//小车控制程序
void StartTaskYawCorrect(void const * argument)
{
 /* USER CODE BEGIN StartTaskYawCorrect */

 /* Infinite loop */

​    CalculateAverage();//更改两个全局变量PitchAverage和YawAverage（平均滤波），并释放MotorSpdChange信标
​    osSemaphoreWait(myBinarySem_MotorSpdChangeHandle,osWaitForever);
​    YawPIDControl(YawAverage);//释放Yaw信标并得到全局变量YawCorrectionSpeed
​    int err = YawCorrectionSpeed;
 	 int array=0;
​    int brray=0;
​    for(;;)
  {
​         while (err < -2)//在【-，-2】，【2，+】这个范围内Yaw调整，还在下方
​                      {
​                          MotorCalib(1, -YawCorrectionSpeed);
​                          MotorCalib(0, YawCorrectionSpeed);
​                          CalculateAverage();//更改两个全局变量PitchAverage和YawAverage（平均滤波），并释放MotorSpdChange信标
​                          YawPIDControl(YawAverage);//释放Yaw信标并得到全局变量YawCorrectionSpeed
​                          err = YawCorrectionSpeed;
​                          if (err < 5 && err > -5)//Yaw在这个【-5，5】调整Pitch，不调Yaw，不能动SPEED=0
​                          {
​                               PitchPIDControl(PitchAverage);// 更改全局变量PitchError和SPEED
​                               MotorCalib(1, SPEED);//SPEED来源于PitchPIDControl，只根据Pitch变，不知道什么意思
​                               MotorCalib(0, SPEED);
​                               break;
​                          }
​                      }

 

​         while (err > 2)//
​                      {
​                          MotorCalib(0, YawCorrectionSpeed);
​                          MotorCalib(1, -YawCorrectionSpeed);
​                          CalculateAverage();//更改两个全局变量PitchAverage和YawnAverage（平均滤波），并释放MotorSpdChange信标
​                          YawPIDControl(YawAverage);//释放Yaw信标并得到全局变量YawCorrectionSpeed
​                          err = YawCorrectionSpeed;
​                          if (err < 5 && err > -5)//5->4 Yaw在这个【-5，5】调整Pitch，不调Yaw
​                          {
​                               PitchPIDControl(PitchAverage);// 更改全局变量PitchError和SPEED
​                               MotorCalib(0, SPEED);
​                               MotorCalib(1, SPEED);
​                               break;
​                          }
​                      }

​         while (err >= -2 && err <= 2)//Yaw在【-2，2】内往前走
​             {
​             while(PitchError<=3 && PitchError>=-3 && flag==0)//Pitch在【-，3】内，即在地面上，flag让这一段全局只执行一次
​             {
​                  MotorCalib(1,-p);//
​                  MotorCalib(0,-p);
​                  osDelay(6000);//8秒走到顶，距离=8000*100mm
​                  CalculateAverage();
​                  PitchPIDControl(PitchAverage);
​             }
​             if(PitchError>3 || PitchError<-3)
​             {
​                  flag=1;
​             }
​             CalculateAverage();//更改两个全局变量PitchAverage和YawnAverage（平均滤波），并释放MotorSpdChange信标
​             YawPIDControl(YawAverage);//释放Yaw信标并得到全局变量YawCorrectionSpeed
​             err = YawCorrectionSpeed;
​             if (err < 5 && err > -5)//Yaw在【-5，5】内调整Pitch，不调Yaw
​             {
​                  PitchPIDControl(PitchAverage);// 更改全局变量PitchError和SPEED
​                  if(PitchError<=1.6 && PitchError >=-1.6)
​                  {
​                      array++;
​                      MotorCalib(0, 0);
​                      MotorCalib(1, 0);
​                      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
​                      /*----------------------------------------------------*/
​                      if(array>=150)
​                      {
​                          HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
​                          while(brray<=12000000)
​                          {
​                          MotorCalib(0, 2*p);
​                          MotorCalib(1, 2*p);
​                               brray++;
​                          }
​                          while(PitchError>1.5 && PitchError <=-1.5)
​                          {    
​                               MotorCalib(0, 2*p);
​                           MotorCalib(1, 2*p);
​                          }
​                          while(1)
​                          {
​                                       MotorCalib(0, 0);
​                                       MotorCalib(1, 0);
​                          }
​                      }

​                      //UART2SendIntegerVariable(array);
​                  }
​                  else
​                  {
​                      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
​                      MotorCalib(0, SPEED);
​                      MotorCalib(1, SPEED);
​                      CalculateAverage();//更改两个全局变量PitchAverage和YawnAverage（平均滤波），并释放MotorSpdChange信标
​                      array=0;
​                      break;
​                  }
​             }
​             }

   osDelay(1);
  }
```

在调试过程中，我们出现了一系列问题。

1. 串口传输数据乱码或数据传输丢失的问题，考虑到程序时序的配合，我们通过适当延长传输过后的时间延迟解决了问题。
2. 小车开启电源后始终后退不随着角度变化改变电机速度的问题，通过串口通讯，我们逐步排查，发现不是传感器的问题，最终锁定是控制程序中电机的初值设置错误，并随之发现电机控制主程序存在有未经死区矫正的电机控制问题。我们将所有对电机控制的环节都改正为有死区矫正的函数，最终实现了正确的功能。
3. 小车在调整Pitch时无法及时注意到Yaw角发生的偏差导致小车落下跷跷板的问题。我们单独编写了一个电机控制进程，每四秒检查一次Yaw角的情况。该进程只会在小车到达翘翘板上并已经开始调节Pitch角后开启，避免了影响小车初始的Yaw角调整过程，加快小车运行的速度。
4. 小车在走上跷跷板的过程中不断调整Yaw角导致行进速度缓慢的问题。我们修改了程序，小车在对齐跷跷板后，将会直接前进6秒，大大缩短了小车到达平衡位置的时间。
5. 在解决这些问题的过程中，我们也积累了一些调试的经验，例如：在需要检查是否运行的代码后加上点灯语句。若程序正常运行将观察到单片机上自带小灯的闪烁。这大大加快了我们的调试速度与判断的准确度。

三、控制系统的测试与分析

3.1测试方法

​     测试分成两部分，第一部分在地面观察小车的旋转和直线运动情况并进行调整，第二部分在跷跷板上观察小车的平衡情况并进行调整。

3.1.2地面测试

​    将小车按任意朝向摆放，观察小车停止旋转后的朝向情况，然后将小车放在地板边界线上，观察小车旋转后沿直线的偏航情况。多次调整PID算法中的参数值，选出合适的参数后进行下部分测试

3.1.3跷跷板测试

  小车的调试过程需要进行不断的检测。在调试传感器的期间经常会出现重复开机后数据不一致的问题，这时首先就要采用万用表检测正负极的电压是否满足传感器的要求。排除电压的干扰后，我们采用串口通讯，在电脑上不断查看传感器得到的Yaw角和Pitch角的数据。结合小车在跷跷板上的反应，并进行不断地调整。具体的测试过程如3.2所示。

3.2测试数据与现象

对于俯仰角参数的测试，我们首先调整Kp的值(Kp1与Kp2分别代表两套PID参数的值)，当Kp1=0.8=Kp2时，在小车接近平衡点时反应过大导致不能收敛，响应过快，说明Kp偏大 ，经过多次尝试，发现当Kp1=0.8，Kp2=0.7时，可以实现跷跷板一端刚刚着地，小车便运动调整俯仰角，达到临界震荡状态，接着我们开始调节Kd。当Kd1=3.8，Kd2=3.2时，小车响应过慢，不能很好的预测到俯仰角变化趋势，需要继续调大；当Kd1=4.2，Kd2=3.2时，小车第一次实现平衡，但两端存在较大高度差，于是再给一个较小的Ki值，消除静态误差；当Ki1=0.0015,Ki2=0.0005时，两端仍然存在一定的高度差；最终当Ki1=0.0015,Ki2=0.0015时，小车可较完美的完成平衡要求。

对于航向角参数的测试，我们同样使用上述方法，最终我们选定航向角的参数为Kp=1.0，Ki=0.2,Kd=1.2。   

![IMG_256](https://github.com/dragonundertheworld/module-robot/blob/main/img/test_and_record.jpg)

图3-1 测试过程的数据及现象记录

表3-1 测试数据及现象记录表

| **PID****参数** | **err>=2 \|\| err<=-2(****参数1)** | **err>-2&&err<2(****参数2)**   | **现象**                                         |
| --------------- | ---------------------------------- | ------------------------------ | ------------------------------------------------ |
| Kp/Ki/Kd(Yaw)   | 1/0.2/0.5                          | 未能很好预测变化趋势，响应偏慢 |                                                  |
| Kp/Ki/Kd(Yaw)   | 1/0.2/0.8                          | 响应变快，但超调仍过大         |                                                  |
| Kp/Ki/Kd(Yaw)   | 1/0.2/1.2                          | 快速响应，符合要求             |                                                  |
| Kp/Ki/Kd(Pitch) | 0.8/0.0015/3.8                     | 0.8/0.0005/3.2                 | 未能很好预测变化趋势，且缓慢时反映过大，容易超调 |
| Kp/Ki/Kd(Pitch) | 0.8/0.0015/4.2                     | 0.7/0.0015/3.2                 | 快速响应，符合要求                               |

 

表3-2 其他测试数据表

|       | 放大倍数    | 大范围     | 小范围 |
| ----- | ----------- | ---------- | ------ |
| Yaw   | 1->10       | [-4,4]     | [-2,2] |
| Pitch | 10->100->10 | [-1.5,1.5] |        |

 

3.3结果分析

我们的小车虽然已经能够完成平衡的任务，但是在以下的一些方面仍在存在不足。

1、我们在实验中采用的滤波算法并不够完美，由于采用均值滤波的方法，难以避免偶尔仍然会出现较大的误差，如果能够采用FIR滤波的方法会更加精准。

2、当小车在跷跷板上运动时，由于Pitch角发生改变，Yaw角也总是会随之发生一定程度的改变。这种情况有时会造成小车在跷跷板上出现较大的偏角，非常容易掉下跷跷板。使用磁力计传感器测得的数据对小车的Yaw角进行纠正可以改善这种问题。

航向矫正过程主要代码如下：

![IMG_256](https://github.com/dragonundertheworld/module-robot/blob/main/img/code_of_yaw_calibration.png)

图 3-2 航向矫正代码

小车的后退程序存在不足，应该对程序逻辑进一步的修改，以使得小车在后退时能够及时矫正自己的航向角，能够检测到当前的Pitch角度并做到在桌面及时停止。

四、总结与心得

屈茂林：

通过梁老师的这门模块化机器人课程，我学到了很多，它将之前的理论课程结合在一起，使我明白了不同课之间的关联作用，加深了课程上学到的理论知识，进一步将知识转变为能力，学会了使用stm32软件，了解了stm32单片机和GY953 九轴传感器等元件等元件。由于之前没有接触过这些元件，最开始是通过学习老师的资料才有所初步了解和掌握。这次课程加深了我对于机器人设计的了解，非常感谢老师的指导。

曹知寒：

我在这次实验中习得了新的知识，提升了能力与思维，感受良多。

1.知识的补充

###### 机械知识

知道了轮子挑选指标

知道了机架的连接方式

认识了各种连接件并了解了各自适合什么样的机械结构

###### 电路知识

熟悉了传感器的各性能指标以及使用方法

熟悉了控制器的工作原理和性能指标

熟悉了通信原理

熟悉了PID控制原理

熟悉了电源的原理

熟悉了舵机的工作原理和特性

2. 能力的提升

###### 动手安装机械结构的能力

能够更加熟练地使用各种工具如螺丝刀、内六角扳手

掌握了一些安装机械结构的小技巧

知道了安装机械结构存在先后顺序，不可颠倒

###### 动手搭建电路系统的能力

尽量挑选长短一致的杜邦线，否则容易缠绕或松动，导致接触不良

电路搭建时走线要合理，否则容易缠绕或松动，导致接触不良

###### 编程能力

使用CubeMx生成配置、函数初始化等

进程与分模块的思想，用os让各模块不相互影响

使用Keil调试的能力

###### 排查问题的能力

从硬件向软件逐个按运行逻辑排查，排查程序时可以使用点灯程序看运行到哪一步



##### 思维的提升

###### 整体思维的提升

在考虑怎么设计与搭建小车时要从总体考虑，不能片面

做决定时要果断冷静，不能因为新的有可能更好的方案而放弃了已有的接近成功的方案，也不能在方案之间徘徊不定，这样浪费时间

要记录每一次的迭代以及数据和图像，便于后续改进、反思与总结

###### 模块化思维的提升

严格区分模块，这样也便于排查问题与调整程序

##### 我的感受

这次的实验很爽，从一开始的完全不懂到最后的通宵改进，最终小车的测试效果不错，基本完成了基础要求。细想一下，通宵那晚效率太低，之后的实验中，都要记得这样的教训：每一次尝试都要有理有据，记录每一次的对应现象进行分析，盲目的调整方案，只能事倍功半，另外，要学会适时的寻求指导与调整方向，多向老师和同学学习，不要闭门造车。

本次课程收获很多，衷心感谢老师的指导与帮助！我也从中学到了很多。

苟栩宁：

通过本次课程的学习，我受益匪浅。在整个项目的开展过程中，不但锻炼了我的编程能力、思维能力、合作能力，也大大提高了我的抗压能力。我第一次完整的完成了一个软件类项目的开发任务，并且第一次承担了部分代码编写的重要工作。初次接触这个课程时，我对工程中很多概念是只停留在概念层面上或者是模糊的，例如：避免外界的干扰、参数的调整、程序模块的规划和设计等等。

在代码的编写中，我第一次接触到操作系统的实际应用，从中收获到很多全新的知识。我熟悉了操作系统的基本概念，练习了对操作系统的应用，理解到了操作系统对于复杂工程的重要意义。整个程序的框架设计也让我的整体思维更加严密，以后接触这样的复杂项目时再也不会像无头苍蝇一样没有明确的方向。

在实验的过程中，我学到了一款新的传感器——九轴传感器，并对电磁干扰，滤波等概念有了一个清晰的认识，这将为我以后的研究打下基础。同时，代码的编写和调试过程中，我积累了很多经验，这大大提高了我c语言的编写能力。

本次实验中所使用的PID控制算法，是我最大的收获之一。对于工程控制中的概念不再仅仅停留在书本的层面，我也第一次感受到了工程实际应用中调节系统参数的过程，并对kp、kd、ki这三个参数有了深刻的理解与认识。调参过程与我之前的想象有很大不同，因为参数的调节很多时候并没有一个标准答案。而是需要我们合理的结合书本上的知识和概念，在不断的修改与观测中到达理想的效果，这个过程是十分辛苦的，有时候第一次实验成功了，过一会儿却是截然相反的结果。为了让小车达到最良好，最稳定的效果，调参的过程和代码完善的工作占用了我们大量的时间。印象最为深刻的是验收的头一天的下午4点到第二天早上的7点半，我基本上一分钟也没有合眼，在实验室里熬了一个通宵。这也是我第一次为了完成一个项目而不得不通宵，但是看到最后小车能够正常稳定的运行，一切的努力都是值得的。

最后，衷心感谢老师对我们的帮助和指导！
