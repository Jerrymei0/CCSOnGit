/* *
 * PWM控制输出电压，然后ADS118采样电压值，再经过PID调节输出准确的电压值。
 * 以下各种函数目前只是模板，内容并不是这样。2020_08_24
 * 显示：可
 * 按键单独测试的时候是好的，但是加到一起后，就变得特别快（因为频率快
 * 不能用4MHz和12MHz的时钟————单片机的问题
 * __delay_cycles(20000000);//延时5S？
 * * * */
#include <msp430f6638.h>

#include "pid_delta.h"
#include "q_ADS1118.h"


//函数声明
void initPWM(void);
void initPara();
float getVoltage();
void pidAdjust(float in_voltage);
void changePWM(int duty_value);
void DispFloatat(unsigned char x,unsigned char y,float dat,unsigned char len1,unsigned char len2 );
void my_key();
void SMCLK_XT2_4Mhz(void);
void suprotect(float vol);

//变量声明
double duty=0;//占空比
PID_DELTA pid;        //声明pid结构体变量
double dealtV=0;  //pid误差量
unsigned int AD_bit; //定义读取AD转换数值
float True_voltage=0;
int key_value;
double num=0;//按键所得数值

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    SMCLK_XT2_4Mhz();
//    SetClock_MCLK12MHZ_SMCLK12MHZ_ACLK32_768K();//12MHz
//    UCSCTL5|=DIVS_2;//使用USC统一时钟系统进行预分频，将SMCLK进行4分频——————1M

    initPWM();
    initPara();//初始值
    OLED_Init();/*init OLED*/
    OLED_Clear(); /*clear OLED screen*/
    init_key();
    OLED_ShowString(0,0, "Voltage:");
    OLED_ShowString(0,2, "current:");
    OLED_ShowString(0,4, "set:");
    while(1)
    {
        True_voltage=getVoltage();
        pidAdjust(True_voltage);
        my_key();
        DispFloatat(72,4,pid.setPoint,2,3);//显示
        DispFloatat(16,6,pid.Proportion,4,1);//显示
        DispFloatat(96,6,pid.Integral,2,1);//显示
    }
}

/******************************AD值读取函数**********************************/
float getVoltage()//可
{
    //测两个的时候为什么是反的
    unsigned int Value,Value2;
    float Voltage;
    float Voltage2;
    float current;
    Value = Write_SIP(0xf38b);           //AD数值     Conversion Register
    Voltage=change_voltage(Value,4.096);
    Voltage=Voltage*12;
    DispFloatat(72,0,Voltage,2,3);//显示电压值
    usleep(50);
    Value2 = Write_SIP(0xe38b);           //AD数值     Conversion Register
    Voltage2=change_voltage(Value2,4.096);
    current=Voltage2/0.65;
    DispFloatat(72,2,current,1,3);//显示电流值
//    suprotect(Voltage2);

    return Voltage;
}
/*****************************过流保护*********************************/
void suprotect(float vol)
{
    if(vol>1.625)
        {
            P8OUT |= BIT4;        //置高
            __delay_cycles(20000000);//延时5S？
            P8OUT &= ~BIT4;        //置高
        }

}
/*****************************PID控制恒压*********************************/
void pidAdjust(float in_voltage)
{
  dealtV = PidDeltaCal(&pid,in_voltage);  //返回误差增量
  if((duty + dealtV) > 40)
  {
      duty = 40;
    changePWM(duty);                      //生效控制
  }
  else if((duty + dealtV) < 0)
  {
      duty = 0;
    changePWM(duty);                      //生效控制
  }else{
      duty = duty + dealtV;                 //修正占空比
    changePWM(duty);                      //生效控制
  }
}

/****************************改变PWM占空比*********************************/
void changePWM(int duty_value)//可
{
    TA0CCR1 = duty_value;
    TA0CCR2 = duty_value+1;//保证两路PWM波除了死区之外同步
}
/****************************PWM初始化输出*********************************/
void initPWM(void)//不可
{
  P1DIR |= BIT2;
  P1SEL |= BIT2;        //选择TA.1功能

  P1DIR |= BIT3;
  P1SEL |= BIT3;        //选择TA.1功能

  TA0CTL |=TASSEL_2 + MC_3 + TACLR;//配置A0计数器,时钟源SMCLK，上升模式，同时清除计数器//*配置计数器
  //TASSEL_2选择了SMCLK，MC_1计数模式，，最后清零TACLR
  TA0CCTL0 = /*OUTMOD_7+*/  CCIE;//捕获比较寄存器0输出，输出模式为2，同时使能定时器中断（CCR0单源中断），CCIE捕获比较寄存器的使能配置
  TA0CCR0 = 100;//捕获比较寄存器,设置定时器中断频率20K
  TA0CCTL1 |= OUTMOD_2; // TD0CCR1, Reset/Set
  TA0CCR1 = 50;             //占空比CCR1/CCR0

  TA0CCTL2 |= OUTMOD_6; // TD0CCR2, Reset/Set
  TA0CCR2 = 51;             //占空比CCR2/CCR0
}

/****************************设置初始值*********************************/
void initPara()
{
  duty = 25;    //测试值？不确定
  pid.setPoint = 2.5;   ////设定值，不确定
  adjust_pid(&pid, 5000, 0, 0);//调整PID系数
  adjust_pid_limit(&pid, -10, 10);//设定PID误差增量的限制范围
  ADS1118_GPIO_Init();  //配置管脚（模拟SPI，加上Vcc、GND需要6根线，除去这俩需要4根线，故需要管脚配置）

//  P8DIR |= BIT4;    //过流保护管脚
}

/****************************浮点数显示函数********************************/
//dat:数据    len1:整数的位数    len2:小数的位数
const long numtab[]={
  1,10,100,1000,10000,100000,1000000,10000000,100000000,1000000000,10000000000};
char a;
void DispFloatat(unsigned char x,unsigned char y,float dat,unsigned char len1,unsigned char len2 )
{
    int dat1,dat2;
    dat1=(int)dat;
    dat2=(int)((dat-dat1)*numtab[len2]);
    OLED_ShowNum(x,y,dat1,len1,16);
    OLED_ShowString(x+8*len1,y, ".");
    if(dat2/numtab[len2-1]==0)
        {
            if(len2>2)
            {
                if(dat2/numtab[len2-2]==0){
                    OLED_ShowString(x+8*len1+8,y,"0");
                    OLED_ShowString(x+8*len1+16,y,"0");
                    OLED_ShowNum(x+8*len1+24,y,dat2,len2-2,16);
                }else{
                    OLED_ShowString(x+8*len1+8,y,"0");
                    OLED_ShowNum(x+8*len1+16,y,dat2,len2-1,16);
                }

            }  else{
                    OLED_ShowString(x+8*len1+8,y,"0");
                    OLED_ShowNum(x+8*len1+16,y,dat2,len2-1,16);
                }
        }
    else
        OLED_ShowNum(x+8*len1+8,y,dat2,len2,16);

}
/****************************按键函数********************************/
//按键是只需要显示两位对吧？
int i=0;
void my_key()
{

    key_value= key();   /*scan Array_button, get the key value*/
            if(key_value!=0)
            {
                    if(i>1)//判断是0还是1
                        {
                           i=0;
                           if(num<=36.0&&num>=30.0)
                               pid.setPoint=num/12;//设定期望电压值
                           OLED_ShowString(0,6, "    ");
                           num=0;
                        }
                    switch(key_value)
                    {
                        case(1):
                               OLED_ShowNum(8*i,6,1,1,16);  /*show the key value*/
                              switch(i)
                               {
                                   case 0:num+=10;break;
                                   case 1:num+=1;break;
                                   default:break;
                               }
                              i++;
                              key_value=0;
                              break;
                      case(2):
                          OLED_ShowNum(8*i,6,2,1,16);  /*show the key value*/
                          switch(i)
                           {
                               case 0:num+=20;break;
                               case 1:num+=2;break;
                               default:break;
                           }
                          i++;
                          key_value=0;
                          break;
                      case(3):
                            OLED_ShowNum(8*i,6,3,1,16);  /*show the key value*/
                            switch(i)
                             {
                                 case 0:num+=30;break;
                                 case 1:num+=3;break;
                                 default:break;
                             }
                            i++;
                            key_value=0;
                            break;
                      case(4)://A
                              if(pid.Proportion>0)
                                  pid.Proportion-=1;
//                            if(pid.setPoint<36.0)
//                                pid.setPoint+=1;//步进设定期望电压值
                            key_value=0;
                            break;
                      case(5):
                        OLED_ShowNum(8*i,6,4,1,16);  /*show the key value*/
                        switch(i)
                         {
                             case 0:num+=40;break;
                             case 1:num+=4;break;
                             default:break;
                         }
                        i++;
                        key_value=0;
                        break;
                      case(6):
                          OLED_ShowNum(8*i,6,5,1,16);  /*show the key value*/
                          switch(i)
                           {
                               case 0:num+=50;break;
                               case 1:num+=5;break;
                               default:break;
                           }
                          i++;
                          key_value=0;
                          break;
                      case(7):
                            OLED_ShowNum(8*i,6,6,1,16);  /*show the key value*/
                            switch(i)
                             {
                                 case 0:num+=60;break;
                                 case 1:num+=6;break;
                                 default:break;
                             }
                            i++;
                            key_value=0;
                            break;
                        case(8)://B
//                              if(pid.setPoint>30.0)
//                                  pid.setPoint-=1;//设定期望电压值
                              if( pid.Integral>0)
                                  pid.Integral-=1;//调I
                              key_value=0;
                              break;
                      case(9):
                          OLED_ShowNum(8*i,6,7,1,16);  /*show the key value*/
                          switch(i)
                           {
                               case 0:num+=70;break;
                               case 1:num+=7;break;
                               default:break;
                           }
                          i++;
                          key_value=0;
                          break;
                      case(10):
                            OLED_ShowNum(8*i,6,8,1,16);  /*show the key value*/
                            switch(i)
                             {
                                 case 0:num+=80;break;
                                 case 1:num+=8;break;
                                 default:break;
                             }
                            i++;
                            key_value=0;
                            break;
                      case(11):
                            OLED_ShowNum(8*i,6,9,1,16);  /*show the key value*/
                            switch(i)
                              {
                                   case 0:num+=90;break;
                                   case 1:num+=9;break;
                                   default:break;
                              }
                            i++;
                            key_value=0;
                            break;
                      case(12)://C
                              pid.Proportion+=1;//调P
                              key_value=0;
                              break;
                      case(13)://*转小数输入

                          break;
                      case(14)://0
                            OLED_ShowNum(8*i,6,0,1,16);  /*show the key value*/
                            switch(i)
                             {
                                 case 0:num+=0;break;
                                 case 1:num+=0;break;
                                 default:break;
                             }
                            i++;
                            key_value=0;
                            break;
                      case(15)://#确定
//                            pid.setPoint=num;//设定期望电压值
                            key_value=0;
                            break;
                      case(16)://D
                            pid.Integral+=1;//调I
                            key_value=0;
                            break;
                      default:break;
                    }
            }
}
void SMCLK_XT2_4Mhz(void)
{

    P7SEL |= BIT2+BIT3;                       // Port select XT2   配置管脚为时钟输出
    UCSCTL6 &= ~XT2OFF;                         // Enable XT2         打开XT2振荡器
    UCSCTL6 &= ~XT2OFF + XT2DRIVE_1;          // Enable XT2       XT2 oscillator operating range is 8 MHz to 16 MHz.
    UCSCTL3 |= SELREF_2;                      // FLLref = REFO      加载电阻？？？
                                              // Since LFXT1 is not used,
                                              // sourcing FLL with LFXT1 can cause
                                              // XT1OFFG flag to set
    UCSCTL4 |= SELA_2;                        // ACLK=REFO,SMCLK=DCO,MCLK=DCO

    // Loop until XT1,XT2 & DCO stabilizes - in this case loop until XT2 settles
    do
    {
      UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + XT1HFOFFG + DCOFFG);
                                              // Clear XT2,XT1,DCO fault flags清除振荡器XT2，XT1，DCO失效标志
      SFRIFG1 &= ~OFIFG;                      // Clear fault flags  清除振荡器失效标志
    }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag     判断所有晶振是否起振

    UCSCTL6 &= ~XT2DRIVE0;                    // Decrease XT2 Drive according to
                                              // expected frequency   根据晶振频率减小XT2驱动电流以降低功耗
    UCSCTL4 |= SELS_5 + SELM_5;               // 选择SMCLK=MCLK=XT2  选择时钟源XTWCLK(没有的话就是DCOCLKDIV)
}
