#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "math.h"

#define pi 3.1415926
#define digit 100000
#define period 7500  // 10KHz对应时钟数，TBCLK = SYSCLKOUT
//#define period 60000  // 10KHz对应时钟数，TBCLK = SYSCLKOUT(for test)
#define M 0.8  // 调制度
int period_count = 0;  // 载波周期数
int Tinv[3] = {0, 0, 0};  // 三相对应比较值
int last[3];  // 上周期Tinv值(for test)
double Dm = 0, Dn = 0, D0 = 0;  // 占空比

void ePWMSetup(void);
double roundn(double);  // 截断小数点后位数
interrupt void epwm1_timer_isr(void);

int main()
{
   InitSysCtrl();

   DINT;

   InitPieCtrl();

   IER = 0x0000;
   IFR = 0x0000;

   InitPieVectTable();

   EALLOW;
   PieVectTable.EPWM1_INT = &epwm1_timer_isr;  // ePWM1中断函数入口
   EDIS;

   ePWMSetup();
	
   IER |= M_INT3;  // enable ePWM1 CPU_interrupt
   PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // enable ePWM1 pie_interrupt

   EINT;   // 总中断 INTM 使能
   ERTM;   // Enable Global realtime interrupt DBGM

   int i;
   for(; ;)
   {
	   asm("          NOP");
	   for(i=1;i<=10;i++)
	   {}
   }

   return 0;
}

void ePWMSetup(void)
{

   EALLOW;

   GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1; // GPIO 初始化
   GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
   GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
   GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
   GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
   GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;

/*   GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;
   GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;
   GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;
   GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;
   GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;
   GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;*/

   GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;
   GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;
   GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;
   GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;
   GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;
   GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;

   GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;
   GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;
   GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;
   GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;
   GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;
   GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;

   EDIS;

   GpioDataRegs.GPASET.bit.GPIO6 = 1;
   GpioDataRegs.GPASET.bit.GPIO7 = 1;
   GpioDataRegs.GPASET.bit.GPIO8 = 1;
   GpioDataRegs.GPASET.bit.GPIO9 = 1;
   GpioDataRegs.GPASET.bit.GPIO10 = 1;
   GpioDataRegs.GPASET.bit.GPIO11 = 1;

   // ----------------EPwm1---------------------
   EPwm1Regs.TBPRD = period;
   EPwm1Regs.TBPHS.half.TBPHS = 0;  // 时基周期寄存器
   EPwm1Regs.TBCTR = 0;  // 时基计数寄存器置零
   EPwm1Regs.TBCTL.bit.PHSDIR = TB_UP;
   EPwm1Regs.TBCTL.bit.CLKDIV = 0;
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;
   EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;

   EPwm1Regs.CMPA.half.CMPA = period / 2; // duty_cycle = 0.5
   EPwm1Regs.CMPB = 0;
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;
   EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;

   EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
   EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;  // A不翻转，B翻转
   EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
   EPwm1Regs.DBRED = 800; // Deadzone
   EPwm1Regs.DBFED = 800;

   EPwm1Regs.ETSEL.bit.INTEN = 1;  // 使能ePWM中断
   EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;
   EPwm1Regs.ETPS.all = 0x01; // interrupt on first event
   EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;

   // ----------------EPwm2---------------------
   EPwm2Regs.TBPRD = period;
   EPwm2Regs.TBPHS.half.TBPHS = 0;  // 时基周期寄存器
   EPwm2Regs.TBCTR = 0;  // 时基计数寄存器置零
   EPwm2Regs.TBCTL.bit.PHSDIR = TB_UP;
   EPwm2Regs.TBCTL.bit.CLKDIV = 0;
   EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;
   EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;

   EPwm2Regs.CMPA.half.CMPA = period / 2; // duty_cycle = 0.5
   EPwm2Regs.CMPB = 0;
   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

   EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;
   EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;

   EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
   EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;  // A不翻转，B翻转
   EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
   EPwm2Regs.DBRED = 800; // Deadzone
   EPwm2Regs.DBFED = 800;

   // ----------------EPwm3---------------------
   EPwm3Regs.TBPRD = period;
   EPwm3Regs.TBPHS.half.TBPHS = 0;  // 时基周期寄存器
   EPwm3Regs.TBCTR = 0;  // 时基计数寄存器置零
   EPwm3Regs.TBCTL.bit.PHSDIR = TB_UP;
   EPwm3Regs.TBCTL.bit.CLKDIV = 0;
   EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;
   EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
   EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;

   EPwm3Regs.CMPA.half.CMPA = period / 2; // duty_cycle = 0.5
   EPwm3Regs.CMPB = 0;
   EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

   EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;
   EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;

   EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
   EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;  // A不翻转，B翻转
   EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
   EPwm3Regs.DBRED = 800; // Deadzone
   EPwm3Regs.DBFED = 800;

   // ----------------EPwm4---------------------
   EPwm4Regs.TBPRD = period;
   EPwm4Regs.TBPHS.half.TBPHS = 0;  // 时基周期寄存器
   EPwm4Regs.TBCTR = 0;  // 时基计数寄存器置零
   EPwm4Regs.TBCTL.bit.PHSDIR = TB_UP;
   EPwm4Regs.TBCTL.bit.CLKDIV = 0;
   EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0;
   EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
   EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;

   EPwm4Regs.CMPA.half.CMPA = period / 2; // duty_cycle = 0.5
   EPwm4Regs.CMPB = period / 2;
   EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

   EPwm4Regs.AQCTLA.bit.CAU = AQ_SET;
   EPwm4Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
   EPwm4Regs.AQCTLB.bit.CBU = AQ_SET;
   EPwm4Regs.AQCTLB.bit.ZRO = AQ_CLEAR;

   EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;
   EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;  // A不翻转，B翻转
   EPwm4Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;

   // ----------------EPwm5---------------------
   EPwm5Regs.TBPRD = period;
   EPwm5Regs.TBPHS.half.TBPHS = 0;  // 时基周期寄存器
   EPwm5Regs.TBCTR = 0;  // 时基计数寄存器置零
   EPwm5Regs.TBCTL.bit.PHSDIR = TB_UP;
   EPwm5Regs.TBCTL.bit.CLKDIV = 0;
   EPwm5Regs.TBCTL.bit.HSPCLKDIV = 0;
   EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
   EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;

   EPwm5Regs.CMPA.half.CMPA = period / 2; // duty_cycle = 0.5
   EPwm5Regs.CMPB = period / 2;
   EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

   EPwm5Regs.AQCTLA.bit.CAU = AQ_SET;
   EPwm5Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
   EPwm5Regs.AQCTLB.bit.CBU = AQ_SET;
   EPwm5Regs.AQCTLB.bit.ZRO = AQ_CLEAR;

   EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;
   EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;  // A不翻转，B翻转
   EPwm5Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;

   // ----------------EPwm6---------------------
   EPwm6Regs.TBPRD = period;
   EPwm6Regs.TBPHS.half.TBPHS = 0;  // 时基周期寄存器
   EPwm6Regs.TBCTR = 0;  // 时基计数寄存器置零
   EPwm6Regs.TBCTL.bit.PHSDIR = TB_UP;
   EPwm6Regs.TBCTL.bit.CLKDIV = 0;
   EPwm6Regs.TBCTL.bit.HSPCLKDIV = 0;
   EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
   EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;

   EPwm6Regs.CMPA.half.CMPA = period / 2; // duty_cycle = 0.5
   EPwm6Regs.CMPB = period / 2;
   EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

   EPwm6Regs.AQCTLA.bit.CAU = AQ_SET;
   EPwm6Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
   EPwm6Regs.AQCTLB.bit.CBU = AQ_SET;
   EPwm6Regs.AQCTLB.bit.ZRO = AQ_CLEAR;

   EPwm6Regs.DBCTL.bit.IN_MODE = DBA_ALL;
   EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;  // A不翻转，B翻转
   EPwm6Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;

}

interrupt void epwm1_timer_isr(void)
{
	double Angle = 0;
	double theta = 0;
	int sector = 0;

    Angle = fmod((100 * pi * (period_count / 10000.0)),(2 * pi));
   	theta = fmod(Angle,1/3.0 * pi);
   	sector = floor( Angle / (1/3.0 * pi)) + 1;
   	Dm = M * sin(1/3.0 * pi - theta) / 2.0;
   	Dn = M * sin(theta) / 2.0;
   	D0 = (0.5 - Dm - Dn) / 2.0;
   	Dm = roundn(Dm);
   	Dn = roundn(Dn);
   	D0 = roundn(D0);
   	if (D0 < 0) D0 = 0;

   	switch (sector)
   	{
       case 1:
           Tinv[0] = floor(period * (D0));
           Tinv[1] = floor(period * (D0 + Dm));
           Tinv[2] = floor(period * (D0 + Dm + Dn));
           break;
        case 2:
            Tinv[0] = floor(period * (D0 + Dn));
            Tinv[1] = floor(period * (D0));
            Tinv[2] = floor(period * (D0 + Dm + Dn));
            break;
        case 3:
            Tinv[0] = floor(period * (D0 + Dm + Dn));
            Tinv[1] = floor(period * (D0));
            Tinv[2] = floor(period * (D0 + Dm));
            break;
        case 4:
            Tinv[0] = floor(period * (D0 + Dm + Dn));
            Tinv[1] = floor(period * (D0 + Dn));
            Tinv[2] = floor(period * (D0));
            break;
        case 5:
            Tinv[0] = floor(period * (D0 + Dm));
            Tinv[1] = floor(period * (D0 + Dm + Dn));
            Tinv[2] = floor(period * (D0));
            break;
        case 6:
            Tinv[0] = floor(period * (D0));
            Tinv[1] = floor(period * (D0 + Dm + Dn));
            Tinv[2] = floor(period * (D0 + Dn));
   	}
   	if (Tinv[0] == 0)
   		Tinv[0]++;
   	if(Tinv[0] == period)
   		Tinv[0]--;
   	if (Tinv[1] == 0)
   		Tinv[1]++;
   	if(Tinv[1] == period)
   		Tinv[1]--;
   	if (Tinv[2] == 0)
   		Tinv[2]++;
   	if(Tinv[2] == period)
   		Tinv[2]--;
   	EPwm1Regs.CMPA.half.CMPA = Tinv[0];
   	EPwm2Regs.CMPA.half.CMPA = Tinv[1];
   	EPwm3Regs.CMPA.half.CMPA = Tinv[2];

    period_count++;
    if (period_count == 10000) period_count = 0;

    last[0] = Tinv[0];
    last[1] = Tinv[1];
    last[2] = Tinv[2];

   // Clear INT flag for this timer
    EPwm1Regs.ETCLR.bit.INT = 1;

   // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

double roundn(double input)
{
	double temp;
	temp = input * digit;
	temp = floor(temp);
	temp = temp / digit;
	return temp;
}
