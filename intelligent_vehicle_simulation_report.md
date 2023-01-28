# 模擬車子
## I.	簡介：
我們的目標是將車子整合GPIO連接超音波模組和UART連接超音波模組使車子能偵測與前方物體的距離避免發生碰撞，最後再使用Interrupt整合各個功能模組(超音波模組、陀螺儀模組、加速規模組和馬達電控板)於車體載具上。
## II.	原理探討：
一台車子需具有的功能有：加/減速、前/後/左/右移動、和超音波測距。
超音波模組的原理是車子發射一個電波然後利用電波反射回來的時間推算出車子與物體的距離，公式[1]:
距離=(音波發射與接收時間差 * 聲音速度(340M/S))/2
我們使用US100(GPIO or UART)來實作如圖(一)[1],
![image](https://github.com/Chien-chia-yen/Intelligent-vehicle-simulation/blob/main/pic/pic1.PNG)圖(一) US-100正反圖
[US-100規格][2]：  
* 直流電源：DC5V  
* 靜態電流：小於2mA  
* 電平：1:5V /0:0V  
* 感應角度：15度以內  
* 探測距離：2CM-4.5M  
* 高精度：可達0.5mm  
[1]而且US100具有溫度感測，距離值已經有溫度調校，無需再根據環境溫度對超音波聲速進行校正。  
在這個模組背面有一個Jumper，來控制兩種模式[1]：  
open：將Jumper拔起來，為GPIO 模式(電位觸發模式)  
short：將Jumper套上，此時設定為UART串列通訊模式  
[US-100 pin腳][1]：  
* 1號Pin(VCC)：接VCC電源（範圍2.4V~5.5V）。
* 2號Pin(Trig/TX)：當為UART模式時, 接外部電路UART 的TX 端；為GPIO模式時，接外部電路的Trig端。  
* 3號Pin(Echo/RX)：當為UART模式時, 接外部電路UART 的RX 端；為GPIO模式時，接外部電路的Echo端。  
* 4號Pin(GND)：接外部電路的地。  
* 號Pin(GND)：接外部電路的地。  
## III.	GPIO/UART連結超音波模組:
[材料]：
開發板：ST官方的STM32VL Discovery，圖(二)[3]。  
![image](https://github.com/Chien-chia-yen/Intelligent-vehicle-simulation/blob/main/pic/pic2.PNG)圖(二) STM32VL Discovery  
微控制器：STM32F100R8T6  
超音波模組：US-100超音波模組  
[實驗步驟]：  
步驟1：US-100 VCC接STM32VL Discovery主板的+5V  
步驟2：US-100 Trig/TX接STM32VL Discovery主板的PA9  
步驟3：US-100 Echo/RX接STM32VL Discovery主板的PA10步驟4：US-100 GND接STM32VL Discovery主板的GND  
以上步驟可參考圖(三)[1][3]。  
![image](https://github.com/Chien-chia-yen/Intelligent-vehicle-simulation/blob/main/pic/pic3.jpg)圖(三)US100與STM32VL Discovery接線圖
[工作方式][1]：   
步驟1：Trig(程式碼使用Pin23)接腳發出一個10us以上的高電位(程式碼使用10us)。  
步驟2：等待Echo高電平輸出。  
步驟3：一旦Echo高電平有輸出就開始計時，此時模組會發送8個40khz的方波，並開始自動檢測是否有返回信號。  
步驟4：當偵測到反射訊號時，Echo接腳變為0V低電位。  
步驟5：計算Echo電位從High到LOW的時間，可知超音波來回時間而計算與物體距離。  
P.s：如果加上while(1)迴圈，就可以一直偵測前方移動物體的距離。  
程式碼撰寫方式(參考[1]):
[設定GPIO連接超音波模組]：
<p><pre><code>
/***每隔1sec會去計算一次車子與前方物體的距離***/  
#include<stdio.h>  
#include ”stm32f10x.h”  
#include<time.h>  
#define True 1  
#define False 0  
typedef int bool;  
void delay_ms(uint16_t);  
void send_trigger_pulse(void);  
void wait_for_echo(bool, int);  
double get_distance(void);  

void delay_ms(uint16_t t)  
{  
	 volatile unsigned long l = 0;  
	 for(uint16_t i = 0; i < t; i++)  
		 for(l = 0; l < 6000; l++){}  
 }  
void send_trigger_pulse()  
{  
     GPIOA->ODR |= (1<<9);//set PA9 high  
     delay_ms(0.01);//delay 0.01ms  
     GPIOA->ODR &= (0<<9); //set PA9 low  
}  
void wait_for_echo(bool value, int timeout)
{
    int count = timeout;
    while(GPIOA->ODR |= (1<<10)!= value && count > 0)
        count = count - 1;
	//當 echo pin != value, 持續等待直到時間到(5sec)
}
double get_distance()
{
    send_trigger_pulse();
    wait_for_echo(True, 5000);//等待回復時間不超過5sec
    clock_t start,end;
    start=clock();
    wait_for_echo(False, 5000);
    end=clock();
    double diff= start-end;
    double distance_cm=diff*340*100/2; //來回,故/2
    return distance_cm;
}

int main(void) 
{
RCC->APB2ENR |= 0xFC | (1<<14); //enable GPIO clocks
	
	/******** 設定 PA9 為 output 覆用 (trigger pin)********/
	/******** 設定 PA10 為 input 覆用 (echo pin)********/
	unit_t t=GPIOA->CRH;//get the content of CRH
	t=t&0xFFFFF00F;//clear the configuration bit for PA9 and PA10
	t=t|0x00000430;//configure bit for PA9 as output and PA10 as input 
	GPIOA->CRH=t;//load CRH as the new value
    while(1)
	{
		printf("cm=%f" , get_distance());
		delay_ms(1000);//delay 1000ms=1sec
	}
					
    return 0;  
} 
</code></pre></p>
[設定UART連接超音波模組]：  
<p><pre><code>
/*當輸入”check_distance”時,車子才會測量自己與前方物體的距離,並顯示出來*/
#include<stdio.h>
#include ”stm32f10x.h”
#include<time.h>
#include<string.h>
#define SIZE_OF_CMD_BUF 16
#define True 1
#define False 0
typedef int bool;
void delay_ms(uint16_t);
void send_trigger_pulse(void);
void wait_for_echo(bool, int);
double get_distance(void);
void usart1_sendByte(unsigned char);
uint8_t usart1_recByte();
void usart1_sendStr(char);
void check();
char command1[]="cm=";
static char cmdbuf[SIZE_OF_CMD_BUF];
void delay_ms(uint16_t t)
{
	volatile unsigned long l = 0;
	for(uint16_t i = 0; i < t; i++)
		for(l = 0; l < 6000; l++){}
}
void send_trigger_pulse()
{
    GPIOA->ODR |= (1<<9); //set PA9 high
    delay_ms(0.01);//delay 0.01ms
    GPIOA->ODR &= (0<<9); //set PA9 low
}
void wait_for_echo(bool value, int timeout)
{
    int count = timeout;
    while(GPIOA->ODR |= (1<<10)!= value && count > 0)
        count = count - 1;
}
double get_distance()
{
    send_trigger_pulse();
    wait_for_echo(True, 5000);//等待回復時間不超過5sec
    clock_t start,end;
    start=clock();
    wait_for_echo(False, 5000);
    end=clock();
    double diff= start-end;
    double distance_cm=diff*340*100/2; //來回,故/2
    return distance_cm;
}
void usart1_sendByte(unsigned char c)
{
    USART1->DR = c;
    while((USART1->SR&(1<<6)) == 0); //wait until the TC flag is set
    USART1->SR &= ~(1<<6); //clear TC flag
}
uint8_t usart1_recByte()
{
    while((USART1->SR&(1<<5)) == 0); //wait until the RXNE flag is set
    return USART1->DR;
}
void usart1_sendStr(char str[])
{
	unsigned int i=0;
	for(i=0;i<strlen(str);i++)
		usart1_sendByte(str[i]);
	
}

<code>
void check()//輸入的指令是否為"check_distance",是的話去求與物體的距離
{
	unsigned int i=0;
	char command1[]="check_distance";
	if(strcmp(cmdbuf,command1)==0)
	{
		float f=get_distance();
		char distance[50]; //size of the numbe
    	sprintf(distance, "%g", f); //convert float to string
		usart1_sendStr(command1);
		usart1_sendStr(distance);
}
	for(i=0;i<SIZE_OF_CMD_BUF;i++) //清空buffer
		cmdbuf[i]='\0';
}
int main(void) 
{
	RCC->APB2ENR |= 0xFC | (1<<14);
	/* Enable clocks for GPIO ports and USART1 clock */
//USART1_init
	/******** 設定 PA9 為 output 覆用 ********/
	/******** 設定 PA10 為 input 覆用 ********/
	GPIOA->ODR |= (1<<10);
	GPIOA->CRH=0x444448B4;
//Rx1(pin10)=input with pull-up, Tx1(pin9)=alt.func output
	USART1->CRI=0x200C;
	USART1->BRR=7500;
    while(1)
	{
		uint8_t c = usart3_recByte(); //接收輸入的指令
		check(); //對輸出的指令作相對應的動作
		delay_ms(1000);//delay 1000ms=1sec
	}
					
    return 0;
}
</code></pre></p>
## IV.	實現方式(如何用interrup將各個功能模組整合)
[Interrupt介紹][4]：  
中斷是指處理器接收到外圍硬體或軟體的信號，導致處理器通過一個執行 context switch 並處理該事件。  
[Interrupt處理流程][4]：  
1.	暫停目前 process 執行並保存此 process 當時執行狀況。  
2.	OS 根據 Interrupt ID 查尋 Interrupt vector 並取得 ISR (Interrupt Service Routine) 起始位址。  
3.	ISR 執行。  
4.	執行完成，恢復先前 process 執行狀況。  
5.	回到原先中斷前的執行。  
以上步驟可參考圖(四)[4]。  
![image](https://github.com/Chien-chia-yen/Intelligent-vehicle-simulation/blob/main/pic/pic4.png)圖(四) Interrupt處理流程  
