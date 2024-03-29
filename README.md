# Intelligent-vehicle-simulation
By Chia-Yen Chien

## 目的
將車子整合GPIO連接超音波模組和UART連接超音波模組使車子能偵測與前方物體的距離避免發生碰撞，最後再使用Interrupt整合各個功能模組(超音波模組、陀螺儀模組、加速規模組和馬達電控板)於車體載具。

## 方法
###	超音波測距離：
使用SysTick Interrupt，設定固定時間週期，每當時間到時會跳到SysTick_Handler function，此時如果將function內寫了測輛距離的指令則車子每隔固定時間會去量一次自己與周圍物體的距離。
除此之外，用超音波測距離可以使用I/O Port Interrupt，車子先發出一波(trigger pin=1，echo pin=0)，當此波因為碰撞到物體而反射回來時trigger pin=1，echo pin=1，此時觸發I/O Port interrupt，程式跳去EXTIn_IRQHandler() functin計算間格時間並推出車子與物體的距離。
###	控制智能車行駛方向和加減速：
用鍵盤控制智能車。使用UART interrupt，當收到鍵盤q (結束程式，車子停止)、w (前進)、x (後退)、d (向右轉)、a (向左轉)、s(減速)和z(加速)的指令後，會跳到USART1_IRQHandler function去執行function相對應的指令。

### 以PWM控制步進馬達：
[控制馬達方向]：
腳位設定如下表，當為0時不供電，當為1時受到IN1/IN2及IN3/IN4的值不同來控制轉向。  
[控制馬達速度]：
通過改變單位時間內脈衝的個數可以實現調頻；通過改變佔空比可以實現調壓。佔空比越大，所得到的平均電壓也就越大，幅值也就越大，速度增加；占空比越小，所得到的平均電壓也就越小，幅值也就越小，速度下降。

### 以SPI連接陀螺儀模組
實作(詳細程式碼展示於PDF)
1. include函式庫：#include <MPU6000.h>、#include<stdio.h>  
2. 初始化SPI。  
3. 讀取X、Y、Z軸加速度的值。(回傳資料單位:Gs)  
4. 讀取陀螺儀數據。(回傳資料待為:度/秒)  
###以I2C連接加速規模組
實作(詳細程式碼展示於PDF)  
1.	啟用I2C1 RCC時鐘  
2.	設定 I2C1 的 SDA 與 SCL 腳位PB6 = SCL ，PB7 = SDA  
3.	讀 ADXL345 佔存器中的 x , y, z 的值  
