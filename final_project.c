#include<stdio.h>
#include<time.h>
#include <bcm2835.h>
#define  trigger_pin RPI_GPIO_P1_23
#define echo_pin RPI_GPIO_P1_24
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
		for(l = 0; l < 6000; l++)
		{
		}
}
void send_trigger_pulse()
{
    bcm2835_gpio_write(trigger_pin, HIGH);
    delay_ms(1);//delay 1ms
    bcm2835_gpio_write(trigger_pin, LOW);
}
void wait_for_echo(bool value, int timeout)
{
    int count = timeout;
    while(bcm2835_gpio_read(echo_pin) != value && count > 0)
        count = count - 1;
}
double get_distance()
{
    send_trigger_pulse();
    wait_for_echo(True, 5000);
		clock_t start,end;
    start=clock();
    wait_for_echo(False, 5000);
    end=clock();
    double diff= start-end;
    double distance_cm=diff*340*100/2;
    return distance_cm;
}

int main(void) 
{
				bcm2835_gpio_fsel(trigger_pin, BCM2835_GPIO_FSEL_OUTP);	//set output pin
				bcm2835_gpio_fsel(echo_pin, BCM2835_GPIO_FSEL_INPT);	////set input pin
        while(1)
				{
					printf("cm=%f" , get_distance());
					delay_ms(1000);//delay 1000ms=1sec
				}
					
        return 0;
}
