//maze solving code
//white line on black surface

#define F_CPU 8000000UL
#define Max	500
#define E_Max	1000


#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <string.h>
#include <stdbool.h>
#include "USART_128.h"


int sensor_weight[8] = {-6,-4,-2,0,0,2,4,6};
int error, des_dev, cur_dev,i,m,j,n,x,s[8], h[8],k;
int c=0, mode=0, temp=0, x=0;

float kp=15, kd = 4, ki = 0;
//float kp=25, kd = 10, ki = 0; 10,1

float E_sum = 0 , e_old = 0, pid_out;
int dry_speed=600,test_speed=800;
char path[100]={'S','L','R','R','L','L','L','R','S','L','S','S'};
int pathLength=0;

int readSensor();
void pwm_init();
float err_calc();
float pid(float);
void motorPIDcontrol(int);
void mazeSolve();
void runExtraInch();
void motor_stop();
void mazeEnd();
void goAndTurn(char);
void save_path(char);
int arrShift(int,int);
void simplifyPath();
void got_itR();
void got_itL();
void got_itB();
void led(void);
void runExtraInch1();


int main(void)
{
	DDRD = 0X00;
	DDRC = 0X00;
	DDRB |= 0XFF;
	DDRA |= 0XFF;
	
	pwm_init();
	USART_Init(12,0);
	USART_InterruptEnable(0);
	while(1)
	{
		//USART_Transmitchar('l',0);
		//USART_Transmitchar(0x0d,0);
		//_delay_us(1);
		mazeSolve();
		_delay_ms(10);
	}
}

void pwm_init()
{
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);
	TCCR1B |= (1<<WGM12) | (1<<WGM13) | (1<<CS10);
	ICR1 = 1500;
}

int readSensor()
{
	s[8] = 0;
	k=0;
	for(i=0;i<8;i++)
	{
		if(bit_is_clear(PIND,i))
		{
			s[i] = 1;
		}
		else
		{
			s[i] = 0;
		}
	}
	k = s[7] + s[6]*2 + s[5]*4 + s[4]*8 + s[3]*16 + s[2]*32 + s[1]*64 + s[0]*128; // left most ir s[0]
	return(k);
}

float err_calc()
{
	cur_dev=0;
	des_dev=0;
	h[8] = 0;
	m=0;
	
	for(i=1;i<7;i++)
	{
		if(bit_is_clear(PIND,i))
		{
			h[i] = 1;
		}
		else
		{
			h[i] = 0;
			m++;
		}
		cur_dev += h[i] * sensor_weight[i];
	}
	
	if(m==0)
	{m=1;}
	
	cur_dev = (cur_dev / m) * 1500;
	error = des_dev - cur_dev;
	return error;
}

float pid(float err)
{
	E_sum += err;
	if (E_sum > E_Max)
	{
		E_sum = E_Max;
	}
	else if (E_sum < -E_Max)
	{
		E_sum = -E_Max;
	}
	
	pid_out = (kp * err) + (ki * E_sum) + (kd * (err - e_old));
	e_old = err;
	
	if(pid_out > Max)
	{
		pid_out = Max;
	}
	else if (pid_out < -Max)
	{
		pid_out = -Max;
	}
	return pid_out;
}

void motorPIDcontrol(int base)
{
	OCR1A = base + (pid(err_calc())) + 100 ;
	OCR1B = base - (pid(err_calc()));
	PORTA |= ((1<<PINA1) | (1<<PINA2));
}
/* middle 6 sensors for line following using pid concept generating error value
and last 2 one will be for node recognition
*/
void mazeSolve()
{
	while(!x)
	{
		mode = readSensor();
		switch(mode)
		{
			case 00:
			motor_stop();
			runExtraInch();
			temp = 0;
			temp = readSensor();
			if(temp == 0)
			{
				mazeEnd();
			}
			else
			{
				OCR1A = 0;
				OCR1B = 800;
				_delay_ms(800);
				save_path('L');
			}
			break;
			
			case 255:				//no_line
			_delay_ms(300);
			motor_stop();
			PORTA &= ~(1<<PINA2);
			PORTA |= (1<<PINA1);
			//goAndTurn('B');	//turn,angle					***
			OCR1B = 700;
			OCR1A = 700; 
			_delay_ms(1000);
			save_path('B');
			break;
			
			case 248:
			case 240:
			case 224:
			motor_stop();
			runExtraInch();
			temp = 0;
			temp = readSensor();
			if(temp == 255)
			{
				OCR1B = 0;
				OCR1A = 900;
				_delay_ms(850);
				save_path('R');
			}
			else
			{
				motorPIDcontrol(dry_speed);
				save_path('S');
			}
			break;
			
			case 15:
			case 7:
			case 31:
			case 3:
			motor_stop();
			runExtraInch();
			//got_itL();
			OCR1A = 0;
			OCR1B = 800;
			_delay_ms(800);
			save_path('L');
			break;
			
			case 231:
			case 207:
			case 243:
			case 199:
			case 227:
			motorPIDcontrol(dry_speed);			//calculatePID(); //motorPIDcontrol();
			break;
			
			default:
			motorPIDcontrol(dry_speed);
		}
	}
	while(x)
	{	
		//int r;
		//r=strlen(path);
		
		if((bit_is_set(PIND,0)) || (bit_is_set(PIND,7)))
		{
			char dir=path[c];
			runExtraInch1();
		//	if(r<=c)
			//{
				//motor_stop();
				//_delay_ms(1000);
			//}
			switch(dir)
			{
				case 'L':
				OCR1A = 0;
				OCR1B = 800;
				_delay_ms(830);
				c++;
				break;
				
				case 'R':
				OCR1B = 0;
				OCR1A = 1000;
				_delay_ms(800);
				c++;
				break;
				
				case 'S':
				motorPIDcontrol(dry_speed);
				c++;
				break;
				
				default:
				//goAndTurn('B');
				motor_stop();
			}
		}
		else
		{
			motorPIDcontrol(dry_speed);
		}
		
	}
}

void runExtraInch()
{
	OCR1A = 600;
	OCR1B = 500;           // motor speed   check;;;;
 	_delay_ms(300);
	motor_stop();
}
void runExtraInch1()
{
	OCR1A = 600;
	OCR1B = 500;           // motor speed   check;;;;
	_delay_ms(200);
	motor_stop();
}

void motor_stop()
{
	OCR1A = 0;
	OCR1B = 0;
	_delay_ms(500);
}

void mazeEnd()
{
	led();
	//OCR1A = 0;
	//OCR1B = 0;
	_delay_ms(5000);
	//_delay_ms(10000);
	simplifyPath();
	_delay_ms(2000);
	PORTA &= ~(1<<PINA3);
	x=1;
	_delay_ms(100);
}

void goAndTurn(char dir)
{
	switch(dir)
	{
		case 'R':
		while((n=readSensor() & (1<<3)) && (n=readSensor() & (1<<4)))
		{
			OCR1A=500;
			OCR1B=0;
		}
		break;
		
		case 'B':
		PORTA &= ~(1<<PINA2);
		while(~((n=readSensor() & (1<<3)) && (n=readSensor() & (1<<4))))
		{
			OCR1A=500;
			OCR1B=500;
		}
		break;
		
		case 'L':
		while((n=readSensor() & (1<<3)) && (n=readSensor() & (1<<4)))
		{
			OCR1B=500;
			OCR1A=0;
		}
		break;
		
		default:
		motorPIDcontrol(dry_speed);
	}
}


void save_path(char dir)
{
	path[pathLength] = dir;
	pathLength++;
	USART_Transmitchar(dir,0);
	USART_Transmitchar(0x0d,0);
	_delay_us(1);
}



int arrShift(int i ,int len)
{
	for(i=i ; i<len-2 ; i++)
	{
		path[i] = path[i+2];
	}
	len = len-2;
	return len;
}

void simplifyPath()
{
	int len = strlen(path);
	int check = 0;
	
	start :
	for(i=0 ; i<len-2 ; i++)
	{
		if(path[i+1] == 'B')
		{
			switch(path[i])
			{
				case 'L':
				switch(path[i+2])
				{
					case 'L':
					path[i+2] = 'S';
					check = 1;
					break;
					
					case 'S':
					path[i+2] = 'R';
					check = 1;
					break;
					
					case 'R':
					path[i+2] = 'B';
					check = 1;
					break;
				}
				break;
				
				case 'S':
				switch(path[i+2])
				{
					case 'L':
					path[i+2] = 'R';
					check = 1;
					break;
					
					case 'S':
					path[i+2] = 'B';
					check = 1;
					break;
				}
				break;
				
				case 'R':
				switch(path[i+2])
				{
					case 'L':
					path[i+2] = 'B';
					check = 1;
					break;
				}
				break;
			}
			if(check == 1)
			{
				len = arrShift(i,len);
				check = 0;
				goto start;
			}
		}
	}
	
}

void led(void)
{
	int z=1;
	while(z)
	{
		OCR1A = 0;
		OCR1B = 0;
		PORTA |= (1<<PINA3);
		if(bit_is_set(PINC,4))
		{
			z=0;
		}
	}
}


//speed at all turns
//led pin 
