/*
 * myTime.h
 *
 *  Created on: Apr 10, 2022
 *      Author: mathi
 */

#ifndef SRC_MYTIME_H_
#define SRC_MYTIME_H_


#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
//#include "main.h"


typedef struct data8_t data8_t;
struct data8_t {
    uint8_t BIN;
    uint8_t MSD;
    uint8_t LSD;
};

typedef enum TimeFormat TimeFormat;
enum TimeFormat{AM,PM24};

typedef struct Time Time;
struct Time {
    TimeFormat format;
    data8_t sec;
    data8_t min;
    data8_t hou;
};

typedef enum DayFormat DayFormat;
enum DayFormat{Mon,Tue,Wed,Thu,Fri,Sat,Sun,Undef};

typedef struct Date Date;
struct Date {
	DayFormat dayName;
	data8_t day;
	data8_t mon;
	data8_t yea;
};

void initClock(void);

void clock_Init(void);

void getCurrentTime(Time *time);

void setCurrentTime(Time *time);

void getCurrentDate(Date *data);

void setCurrentDate(Date *data);

uint8_t BCDtoBinary(uint8_t);

uint8_t BinarytoBCD(uint8_t value);

Time CreateTime(uint8_t hours, uint8_t minutes, uint8_t seconds);

Date CreateDate(uint8_t day, uint8_t month, uint16_t year);

void MX_RTC_Init(void);

#endif /* SRC_MYTIME_H_ */
