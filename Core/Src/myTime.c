#include "myTime.h"

RTC_TypeDef *_RTC;
PWR_TypeDef *_PWR;
RTC_HandleTypeDef hrtc;

void clock_Init(void) {
	_RTC = RTC;
	_PWR = PWR;

	MX_RTC_Init();

}

void initClock(void) {
	_RTC = RTC;
	_PWR = PWR;


	/**************** RTC register access *************** */
	/**The RTC registers are 32-bit registers. The APB interface introduces 2 wait-states in RTC
	register accesses except on read accesses to calendar shadow registers when
	BYPSHAD=0**/
	_RTC->CR &= ~RTC_CR_BYPSHAD_Msk;
	_RTC->CR |= 1U << RTC_CR_BYPSHAD_Pos;


	/**************** RTC register write protection *************** */
	/**After system reset, the RTC registers are protected against parasitic write access with the
	DBP bit of the PWR power control register (PWR_CR). The DBP bit must be set to enable
	RTC registers write access.**/
	_PWR->CR &= ~PWR_CR_DBP_Msk;
	_PWR->CR |= 1U << PWR_CR_DBP_Pos;


	/**After backup domain reset, all the RTC registers are write-protected. Writing to the RTC
	registers is enabled by writing a key into the Write Protection register, RTC_WPR.
	The following steps are required to unlock the write protection on all the RTC registers
	except for RTC_ISR[13:8], RTC_TAFCR, and RTC_BKPxR.
	1. Write ‘0xCA’ into the RTC_WPR register.
	2. Write ‘0x53’ into the RTC_WPR register.
	Writing a wrong key reactivates the write protection.
	The protection mechanism is not affected by system reset**/
	_RTC->WPR = 0xCAU;
	_RTC->WPR = 0x53U;


	/**************** Calendar initialization and configuration ****************/
	/**To program the initial time and date calendar values, including the time format and the
	prescaler configuration, the following sequence is required:
	1. Set INIT bit to 1 in the RTC_ISR register to enter initialization mode. In this mode, the
	calendar counter is stopped and its value can be updated.**/
	_RTC->ISR &= ~RTC_ISR_INIT_Msk;
	_RTC->ISR |= 1U << RTC_ISR_INIT_Pos;


	/**2. Poll INITF bit of in the RTC_ISR register. The initialization phase mode is entered when
	INITF is set to 1. It takes from 1 to 2 RTCCLK clock cycles (due to clock
	synchronization).**/
	while(_RTC->ISR & RTC_ISR_INITF_Msk);


	/**3. To generate a 1 Hz clock for the calendar counter, program first the synchronous
	prescaler factor in RTC_PRER register, and then program the asynchronous prescaler
	factor. Even if only one of the two fields needs to be changed, 2 separate write
	accesses must be performed to the RTC_PRER register.**/
	_RTC->PRER = 0x7FUL;//PREDIV_A
	_RTC->PRER |= 0xFFUL << 16U;//PREDIV_S


	/**4. Load the initial time and date values in the shadow registers (RTC_TR and RTC_DR),
	and configure the time format (12 or 24 hours) through the FMT bit in the RTC_CR
	register.**/
	_RTC->TR = 0x123500UL;//12.35.00
	_RTC->DR = 0x126718UL;//2012-07-18
	_RTC->CR &= ~RTC_CR_FMT_Msk;//FMT 24H format
	_RTC->CR |= 0U << RTC_CR_FMT_Pos;
	//à modifier pour initialiser l'heure


	/**5. Exit the initialization mode by clearing the INIT bit. The actual calendar counter value is
	then automatically loaded and the counting restarts after 4 RTCCLK clock cycles.
	When the initialization sequence is complete, the calendar starts counting.**/
	_RTC->ISR &= ~RTC_ISR_INIT_Msk;
	_RTC->ISR |= 0U << RTC_ISR_INIT_Pos;


	/**Note: After a system reset, the application can read the INITS flag in the RTC_ISR register to
	check if the calendar has been initialized or not. If this flag equals 0, the calendar has not
	been initialized since the year field is set at its backup domain reset default value (0x00).
	To read the calendar after initialization, the software must first check that the RSF flag is set
	in the RTC_ISR register**/
}

void getCurrentTime(Time *time) {
	/**************** When BYPSHAD control bit is cleared in the RTC_CR register **************** (not concerned)/*/
	/**To read the RTC calendar registers (RTC_SSR, RTC_TR and RTC_DR) properly, the APB1
	clock frequency (f PCLK1 ) must be equal to or greater than seven times the fRTCCLK RTC
	clock frequency. This ensures a secure behavior of the synchronization mechanism.
	If the APB1 clock frequency is less than seven times the RTC clock frequency, the software
	must read the calendar time and date registers twice. If the second read of the RTC_TR
	gives the same result as the first read, this ensures that the data is correct. Otherwise a third
	read access must be done. In any case the APB1 clock frequency must never be lower than
	the RTC clock frequency. The RSF bit is set in RTC_ISR register each time the calendar registers are copied into the
	RTC_SSR, RTC_TR and RTC_DR shadow registers. The copy is performed every two
	RTCCLK cycles. To ensure consistency between the 3 values, reading either RTC_SSR or
	RTC_TR locks the values in the higher-order calendar shadow registers until RTC_DR is
	read. In case the software makes read accesses to the calendar in a time interval smaller
	than 2 RTCCLK periods: RSF must be cleared by software after the first calendar read, and
	then the software must wait until RSF is set before reading again the RTC_SSR, RTC_TR
	and RTC_DR registers.
	After waking up from low-power mode (Stop or Standby), RSF must be cleared by software.
	The software must then wait until it is set again before reading the RTC_SSR, RTC_TR and
	RTC_DR registers.
	The RSF bit must be cleared after wakeup and not before entering low-power mode.
	Note: After a system reset, the software must wait until RSF is set before reading the RTC_SSR,
	RTC_TR and RTC_DR registers. Indeed, a system reset resets the shadow registers to
	their default values.
	After an initialization (refer to Calendar initialization and configuration on page 804): the
	software must wait until RSF is set before reading the RTC_SSR, RTC_TR and RTC_DR
	registers.
	After synchronization (refer to Section 26.3.8: RTC synchronization): the software must wait
	until RSF is set before reading the RTC_SSR, RTC_TR and RTC_DR registers.**/

	/**************** When the BYPSHAD control bit is set in the RTC_CR register (bypass shadow registers)****************/
	/**Reading the calendar registers gives the values from the calendar counters directly, thus
	eliminating the need to wait for the RSF bit to be set. This is especially useful after exiting
	from low-power modes (STOP or Standby), since the shadow registers are not updated
	during these modes.
	When the BYPSHAD bit is set to 1, the results of the different registers might not be
	coherent with each other if an RTCCLK edge occurs between two read accesses to the
	registers. Additionally, the value of one of the registers may be incorrect if an RTCCLK edge
	occurs during the read operation. The software must read all the registers twice, and then
	compare the results to confirm that the data is coherent and correct. Alternatively, the
	software can just compare the two results of the least-significant calendar register.
	Note: While BYPSHAD=1, instructions which read the calendar registers require one extra APB
	cycle to complete.**/

	uint32_t RTC_TR = _RTC->TR;

	uint8_t secBCD = (RTC_TR & (RTC_TR_ST_Msk | RTC_TR_SU_Msk)) >> RTC_TR_SU_Pos;
	time->sec.MSD = (RTC_TR & RTC_TR_ST_Msk) >> RTC_TR_ST_Pos;
	time->sec.LSD = (RTC_TR & RTC_TR_SU_Msk) >> RTC_TR_SU_Pos;
	time->sec.BIN = BCDtoBinary(secBCD);

	uint8_t minBCD = (RTC_TR & (RTC_TR_MNT_Msk | RTC_TR_MNU_Msk)) >> RTC_TR_MNU_Pos;
	time->min.MSD = (RTC_TR & RTC_TR_MNT_Msk) >> RTC_TR_MNT_Pos;
	time->min.LSD = (RTC_TR & RTC_TR_MNU_Msk) >> RTC_TR_MNU_Pos;
	time->min.BIN = BCDtoBinary(minBCD);

	uint8_t houBCD = (RTC_TR & (RTC_TR_HT_Msk | RTC_TR_HU_Msk)) >> RTC_TR_HU_Pos;
	time->hou.MSD = (RTC_TR & RTC_TR_HT_Msk) >> RTC_TR_HT_Pos;
	time->hou.LSD = (RTC_TR & RTC_TR_HU_Msk) >> RTC_TR_HU_Pos;
	time->hou.BIN = BCDtoBinary(houBCD);

	uint8_t format = (RTC_TR & (RTC_TR_PM_Msk)) >> RTC_TR_PM_Pos;
	if (format) {
		time->format = AM;
	} else {
		time->format = PM24;
	}
}

void setCurrentTime(Time *time) {
	RTC_TimeTypeDef tm;
	tm.Hours = time->hou.BIN;
	tm.Minutes = time->min.BIN;
	tm.Seconds = time->sec.BIN;
	tm.TimeFormat = (time->format == AM) ? RTC_HOURFORMAT12_AM : RTC_HOURFORMAT12_PM;
	tm.SubSeconds = 0;
	tm.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	tm.StoreOperation = RTC_STOREOPERATION_RESET;

	HAL_RTC_SetTime(&hrtc, &tm, RTC_FORMAT_BIN);
}

void getCurrentDate(Date *date) {
	uint32_t RTC_DR = _RTC->DR;

	uint8_t yeaBCD = (RTC_DR & (RTC_DR_YT_Msk | RTC_DR_YU_Msk)) >> RTC_DR_YU_Pos;
	date->yea.MSD = (RTC_DR & RTC_DR_YT_Msk) >> RTC_DR_YT_Pos;
	date->yea.LSD = (RTC_DR & RTC_DR_YU_Msk) >> RTC_DR_YU_Pos;
	date->yea.BIN = BCDtoBinary(yeaBCD);

	uint8_t monBCD = (RTC_DR & (RTC_DR_MT_Msk | RTC_DR_MU_Msk)) >> RTC_DR_MU_Pos;
	date->mon.MSD = (RTC_DR & RTC_DR_MT_Msk) >> RTC_DR_MT_Pos;
	date->mon.LSD = (RTC_DR & RTC_DR_MU_Msk) >> RTC_DR_MU_Pos;
	date->mon.BIN = BCDtoBinary(monBCD);

	uint8_t dayBCD = (RTC_DR & (RTC_DR_DT_Msk | RTC_DR_DU_Msk)) >> RTC_DR_DU_Pos;
	date->day.MSD = (RTC_DR & RTC_DR_DT_Msk) >> RTC_DR_DT_Pos;
	date->day.LSD = (RTC_DR & RTC_DR_DU_Msk) >> RTC_DR_DU_Pos;
	date->day.BIN = BCDtoBinary(dayBCD);

	uint8_t dayf = (RTC_DR & RTC_DR_WDU_Msk) >> RTC_DR_WDU_Pos;
	if (dayf == 0) {
		date->dayName = Undef;
	} else if (dayf > 7) {
		date->dayName = Undef;
	} else {
		date->dayName = dayf - 1;
	}
}

void setCurrentDate(Date *date) {
	RTC_DateTypeDef dm;
	dm.WeekDay = date->dayName + 1;
	dm.Month = date->mon.BIN;
	dm.Date = date->day.BIN;
	dm.Year = date->yea.BIN;

	HAL_RTC_SetDate(&hrtc, &dm, RTC_FORMAT_BIN);
}

uint8_t BCDtoBinary(uint8_t value) {
	uint8_t MS4B = (value & 0xF0) >> 4U;
	uint8_t LS4B = (value & 0x0F) >> 0U;
	uint8_t ret = MS4B * 10 + LS4B;
	return ret;
}

uint8_t BinarytoBCD(uint8_t value) {
	uint8_t LSB = value % 10;
	uint8_t MSB = (value - LSB) /10;
	uint8_t ret = MSB<<4U | LSB<<0U;
	return ret;
}

Time CreateTime(uint8_t hours, uint8_t minutes, uint8_t seconds) {
	Time ret;
	uint8_t BCDhou = BinarytoBCD(hours);
	ret.hou.BIN = hours;
	ret.hou.MSD = (BCDhou & 0xF0) >> 4U;
	ret.hou.LSD = (BCDhou & 0x0F) >> 0U;

	uint8_t BCDmin = BinarytoBCD(minutes);
	ret.min.BIN = minutes;
	ret.min.MSD = (BCDmin & 0xF0) >> 4U;
	ret.min.LSD = (BCDmin & 0x0F) >> 0U;

	uint8_t BCDsec = BinarytoBCD(seconds);
	ret.sec.BIN = seconds;
	ret.sec.MSD = (BCDsec & 0xF0) >> 4U;
	ret.sec.LSD = (BCDsec & 0x0F) >> 0U;

	ret.format = PM24;
	return ret;
}

Date CreateDate(uint8_t day, uint8_t month, uint16_t year) {
	Date ret;
	uint8_t BCDday = BinarytoBCD(day);
	ret.day.BIN = day;
	ret.day.MSD = (BCDday & 0xF0) >> 4U;
	ret.day.LSD = (BCDday & 0x0F) >> 0U;

	uint8_t BCDmon = BinarytoBCD(month);
	ret.mon.BIN = month;
	ret.mon.MSD = (BCDmon & 0xF0) >> 4U;
	ret.mon.LSD = (BCDmon & 0x0F) >> 0U;

	uint8_t yearF = year%100;
	uint8_t BCDyea = BinarytoBCD(yearF);
	ret.yea.BIN = yearF;
	ret.yea.MSD = (BCDyea & 0xF0) >> 4U;
	ret.yea.LSD = (BCDyea & 0x0F) >> 0U;

	ret.dayName = Mon;
	return ret;
}

void MX_RTC_Init(void)
{
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
}
