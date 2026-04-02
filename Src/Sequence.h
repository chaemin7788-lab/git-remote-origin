/*
 * Sequence.h
 *
 *  Created on: 2023. 12. 11.
 *      Author: C최동희
 */

#ifndef SEQUENCE_H_
#define SEQUENCE_H_

#include "main.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>

void SequenceMain();
void SequenceInit();
void SequenceTriger(uint8_t index);
void check_Seq(uint16_t fan_map);

#endif /* SEQUENCE_H_ */
