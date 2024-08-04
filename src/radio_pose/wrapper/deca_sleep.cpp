/*
 * platform.cpp
 *
 *  Created on: 17.03.2016
 *      Author: thomas
 */

#include "rodos.h"

extern "C" {

void deca_sleep(unsigned int time_ms){
	int64_t nextTime = NOW() + (int64_t)time_ms * MILLISECONDS;
	Thread::suspendCallerUntil(nextTime);

}


}


