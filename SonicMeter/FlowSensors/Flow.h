/*
 * NextCentury Firmware
 * Copyright (C) 2014 - 2017. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, is not permitted.
 *
 * Created: 11/4/2016
 *  Author: A. Paul
 */


#ifndef FLOW_H_
#define FLOW_H_

//=========================================================================
// Notes
//=========================================================================


//=========================================================================
// Definitions
//=========================================================================


//=========================================================================
// Includes
//=========================================================================
#include "TDC.h"

//=========================================================================
// Types
//=========================================================================


//=========================================================================
// Prototypes
//=========================================================================
void FLOW_Init(void);
void FLOW_PrintValues(void);
void FLOW_TransactionCompleteCallback(TdcStatus_t status);

//=========================================================================
// Variables
//=========================================================================


//=========================================================================
// Implementations
//=========================================================================


#endif /* Flow.h */