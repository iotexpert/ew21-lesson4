/*******************************************************************************
* File Name: capsense_task.c
*
* Description: This file contains the task that handles the 3D magnetic joystick.
*
********************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
********************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death (“High Risk Product”). By
* including Cypress’s product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*****************************************​**************************************/


/******************************************************************************
* Header files includes
******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


#include "joystick_task.h"

#include "PSoC_TLx_interface.h"
#include "TLxJoystick.h"

#include "cloud_task.h"
/*******************************************************************************
* Global constants
*******************************************************************************/
#define JOYSTICK_INTERVAL_MS    (100)   /* in milliseconds*/
#define JOYSTICK_HYSTERESIS		(1)



/*******************************************************************************
* Function Name: task_joystick
********************************************************************************
* Summary:
*  Task that initializes the Joystick block and processes the input.
*
* Parameters:
*  void *param : Task parameter defined during task creation (unused)
*
*******************************************************************************/
void joystick_task(void* param)
{
	CY_UNUSED_PARAMETER(param);

    cy_rslt_t result;

    TLx493D_data_frame_t frame;
	TLxJoyStickXY_t joystick_curr;
	TLxJoyStickXY_t joystick_prev;

    /* Initialize I2C interface to talk to the TLx493D sensor */
	TLxI2CInit(CYBSP_I2C_SDA,
						CYBSP_I2C_SCL,
						0 /* Use Default Sensor Address */,
						0 /* Use Default Speed */,
						NULL /* Use Auto-assigned clock */);


    /* Configure the TLx493D sensor */
    result = TLx493D_init();
    if (result != CY_RSLT_SUCCESS)
    {
    	printf("Joystick not detected. Exiting Joystick task.\n");
    	vTaskDelete(NULL);
    }

    /* Set Sensor to Master Control Mode */
    result =  TLx493D_set_operation_mode(TLx493D_OP_MODE_MCM);

    /* Repeatedly running part of the task */
    for(;;)
    {
		/* Read a data frame from the sensor */
		result = TLx493D_read_frame(&frame);

		TLxJoystickCovertXY(&frame,&joystick_curr);

		/* Only update/print new value if it has changed by more than the hysteresis value */
		if((joystick_curr.x > (joystick_prev.x + JOYSTICK_HYSTERESIS)) || (joystick_curr.x < (joystick_prev.x - JOYSTICK_HYSTERESIS)))
		{
			printf("Joystick Position: %d\n", joystick_curr.x);
			cloud_sendMotorSpeed(joystick_curr.x);
		}

		joystick_prev.x = joystick_curr.x;
		joystick_prev.y = joystick_curr.y;


		vTaskDelay(JOYSTICK_INTERVAL_MS);
    }
}

