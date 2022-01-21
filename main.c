/******************************************************************************
* Project Name		: Mesh_Flood_PSoC4BLE
* File Name			: main.c
* Version 			: 1.0
* Device Used		: CY8C4247LQI-BL483
* Software Used		: PSoC Creator 3.1 SP1
* Compiler    		: ARM GCC 4.8.4, ARM RVDS Generic, ARM MDK Generic
* Related Hardware	: CY8CKIT-042-BLE Bluetooth Low Energy Pioneer Kit 
* Owner             : roit@cypress.com
*
********************************************************************************
* Copyright (2014-15), Cypress Semiconductor Corporation. All Rights Reserved.
********************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress)
* and is protected by and subject to worldwide patent protection (United
* States and foreign), United States copyright laws and international treaty
* provisions. Cypress hereby grants to licensee a personal, non-exclusive,
* non-transferable license to copy, use, modify, create derivative works of,
* and compile the Cypress Source Code and derivative works for the sole
* purpose of creating custom software in support of licensee product to be
* used only in conjunction with a Cypress integrated circuit as specified in
* the applicable agreement. Any reproduction, modification, translation,
* compilation, or representation of this software except as specified above 
* is prohibited without the express written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH 
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the 
* materials described herein. Cypress does not assume any liability arising out 
* of the application or use of any product or circuit described herein. Cypress 
* does not authorize its products for use as critical components in life-support 
* systems where a malfunction or failure may reasonably be expected to result in 
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of 
* such use and in doing so indemnifies Cypress against all charges. 
*
* Use of this Software may be limited by and subject to the applicable Cypress
* software license agreement. 
*******************************************************************************/
#include <main.h>
#include <project.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

extern uint8 switch_Role;
extern uint8 ble_gap_state;
extern uint8 deviceConnected;

extern uint8 restartScanning;

extern uint8 dataADVCounter;

#ifdef ENABLE_ADV_DATA_COUNTER
extern CYBLE_GAPP_DISC_DATA_T  new_advData;
extern uint8 potential_node_found;
extern uint8 potential_node_bdAddr[6];
extern uint8 potential_node_bdAddrType;
#endif

extern uint8 clientConnectToDevice;
extern CYBLE_GAP_BD_ADDR_T				peripAddr;;

uint8 calc_red, calc_green;
uint8_t echo_flag = 0;
uint16_t echo_distance;
uint16_t echo_distance_1;
uint16_t echo_distance_2;
uint16_t echo_distance_3;
char serial_output[20];

int processTachFlag = 0;
int tachLeftNotify = 0;
int tachRightNotify = 0;
int tachLeft = 0;
int tachRight = 0;

int motorFlag = 0;
int direct = 0;


void updateSpeed()
{
    if(CyBle_GetState() != CYBLE_STATE_CONNECTED)
        return;

    CYBLE_GATTS_HANDLE_VALUE_NTF_T tempHandle;
    
    tempHandle.attrHandle = CYBLE_MOTORSERVICE_SPEEDLEFT_CHAR_HANDLE;
    tempHandle.value.val = (uint8 *)&speedLeft;
    tempHandle.value.len = 1;
    CyBle_GattsWriteAttributeValue(&tempHandle, 0, &cyBle_connHandle, 0);
    
    tempHandle.attrHandle = CYBLE_MOTORSERVICE_SPEEDRIGHT_CHAR_HANDLE;
    tempHandle.value.val = (uint8 *)&speedRight;
    tempHandle.value.len = 1;
    CyBle_GattsWriteAttributeValue(&tempHandle, 0, &cyBle_connHandle, 0);      
}

void setSpeed(motor m, int speed)
{
    int s = abs(speed);
   // int dir = (s == speed)?0:1;
    
    if(s > MAXSPEED)
        return;
    
    switch (m)
    {
        case LEFT:
            PWMLEFT_WriteCompare(s);
            speedLeft = speed;         
        break;
        case RIGHT:
            PWMRIGHT_WriteCompare(s);
            speedRight = speed;
        break;
    }
    updateSpeed();      // Update the speeed in the GATT database
}

void updateRpm()
{
    if(CyBle_GetState() != CYBLE_STATE_CONNECTED)
        return;
    
    CYBLE_GATTS_HANDLE_VALUE_NTF_T tempHandle;
    
    if (tachRightNotify)
        CyBle_GattsNotification(cyBle_connHandle, &tempHandle); 
        
}

CY_ISR(swisr)
{
   motorFlag = 1;
   Control_ClearInterrupt();
}

CY_ISR( Timer_Int_Handler_1 ) {
    // read centimeters
    echo_distance_1 = Timer_Echo_1_ReadCapture();
    echo_flag = 1;
    
    Timer_Echo_1_ClearInterrupt ( Timer_Echo_1_INTR_MASK_CC_MATCH );
}
CY_ISR( Timer_Int_Handler_3 ) {
    // read centimeters
    echo_distance_3 = Timer_Echo_3_ReadCapture();
    echo_flag = 1;
    
    Timer_Echo_3_ClearInterrupt ( Timer_Echo_3_INTR_MASK_CC_MATCH );
}

void start_trig()
{
    Pin_US_Trigger_1_Write(1);
    CyDelay(10);
    Pin_US_Trigger_1_Write(0);
    CyDelay(30);
    Pin_US_Trigger_3_Write(1);
    CyDelay(10);
    Pin_US_Trigger_3_Write(0);
    CyDelay(30);
}

bool process_sensors_vertical(void)
{
    if (((echo_distance_1>=2.0)&&(echo_distance_1<20.0))||((echo_distance_3>=2.0)&&(echo_distance_3<20.0))){
        setSpeed(LEFT,0);
        setSpeed(RIGHT,0);
    }
    else{
       setSpeed(LEFT,calc_red);
       setSpeed(RIGHT,calc_green);
    }
    return FALSE;
}

inline void processTach()
{
    processTachFlag = 0;
    updateRpm(); // Update the GATT database with the new RPM values
}

void sensor()
{
    CyGlobalIntDisable;
    start_trig();
    process_sensors_vertical();
    echo_flag = 0;
    CyGlobalIntEnable;
}

CY_ISR(tachISR)
{
    processTachFlag = 1;
    sensor();
    tachPwm_ReadStatusRegister(); // clear the interrupt  
}


void handleMotor()
{
    static int motorState = 1;
    if(motorFlag)
    {
        if(motorState)
        {
            PWMLEFT_Stop();
            PWMRIGHT_Stop();
        }
        else
        {
            PWMLEFT_Start();
            PWMRIGHT_Start();
        }
        motorState = !motorState;
        motorFlag = 0;
    } 
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*       Entry point for the firmware, Starts the system and continuously processes
* Application and BLE events.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main()
{
    /* Initialize system */	
	InitializeSystem();
    start_trig();
	
	#ifdef DEBUG_ENABLED
		UART_UartPutString("Sytem Started ");
		UART_UartPutCRLF(' ');
	#endif
	
    for(;;)
    {
		/* Process BLE Events. This function generates the respective 
		* events to application layer */
       
		CyBle_ProcessEvents();
        
        /* If flag is set, then switch the GAP role between Peripheral and 
		* Central. This function disconnects existing connection, if any, 
		* before switching role. Depending on switched role, the system will
		* either start scanning or start advertisement */
		SwitchRole();
		
		/* If a valid node is found, then this function will initiate a connection 
		* to that node. */
		ConnectToPeripheralDevice();
		
		/* This function will restart scanning when earlier connection has been
		* disconnected. This function also checks internal counter and sets flag to
		* switch role from Central to Peripheral when the count is above set value. */
		RestartCentralScanning();
      /*  
        while ( echo_flag == 1 ) {
            sensor();
        }*/
    }
}

/*******************************************************************************
* Function Name: InitializeSystem
********************************************************************************
* Summary:
*       Starts the components in the project and registers BLE event handler 
* function. It also modifies the advertisement packet and appends data counter
* at its end.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void InitializeSystem(void)
{
	/* Enable global interrupts. */
	CyGlobalIntEnable;
	
	/* Start BLE component and Register the generic Event callback function */
	CyBle_Start(GenericEventHandler);
    
    swint_StartEx(swisr);
	
	/* Start the PrISM component and configure drive mode of LED pins to be
	* initially OFF*/
	PrISM_1_Start();
	PrISM_2_Start();
    PWMLEFT_Start();
    PWMRIGHT_Start();
    setSpeed(LEFT,0);
    setSpeed(RIGHT,0);
    
    Timer_Echo_1_Start();
    Timer_Echo_3_Start();
       
    // Registration of Timer ISR
    Timer_Echo_Int_1_StartEx( Timer_Int_Handler_1 );
    Timer_Echo_Int_3_StartEx( Timer_Int_Handler_3 );
    
	PrISM_1_WritePulse0(255);
	PrISM_1_WritePulse1(255);
	PrISM_2_WritePulse0(0);
    
    tachPwm_Start();
    tachTimer_StartEx(tachISR);
	
	RED_SetDriveMode(RED_DM_STRONG);
	GREEN_SetDriveMode(GREEN_DM_STRONG);
	BLUE_SetDriveMode(BLUE_DM_STRONG);
	
	/* Configure the Watchdog (WDT) timer for 100 millisecond timing */
	InitializeWatchdog(WATCHDOG_COUNT_VAL);
    
	#ifdef ENABLE_ADV_DATA_COUNTER
	new_advData = *cyBle_discoveryModeInfo.advData;
	
	if( cyBle_discoveryModeInfo.advData->advDataLen < 29)
	{
		/* Initialize the DataCounter data in advertisement packet. This is custom data in 
		* ADV packet and used to track whether the RGB LED data is latest or not */
		new_advData.advData[cyBle_discoveryModeInfo.advData->advDataLen] = CUSTOM_ADV_DATA_MARKER;	
		new_advData.advData[cyBle_discoveryModeInfo.advData->advDataLen+1] = dataADVCounter;
		new_advData.advDataLen = cyBle_discoveryModeInfo.advData->advDataLen+2;
	}
	
	/* Assign the new ADV data to stack */
	cyBle_discoveryModeInfo.advData = &new_advData;
	#endif
		
	/* Provide a color pusle on RGB LED to indicate startup */
	PrISM_1_WritePulse1(128);
	CyDelay(20);
	PrISM_1_WritePulse1(255);
	handleMotor();
            
	#ifdef DEBUG_ENABLED
    UART_Start();
	UART_UartPutString("--------------------------Mesh Flood------------------------------------------");
	UART_UartPutCRLF(' ');
	#endif
}

/*******************************************************************************
* Function Name: UpdateRGBled
********************************************************************************
* Summary:
*        Update the RGB LED color by reconfiguring PrISM component. This function 
* is valid for PSoC 4 BLE parts and not PROC BLE, as PRoC BLE does not have UDBs
* that makes up the PrISM component. To replace this with software PrISM, refer to
* CY8CKIT-042-BLE Pioneer Kit example project, PRoC_BLE_CapSense_Slider_LED, from
* http://www.cypress.com/CY8CKIT-042-BLE
*
* Parameters:
*  rgb_led_data: array storing the RGB and Intensity vlaue
*  len: length of the array.
*
* Return:
*  void
*
*******************************************************************************/
void UpdateRGBled(uint8 * rgb_led_data, uint8 len)
{
// calc_blue;
	
	if(len == RGB_LED_DATA_LEN)
	{
		/* If a valid length packet has been sent, calculate the intensity of each of
		* R, G and B color and update the PrISM module */
		calc_red = (uint8)(((uint16)rgb_led_data[RGB_RED_INDEX]));
		calc_green = (uint8)(((uint16)rgb_led_data[RGB_GREEN_INDEX]));
	//	calc_blue = (uint8)(((uint16)rgb_led_data[RGB_BLUE_INDEX]*rgb_led_data[RGB_INTENSITY_INDEX])/RGB_LED_MAX_VAL);	

        setSpeed(LEFT, calc_red);
        setSpeed(RIGHT, calc_green);
        
        
        PrISM_1_WritePulse0(RGB_LED_MAX_VAL - calc_red);
		PrISM_1_WritePulse1(RGB_LED_MAX_VAL - calc_green);
		//PrISM_2_WritePulse0(RGB_LED_MAX_VAL - calc_blue);
        		
		#ifdef DEBUG_ENABLED
		UART_UartPutString("PrISM Updated \r\n\n");
        PrintNum(rgb_led_data[RGB_RED_INDEX]);
        UART_UartPutCRLF(' ');
        PrintNum(rgb_led_data[RGB_GREEN_INDEX]);
        UART_UartPutCRLF(' ');
        PrintNum(rgb_led_data[RGB_BLUE_INDEX]);
        UART_UartPutCRLF(' ');
        PrintNum(rgb_led_data[RGB_INTENSITY_INDEX]);
		UART_UartPutCRLF(' ');
		#endif
	}
}

/* [] END OF FILE */
