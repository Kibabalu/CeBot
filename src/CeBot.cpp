/*---------------------------------------------------------------------------------------------------*/
/*	Project 'bot2'	Cerasus bot	controlled by an ATmega328P                                          */
/*					                                                                                 */
/*					Hardware:   Cerasus Bot with Microcontroller ATmega328P and                      */
/*							    6-channel Pololu Micro Maestro servo driver, communication	         */
/*								with serial TTL interface                           			     */
/*																							   	     */
/*	2017 Frank Kirschbaum (frank.kirschbaum@me.com)                                 				 */
/*																								     */
/*																								     */
/*	Kibas Coding Standard and Style Guide:                                    					     */
/*  (inspired by http://www.freertos.org/FreeRTOS-Coding-Standard-and-Style-Guide.html)			     */
/*																								     */
/*	Name conventions:																			     */
/*	Prefixes for constants, variables, functions, and methods:   								     */
/*	void/void*                   v/pv    void                                                        */
/*	int/int*                     i/pi    integer                                                     */
/*	uint/uint*                   ui/pui  unsigned integer                                            */
/*	int8_t/int8_t*               c/pc    char (Byte)                                                 */
/*	uint8_t/uint8_t*             uc/puc  unsigned char										         */
/*	int16_t/int16_t*             s/ps    short                                                       */
/*	uint16_t/uint16_t*           us/pus  unsigned short                                              */
/*	int32_t/int32_t*             l/pl    long                                                        */
/*	uint32_t/uint32_t*           ul/pul  unsigned long                                               */
/*	char/unsigned char           uc/puc  char (byte) für Zeichen                                     */
/*	float/float*                 f/pf    float                                                       */
/*	double/double*               d/pd    double                                                      */
/*	BaseType_t/BaseType_t*       x/px    base type, optimal für Registerbreite                       */
/*	UBaseType_t/UBaseType_t*     ux/pux  unsigned base type, optimal für Registerbreite              */
/*	TickType_t/TickType_t*       x/px    16/32 Bit, abhängig von Registerbreite                      */
/*	size_t/size_t*               x/px                                                                */
/*	TaskHandle_t                 pv      Task-handle (Pointer) for referencing of Tasks              */
/*	SemaphoreHandle_t            x                                                                   */
/*	Postfix:                                                                                         */
/*	class member variables       XYZ_    underscore at the end of a member variable                  */
/*                                                                                                   */
/*	Readability of source code:	                                                                     */
/*	Space after ( and before )                                                                       */
/*	Space after [ and before ]                                                                       */
/*	under each function declaration etc. ----...---                                                  */
/*                                                                                                   */
/*	'Der Unterschied zwischen Theorie und Praxis ist in der Praxis größer als in der Theorie'        */
/*                                                                                                   */
/*---------------------------------------------------------------------------------------------------*/

#define DEBUG 0                                         // enable/disable debug mode

#include <DebugMacro.h>                                 // dprint(x) and dshow("Blablubb");
#include <RegisterBitsMacros.h>                         // fast direct manipulation of registers
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <util/atomic.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <SPI.h>                                    // Support for serial peripherial interface
#include <FreeRTOSconfig.h>
#include <Arduino_FreeRTOS.h>                       // FreeRTOS-Port for ATmega1284P
#include <PololuMaestro.h>                          // support for Pololu Maestro servo driver
#include <KibaControl.hpp>                          // 1-D- and 2-D-maps, PID controler
#include <Neurona.h>                                // Multi Layer Perceptron for nonlinear regression

#define LEDPORT PORTD
#define ucLEDPin1 PD4                                 // digital output for LED0
#define ucLEDPin2 PD5                                // digital output for LED1
#define ucLEDPin3 PD6                                 // digital output for LED2

#define ucNumberChannelsMaestro 6                   // Number channels of the Maestro board
#define ucNumberPulsesMaestro 4                     // Number pulses per microsecond Maestro board
#define maestroSerial Serial1                        // serial communication with the Maestro board

/*---------------------------------------------------------------------------------------------------*/
/*
 * global declarations
 */
uint8_t ucLED1State = 0;                              // state of LED1
uint8_t ucLED2State = LOW;                            // state of LED2, toggles from time to time
uint8_t ucLED3State = 0;                              // state of LED3, toggles from time to time
uint8_t ucLED4State = 0;                              // state of LED4, toggles from time to time
uint8_t ucRGBLEDState = 1;

TaskHandle_t pvTask1000ms;                            // handle for 1000ms task
TaskHandle_t pvTask100ms;                             // handle for 100ms task
TaskHandle_t pvTask20ms;                              // handle for 20ms task

uint16_t    usActMaxVel[ ] = {  0, 0, 0, 0, 0, 0 };                     // maximum servo speeds
uint8_t     ucActMaxAcc[ ] = { 4, 4, 4, 4, 4, 4 };                      // maximum servo accelarations
uint16_t    usActNomPos[ ] = { 992, 1500, 1500, 1500, 1500, 1500 };     // nominal servo positions
uint16_t    usActDesPos[ ] = { 1500, 1500, 1500, 1500, 1500, 1500 };    // desired servo positions
uint16_t    usActMinPos[ ] = { 432, 912, 944, 816, 352, 800 };          // minimum servo positions
uint16_t    usActMaxPos[ ] = { 2400, 2400, 2032, 2192, 2400, 2224 };    // maximum servo positions
uint16_t    usActCurPos[ 6 ];                                           // actual servo positions

const PROGMEM uint32_t ulBaud = 19200;                          // Baud rate for serial communication

const PROGMEM uint16_t iIntervalTicks1000ms = 977;              // sampling time in units of 1024 usec
const PROGMEM uint16_t iIntervalTicks100ms = 98;                // sampling time in units of 1024 usec
const PROGMEM uint16_t iIntervalTicks20ms = 20;                 // sampling time in units of 1024 usec

MicroMaestro maestro(maestroSerial);
/*---------------------------------------------------------------------------------------------------*/
/*
 * function prototypes
 */
void vGetActPos( uint16_t* usActCurPos );                       // Get actual servo positions
void vSetDesPos( uint16_t* usActDesPos );                       // Set desired servo positions
void vSetLimits( uint16_t* usActMaxVel, uint8_t* ucActMaxAcc ); // Set speed and accelaration limits
uint8_t ucRGBLEDStateMachine( void );                           // state machine for toggling RGB-LED
static void vTask1000ms( void* arg );                           // 1000ms task function
static void vTask100ms( void* arg );                            // 100ms task function
static void vTask20ms( void* arg );                             // 20ms task function
void setup();                                                   // setup function
/*---------------------------------------------------------------------------------------------------*/
/*
 * Getting the actual servo positions
 */
void vGetActPos( uint16_t* usActCurPos )
{
    for ( int ii = 0; ii < ucNumberChannelsMaestro; ii++ )
    {
        usActCurPos[ ii ] = maestro.getPosition( ii );          // get the position of servo ii
    }
}
/*---------------------------------------------------------------------------------------------------*/
/*
 * Setting the desired servo positions
 */
void vSetDesPos( uint16_t* usActDesPos )
{
    for ( int ii = 0; ii < ucNumberChannelsMaestro; ii++ )
    {
        maestro.setTarget( ii, usActDesPos[ ii ] * ucNumberPulsesMaestro );  // set desired postion
    }
}
/*---------------------------------------------------------------------------------------------------*/
/*
 * Setting the maximal speeds and accelerations
 */
void vSetLimits( uint16_t* usActMaxVel, uint8_t* ucActMaxAcc )
{
    for ( int ii = 0; ii < ucNumberChannelsMaestro; ii++ )
    {
        maestro.setSpeed( ii, usActMaxVel[ ii ] );          // set max. speed for servo ii
        maestro.setAcceleration( ii, ucActMaxAcc[ ii ] );   // set max. accelaration for servo ii
    }
}
/*---------------------------------------------------------------------------------------------------*/
/*
 * 1000ms task
 */
static void vTask1000ms( void* arg )
{
    TickType_t ticks = xTaskGetTickCount( );    // initialise the ticks variable with the current time

    while ( 1 )
    {
        vTaskDelayUntil( &ticks, iIntervalTicks1000ms );    // wait until time for next task run

        /*
         * the following stuff is done repeately every 1000ms:
         */

         ucLED1State = ucLED1State == LOW ? HIGH : LOW;     // toggle ucLED1State2
         digitalWrite( ucLEDPin1, ucLED1State );            // write ucLED2State to output ucLEDPin2

        if(ucLED1State == LOW)
        {
            LEDPORT |= (1 << ucLEDPin1);
            ucLED1State = HIGH;
        }
        else
        {
            LEDPORT &= ~(1 << ucLEDPin1);
            ucLED1State = LOW;
        }

        vSetLimits( usActMaxVel, ucActMaxAcc);              // set limits for speed and accelerations

        //vGetActPos( usActCurPos );                        // get actual servo positions

        /* computung new desired positions: */
        usActDesPos[ 0 ] = (uint16_t)random( usActMinPos[ 0 ], usActMaxPos[ 0 ] );
        usActDesPos[ 1 ] = (uint16_t)random( usActMinPos[ 1 ], usActMaxPos[ 1 ]-500 );
        usActDesPos[ 2 ] = (uint16_t)random( usActMinPos[ 2 ] + 500, usActMaxPos[ 2 ] );
        usActDesPos[ 3 ] = (uint16_t)random( usActMinPos[ 3 ], usActMaxPos[ 3 ] );
        usActDesPos[ 4 ] = (uint16_t)random( usActMinPos[ 4 ], usActMaxPos[ 4 ] );
        usActDesPos[ 5 ] = (uint16_t)random( usActMinPos[ 5 ], usActMaxPos[ 5 ] );

        if ( !maestro.getMovingState( ) )                    // if all movings are finished
        {
            vSetDesPos( usActDesPos );                       // set desired servo positions
        }
    }
}
/*---------------------------------------------------------------------------------------------------*/
/*
 * 100ms task
 */
static void vTask100ms( void* arg )
{
    TickType_t ticks = xTaskGetTickCount( );    // initialise the ticks variable with the current time

    while ( 1 )
    {
        vTaskDelayUntil( &ticks, iIntervalTicks100ms );     // wait until time for next task run

        /*
         * the following stuff is done repeately every 100ms:
         */
    }
}
/*---------------------------------------------------------------------------------------------------*/
/*
 * 20ms task
 */
static void vTask20ms( void* arg )
{
    TickType_t ticks = xTaskGetTickCount( );    // initialise the ticks variable with the current time

    while ( 1 )
    {
        vTaskDelayUntil( &ticks, iIntervalTicks20ms );      // wait until time for next task run

        /*
         * the following stuff is done repeately every 20ms:
         */
    }
}
/*---------------------------------------------------------------------------------------------------*/
/*
 * setup function
 */
void setup( )
{
    DDRD |= (1 << DDD0);                                        // PD0 output
    DDRD |= (1 << DDD1);                                        // PD1 output
    DDRD |= (1 << DDD2);                                        // PD2 output


    randomSeed(analogRead(5));                                  // randomize using noise from analog

    maestroSerial.begin(ulBaud);

    /*
     *task creation stuff:
     */
    portBASE_TYPE s1, s2, s3;     // return variables for the RTOS task creation
    /*
     * setting up the RTOS tasks and starting the scheduler
     */
    s1 = xTaskCreate( vTask1000ms, NULL, configMINIMAL_STACK_SIZE, NULL, 3, &pvTask1000ms );
                                                                // create 1000ms task at priority 3
    s2 = xTaskCreate( vTask100ms, NULL, configMINIMAL_STACK_SIZE, NULL, 2, &pvTask100ms );
                                                                // create 100ms task at priority 2
    s3 = xTaskCreate( vTask20ms, NULL, configMINIMAL_STACK_SIZE, NULL, 1, &pvTask20ms );
                                                                // create 20ms task at priority 1
    /*
    * checking for creation errors
    */
    if ( s1 != pdPASS || s2 != pdPASS || s3 != pdPASS )
    {
        // dshow( "Creation problem" );
        while( 1 );
    }
    /*
     * start scheduler
     */
    // dshow( "starting scheduler ..." );
    vTaskStartScheduler( );                                     // starting the scheduler

    while( 1 );                                                 // everlasting loop
}
/*---------------------------------------------------------------------------------------------------*/
/*
 * main loop
 */
 void loop( )
 {
	 /* Not used - idle loop has a very small, configMINIMAL_STACK_SIZE, stack
	  * loop must never block
      */
 }
