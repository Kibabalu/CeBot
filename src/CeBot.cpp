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

#include <DebugMacro.h>                             // dprint(x) and dshow("Blablubb");
#include <RegisterBitsMacros.h>                     // fast direct manipulation of registers
#include <Arduino_FreeRTOS.h>                       // FreeRTOS-Port for ATmega1284P
#include <SPI.h>                                    // Support for serial peripherial interface
#include <PololuMaestro.h>                          // support for Pololu Maestro servo driver
#include <KibaControl.hpp>                          // 1-D- and 2-D-maps, PID controller
#include <Neurona.h>                                // Multi Layer Perceptron for nonlinear regression

#define DEBUG 0                                     // enable/disable debug mode

#define LEDPORT PORTD                               // PORTD digital outputs for LEDs etc.
#define ucLED1 ( 1 << PD6 )                         // digital output PD4 for LED 1
#define ucLED2 ( 1 << PD5 )                         // digital output PD5 for LED 2
#define ucSummer ( 1 << PD7)                        // digital output PD7 for summer (aTeVal board)
#define ucButton1 ( 1 << PD2)
#define ucButton2 ( 1 << PD3)
#define ucButton3 ( 1 << PD4)
#define ucPoti1 ( 1 << PA1)
#define ucPoti2 ( 1 << PA0)

#define ucNumberChannelsMaestro 6                   // Number channels of the Maestro board
#define ucNumberPulsesMaestro 4                     // Number pulses per microsecond Maestro board
#define maestroSerial Serial1                       // serial communication with the Maestro board

/*---------------------------------------------------------------------------------------------------*/
/*
 * global declarations
 */
uint8_t ucLED1State = LOW;                            // state of LED1
uint8_t ucLED2State = LOW;                            // state of LED2, toggles from time to time
uint8_t ucLED3State = LOW;                            // state of LED3, toggles from time to time

TaskHandle_t pvTask1000ms;                            // handle for 1000ms task
TaskHandle_t pvTask100ms;                             // handle for 100ms task
TaskHandle_t pvTask10ms;                              // handle for 10ms task

uint16_t    usActMaxVel[ ] = {  0, 0, 0, 0, 0, 0 };                     // maximum servo speeds
uint8_t     ucActMaxAcc[ ] = { 4, 4, 4, 4, 4, 4 };                      // maximum servo accelarations
uint16_t    usActNomPos[ ] = { 992, 1500, 1500, 1500, 1500, 1500 };     // nominal servo positions
uint16_t    usActDesPos[ ] = { 1500, 1500, 1500, 1500, 1500, 1500 };    // desired servo positions
uint16_t    usActMinPos[ ] = { 432, 912, 944, 816, 352, 800 };          // minimum servo positions
uint16_t    usActMaxPos[ ] = { 2400, 2400, 2032, 2192, 2400, 2224 };    // maximum servo positions
uint16_t    usActCurPos[ 6 ];                                           // actual servo positions

const PROGMEM uint32_t ulBaud = 19200;                  // Baud rate for serial communication

const PROGMEM uint16_t iIntervalTicks1000ms = 977;      // sampling time in units of 1024 usec
const PROGMEM uint16_t iIntervalTicks100ms = 98;        // sampling time in units of 1024 usec
const PROGMEM uint16_t iIntervalTicks10ms = 10;         // sampling time in units of 1024 usec

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
static void vTask10ms( void* arg );                             // 10ms task function
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
        vTaskDelayUntil( &ticks, iIntervalTicks1000ms / portTICK_PERIOD_MS );    // wait until time for next task run

        /*
         * the following stuff is done repeately every 1000ms:
         */

         // toggle LED 2
         LEDPORT ^= ucLED2;

        //vSetLimits( usActMaxVel, ucActMaxAcc);              // set limits for speed and accelerations

        //vGetActPos( usActCurPos );                        // get actual servo positions

        /* computung new desired positions: */
        usActDesPos[ 0 ] = (uint16_t)random( usActMinPos[ 0 ], usActMaxPos[ 0 ] );
        usActDesPos[ 1 ] = (uint16_t)random( usActMinPos[ 1 ], usActMaxPos[ 1 ]-500 );
        usActDesPos[ 2 ] = (uint16_t)random( usActMinPos[ 2 ] + 500, usActMaxPos[ 2 ] );
        usActDesPos[ 3 ] = (uint16_t)random( usActMinPos[ 3 ], usActMaxPos[ 3 ] );
        usActDesPos[ 4 ] = (uint16_t)random( usActMinPos[ 4 ], usActMaxPos[ 4 ] );
        usActDesPos[ 5 ] = (uint16_t)random( usActMinPos[ 5 ], usActMaxPos[ 5 ] );

        /*if ( !maestro.getMovingState( ) )                    // if all movings are finished
        {
            vSetDesPos( usActDesPos );                       // set desired servo positions
        }*/

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
        vTaskDelayUntil( &ticks, iIntervalTicks100ms / portTICK_PERIOD_MS );     // wait until time for next task run

        /*
         * the following stuff is done repeately every 100ms:
         */

         // toggle LED 1
         LEDPORT ^= ucLED1;
    }
}
/*---------------------------------------------------------------------------------------------------*/
/*
 * 10ms task
 */
static void vTask10ms( void* arg )
{
    TickType_t ticks = xTaskGetTickCount( );    // initialise the ticks variable with the current time

    while ( 1 )
    {
        //vTaskDelayUntil( &ticks, iIntervalTicks10ms );      // wait until time for next task run
        vTaskDelayUntil( &ticks, iIntervalTicks10ms / portTICK_PERIOD_MS );
        /*
         * the following stuff is done repeately every 10ms:
         */

    }
}
/*---------------------------------------------------------------------------------------------------*/
/*
 * setup function
 */
void setup( )
{
    randomSeed(analogRead(5));                                  // randomize using noise from analog

    maestroSerial.begin(ulBaud);

    /*
     * defining the input/output behaviour of the port pins
     */
    DDRD |= ucLED1 | ucLED2 | ucSummer;                  // digital outputs
    DDRD &= ~ ( ucButton1 | ucButton2 | ucButton3);     // digital inputs

    ADMUX |= (1<<REFS0);                                // AVCC is reference voltage
    ADCSRA |= (1<<ADPS1) | (1<<ADPS2);                  // dividing factor, frequency
    ADCSRA |= (1<<ADEN);                                // switching on the ADC
    ADCSRA |= (1<<ADSC);                                // 1st dummy converting

    /*
     * task creation stuff:
     */
    portBASE_TYPE s1, s2, s3;     // return variables for the RTOS task creation
    /*
     * setting up the RTOS tasks and starting the scheduler
     */
    s1 = xTaskCreate( vTask1000ms, NULL, configMINIMAL_STACK_SIZE, NULL, 3, &pvTask1000ms );
                                                                // create 1000ms task at priority 3
    s2 = xTaskCreate( vTask100ms, NULL, configMINIMAL_STACK_SIZE, NULL, 2, &pvTask100ms );
                                                                // create 100ms task at priority 2
    s3 = xTaskCreate( vTask10ms, NULL, configMINIMAL_STACK_SIZE, NULL, 1, &pvTask10ms );
                                                                // create 10ms task at priority 1
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
