/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#if defined( CC2540_MINIDK )
  #include "simplekeys.h"
#endif

#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"
//#include "PM2_5.h"
//#include "CO2.h"
//#include "DHT22.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

#include "battservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define DUST_DENSITY_RESET_VALUE       0x7FFF
#define HUMIDITY_RESET_VALUE           0x7FFF
#define TEMPERATURE_RESET_VALUE        0x7FFF
#define CO2_DENSITY_RESET_VALUE        0x7FFF

// How often to perform periodic event (ms)
//#define SBP_PERIODIC_EVT_PERIOD_1ST                    10//jcn 5000
//#define SBP_PERIODIC_EVT_PERIOD                        1000//jcn 5000
#define SBP_PERIODIC_EVT_PERIOD_AD_INIT                  100
#define SBP_PERIODIC_EVT_PERIOD_FAST                     10//jcn 5000
#define SBP_PERIODIC_EVT_PERIOD_SLOW                     1000//950//950=1s-10ms*5(thMeasCT)//jcn 5000

// jcn, How often to perform ADC event(ms)
#define SBP_ADC_PERIODIC_EVT_PERIOD               10

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          1600  //jcn 160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         30  //jcn //0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
//static uint8 battMeasCT=0;
static uint8 thMeasCT=0x00;

static uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

/*
// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x14,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x53,   // 'S'
  0x69,   // 'i'
  0x6d,   // 'm'
  0x70,   // 'p'
  0x6c,   // 'l'
  0x65,   // 'e'
  0x42,   // 'B'
  0x4c,   // 'L'
  0x45,   // 'E'
  0x50,   // 'P'
  0x65,   // 'e'
  0x72,   // 'r'
  0x69,   // 'i'
  0x70,   // 'p'
  0x68,   // 'h'
  0x65,   // 'e'
  0x72,   // 'r'
  0x61,   // 'a'
  0x6c,   // 'l'

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};
*/

#define TH_DEVICE_ID  0x0000

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
/*  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),

  // air advertising data
  0x08,
  GAP_JCN_AIR_DATA,
  0x00,               // [9],  low byte of ID
  0x00,               // [10], high byte of ID
  0x00,               // [11], low byte of temperature
  0x00,               // [12], high byte of temperature
  0x00,               // [13], low byte of humidity
  0x00,               // [14], high byte of humidity
  0x00,               // [15], battery level, 0 ~ 100(%)

//  0x00,               // [14], high byte of humidity
//  0x00,               // [15], battery level, 0 ~ 100(%)
*/

/*
  // air advertising data
  0x09,               // [0], data length
  GAP_JCN_AIR_DATA,   // [1], 
  LO_UINT16(TH_DEVICE_ID),               // [2], low byte of ID
  HI_UINT16(TH_DEVICE_ID),               // [3], high byte of ID
  0x00,               // [4], low byte of temperature
  0x00,               // [5], high byte of temperature
  0x00,               // [6], low byte of humidity
  0x00,               // [7], high byte of humidity
  0x00,               // [8], battery level, 0 ~ 100(%)
  0x00,               // [9], checksum, sum([2]:[8])
*/
  
  // air advertising data
  GAP_JCN_AIR_DATA,   // [0], 
  0x00,               // [1], RFU
  0x00,               // [2], low byte of temperature
  0x00,               // [3], high byte of temperature
  0x00,               // [4], humidity
  0x00,               // [5], battery level, 0 ~ 100(%)
  0x00,               // [6], checksum, sum([2]:[8])
};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Air-TH";

static uint16 ntcTemp;
static uint16 ntcTempCnt;
static uint16 ntcTempAdBandTop, ntcTempAdBandBtm;
/*
#define RES_TEMP_TABLE_SIZE     (70+1)
static uint16 ntcTempTable[RES_TEMP_TABLE_SIZE] =
{
//10K pull-down, 0~70degrees, step=1degree
//	0	1	2	3	4	5	6	7	8	9
	518,	536,	555,	574,	593,	612,	631,	651,	671,	691,	//0
	711,	732,	752,	773,	794,	814,	835,	856,	877,	898,	//1
	919,	940,	961,	982,	1003,	1024,	1044,	1065,	1085,	1105,	//2
	1125,	1145,	1164,	1184,	1203,	1222,	1241,	1259,	1277,	1295,	//3
	1313,	1330,	1347,	1364,	1380,	1397,	1413,	1428,	1444,	1459,	//4
	1473,	1488,	1502,	1516,	1529,	1542,	1555,	1568,	1580,	1593,	//5
	1604,	1616,	1627,	1638,	1649,	1659,	1669,	1679,	1689,	1698,	//6
	1707,	                                                                        //7
};
*/
#define RES_TEMP_TABLE_SIZE     (700+1)
const uint16 ntcTempTable[RES_TEMP_TABLE_SIZE] =
{
//5.1K pull-down, 0~70degrees, step=0.1degree
//	0	1	2	3	4	5	6	7	8	9
	1208,	1212,	1217,	1222,	1227,	1232,	1237,	1241,	1246,	1251,	//0 
	1256,	1261,	1266,	1271,	1276,	1281,	1286,	1291,	1296,	1301,	//1 
	1306,	1311,	1316,	1321,	1326,	1331,	1336,	1341,	1347,	1352,	//2 
	1357,	1362,	1367,	1372,	1378,	1383,	1388,	1393,	1399,	1404,	//3 
	1409,	1414,	1420,	1425,	1430,	1436,	1441,	1446,	1452,	1457,	//4 
	1463,	1468,	1473,	1479,	1484,	1490,	1495,	1501,	1506,	1512,	//5 
	1517,	1523,	1529,	1534,	1540,	1545,	1551,	1557,	1562,	1568,	//6 
	1573,	1579,	1585,	1591,	1596,	1602,	1608,	1613,	1619,	1625,	//7 
	1631,	1637,	1642,	1648,	1654,	1660,	1666,	1672,	1677,	1683,	//8 
	1689,	1695,	1701,	1707,	1713,	1719,	1725,	1731,	1737,	1743,	//9 
	1749,	1755,	1761,	1767,	1773,	1779,	1785,	1791,	1797,	1804,	//10
	1810,	1816,	1822,	1828,	1834,	1841,	1847,	1853,	1859,	1866,	//11
	1872,	1878,	1884,	1891,	1897,	1903,	1910,	1916,	1922,	1929,	//12
	1935,	1941,	1948,	1954,	1960,	1967,	1973,	1980,	1986,	1993,	//13
	1999,	2006,	2012,	2019,	2025,	2032,	2038,	2045,	2051,	2058,	//14
	2064,	2071,	2078,	2084,	2091,	2097,	2104,	2111,	2117,	2124,	//15
	2131,	2137,	2144,	2151,	2157,	2164,	2171,	2178,	2184,	2191,	//16
	2198,	2205,	2212,	2218,	2225,	2232,	2239,	2246,	2252,	2259,	//17
	2266,	2273,	2280,	2287,	2294,	2301,	2308,	2314,	2321,	2328,	//18
	2335,	2342,	2349,	2356,	2363,	2370,	2377,	2384,	2391,	2398,	//19
	2405,	2412,	2419,	2426,	2434,	2441,	2448,	2455,	2462,	2469,	//20
	2476,	2483,	2490,	2498,	2505,	2512,	2519,	2526,	2533,	2540,	//21
	2548,	2555,	2562,	2569,	2577,	2584,	2591,	2598,	2605,	2613,	//22
	2620,	2627,	2635,	2642,	2649,	2656,	2664,	2671,	2678,	2686,	//23
	2693,	2700,	2708,	2715,	2722,	2730,	2737,	2744,	2752,	2759,	//24
	2767,	2774,	2781,	2789,	2796,	2804,	2811,	2818,	2826,	2833,	//25
	2841,	2848,	2856,	2863,	2870,	2878,	2885,	2893,	2900,	2908,	//26
	2915,	2923,	2930,	2938,	2945,	2953,	2960,	2968,	2975,	2983,	//27
	2990,	2998,	3005,	3013,	3021,	3028,	3036,	3043,	3051,	3058,	//28
	3066,	3073,	3081,	3089,	3096,	3104,	3111,	3119,	3127,	3134,	//29
	3142,	3149,	3157,	3165,	3172,	3180,	3187,	3195,	3203,	3210,	//30
	3218,	3225,	3233,	3241,	3248,	3256,	3264,	3271,	3279,	3287,	//31
	3294,	3302,	3309,	3317,	3325,	3332,	3340,	3348,	3355,	3363,	//32
	3371,	3378,	3386,	3394,	3401,	3409,	3417,	3424,	3432,	3440,	//33
	3447,	3455,	3463,	3470,	3478,	3486,	3493,	3501,	3509,	3516,	//34
	3524,	3532,	3539,	3547,	3555,	3562,	3570,	3578,	3585,	3593,	//35
	3601,	3608,	3616,	3624,	3631,	3639,	3647,	3654,	3662,	3670,	//36
	3677,	3685,	3693,	3700,	3708,	3716,	3723,	3731,	3739,	3746,	//37
	3754,	3761,	3769,	3777,	3784,	3792,	3800,	3807,	3815,	3823,	//38
	3830,	3838,	3845,	3853,	3861,	3868,	3876,	3884,	3891,	3899,	//39
	3906,	3914,	3921,	3929,	3937,	3944,	3952,	3959,	3967,	3975,	//40
	3982,	3990,	3997,	4005,	4012,	4020,	4028,	4035,	4043,	4050,	//41
	4058,	4065,	4073,	4080,	4088,	4095,	4103,	4110,	4118,	4125,	//42
	4133,	4140,	4148,	4155,	4163,	4170,	4178,	4185,	4193,	4200,	//43
	4208,	4215,	4223,	4230,	4237,	4245,	4252,	4260,	4267,	4275,	//44
	4282,	4289,	4297,	4304,	4312,	4319,	4326,	4334,	4341,	4348,	//45
	4356,	4363,	4370,	4378,	4385,	4392,	4400,	4407,	4414,	4422,	//46
	4429,	4436,	4444,	4451,	4458,	4465,	4473,	4480,	4487,	4494,	//47
	4502,	4509,	4516,	4523,	4530,	4538,	4545,	4552,	4559,	4566,	//48
	4574,	4581,	4588,	4595,	4602,	4609,	4616,	4624,	4631,	4638,	//49
	4645,	4652,	4659,	4666,	4673,	4680,	4687,	4694,	4701,	4709,	//50
	4716,	4723,	4730,	4737,	4744,	4751,	4758,	4765,	4772,	4778,	//51
	4785,	4792,	4799,	4806,	4813,	4820,	4827,	4834,	4841,	4848,	//52
	4855,	4861,	4868,	4875,	4882,	4889,	4896,	4902,	4909,	4916,	//53
	4923,	4930,	4936,	4943,	4950,	4957,	4963,	4970,	4977,	4984,	//54
	4990,	4997,	5004,	5010,	5017,	5024,	5030,	5037,	5044,	5050,	//55
	5057,	5064,	5070,	5077,	5083,	5090,	5097,	5103,	5110,	5116,	//56
	5123,	5129,	5136,	5142,	5149,	5155,	5162,	5168,	5175,	5181,	//57
	5188,	5194,	5201,	5207,	5213,	5220,	5226,	5233,	5239,	5245,	//58
	5252,	5258,	5264,	5271,	5277,	5283,	5290,	5296,	5302,	5309,	//59
	5315,	5321,	5327,	5334,	5340,	5346,	5352,	5358,	5365,	5371,	//60
	5377,	5383,	5389,	5395,	5401,	5408,	5414,	5420,	5426,	5432,	//61
	5438,	5444,	5450,	5456,	5462,	5468,	5474,	5480,	5486,	5492,	//62
	5498,	5504,	5510,	5516,	5522,	5528,	5534,	5540,	5546,	5552,	//63
	5557,	5563,	5569,	5575,	5581,	5587,	5593,	5598,	5604,	5610,	//64
	5616,	5621,	5627,	5633,	5639,	5644,	5650,	5656,	5662,	5667,	//65
	5673,	5679,	5684,	5690,	5696,	5701,	5707,	5712,	5718,	5724,	//66
	5729,	5735,	5740,	5746,	5751,	5757,	5762,	5768,	5773,	5779,	//67
	5784,	5790,	5795,	5801,	5806,	5812,	5817,	5823,	5828,	5833,	//68
	5839,	5844,	5849,	5855,	5860,	5865,	5871,	5876,	5881,	5887,	//69
	5892,	                                                                        //70
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
//static void performPeriodicADCTask( void );   // jcn
static void simpleProfileChangeCB( uint8 paramID );

/*
#define AD_NUM  10
static uint16 ad_buf[AD_NUM];
static uint16 ad_ct=0;
static uint16 ad_avg;
*/
#define BAT_AD_NUM  8
static uint16 bat_ad_buf[BAT_AD_NUM];
static uint16 bat_ad_ct=0;
static uint8 bat_ad_avg;
static uint8  bat_ad_init_finish=0;

#define NTC_AD_NUM  8
static uint16 ntc_ad_buf[NTC_AD_NUM];
static uint16 ntc_ad_ct=0;
static uint16 ntc_ad_avg;

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTED,
};
// Application state
static uint8 simpleBLEState = BLE_STATE_IDLE;
//ghostyu bond
static uint8 gPairStatus=0;/*用来管理当前的状态，如果密码不正确，立即取消连接，0表示未配对，1表示已配对*/
void ProcessPasscodeCB(uint8 *deviceAddr,uint16 connectionHandle,uint8 uiInputs,uint8 uiOutputs );
static void ProcessPairStateCB( uint16 connHandle, uint8 state, uint8 status );

#if defined( CC2540_MINIDK )
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );
#endif

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
static char *bdAddr2Str ( uint8 *pAddr );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

static void read_temperature(void);
static void read_batt_level(void);
uint16 *bubble(uint16*, uint16);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};


//要实现密码绑定，首先需要实现bongmgr的回调函数。
// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  ProcessPasscodeCB,                     // 密码回调
  ProcessPairStateCB                      // 绑定状态回调
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{
  simpleBLEPeripheral_TaskID = task_id;

  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = FALSE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = FALSE;//TRUE;
    #endif

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

//    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
	
  }

/*  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    //uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 pairMode = GAPBOND_PAIRING_MODE_INITIATE;
    uint8 mitm = FALSE;//TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;//GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }
*/
  
  // Initialize GATT attributes
/*  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
  Batt_AddService();
*/
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the SimpleProfile Characteristic Values
  {
    uint8 charValue1 = 1;
    uint8 charValue2 = 2;
    uint8 charValue3 = 3;
    uint8 charValue4 = 4;
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
  }

  //---------------- start ADC periodic Timer ----------------
//  osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_ADC_PERIODIC_EVT, SBP_ADC_PERIODIC_EVT_PERIOD );
  
#if defined( CC2540_MINIDK )

  SK_AddService( GATT_ALL_SERVICES ); // Simple Keys Profile

  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLEPeripheral_TaskID );

  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.

  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFC; // Port 0 pins P0.0 and P0.1 as input (buttons),
                // all others (P0.2-P0.7) as output
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output

  P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low

#endif // #if defined( CC2540_MINIDK )

#if (defined HAL_LCD) && (HAL_LCD == TRUE)

#if defined FEATURE_OAD
  #if defined (HAL_IMAGE_A)
    HalLcdWriteStringValue( "BLE Peri-A", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #else
    HalLcdWriteStringValue( "BLE Peri-B", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #endif // HAL_IMAGE_A
#else
  HalLcdWriteString( "BLE Peripheral", HAL_LCD_LINE_1 );
#endif // FEATURE_OAD

#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
//  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )

  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );

/*  //设置pwm端口为输出
  P0DIR|= BV(7);
  //设置为gpio
  P0SEL &= ~BV(7);
*/
  //设置端口为输出
//  P0DIR|= BV(4);
  P0DIR|= BV(5);
  //设置为gpio
//  P0SEL &= ~BV(4);
  P0SEL &= ~BV(5);

  ntcTempCnt = 0x00;
  ntcTempAdBandTop =
  ntcTempAdBandBtm = 0x00;
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{
  uint8 sum;
  uint8 i;

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

    // Set timer for first periodic event
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD_AD_INIT );
//    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_ADC_PERIODIC_EVT, SBP_ADC_PERIODIC_EVT_PERIOD );

    return ( events ^ SBP_START_DEVICE_EVT );
  }

  if ( events & SBP_PERIODIC_EVT )
  {
    if(bat_ad_init_finish==0)
    {
      read_batt_level();

      // check if all data are not "0x00"
      for(i=0; i<BAT_AD_NUM; i++)
      {
        if(bat_ad_buf[i] == 0x00)
        {
          break;
        }
      }

      // all data are not "00"
      if(i==BAT_AD_NUM)
      {
        bat_ad_init_finish = 1;

        advertData[5] = bat_ad_avg;
      }

      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD_AD_INIT );
    #if defined ( POWER_SAVING )
      osal_pwrmgr_device( PWRMGR_BATTERY );
    #endif
    }
    else
    {
      // Perform periodic application task
      performPeriodicTask();

      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD_SLOW );

    #if defined ( POWER_SAVING )
      osal_pwrmgr_device( PWRMGR_BATTERY );
    #endif
    }
    
    return (events ^ SBP_PERIODIC_EVT);
  }

  if ( events & SBP_ADC_PERIODIC_EVT )
  {
    // Restart timer
    if ( SBP_ADC_PERIODIC_EVT_PERIOD )
    {
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_ADC_PERIODIC_EVT, SBP_ADC_PERIODIC_EVT_PERIOD );
    }

    // jcn, Perform periodic ADC
//    performPeriodicADCTask();
      
    return (events ^ SBP_ADC_PERIODIC_EVT);
  }
  
#if defined ( PLUS_BROADCASTER )
  if ( events & SBP_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );

    return (events ^ SBP_ADV_IN_CONNECTION_EVT);
  }
#endif // PLUS_BROADCASTER

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  #if defined( CC2540_MINIDK )
    case KEY_CHANGE:
      simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  #endif // #if defined( CC2540_MINIDK )

  default:
    // do nothing
    break;
  }
}

#if defined( CC2540_MINIDK )
/*********************************************************************
 * @fn      simpleBLEPeripheral_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 SK_Keys = 0;

  VOID shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_SW_1 )
  {
    SK_Keys |= SK_KEY_LEFT;
  }

  if ( keys & HAL_KEY_SW_2 )
  {

    SK_Keys |= SK_KEY_RIGHT;

    // if device is not in a connection, pressing the right key should toggle
    // advertising on and off
    if( gapProfileState != GAPROLE_CONNECTED )
    {
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

      if( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
      }
      else
      {
        new_adv_enabled_status = FALSE;
      }

      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
    }

  }

  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
  SK_SetParameter( SK_KEY_ATTR, sizeof ( uint8 ), &SK_Keys );
}
#endif // #if defined( CC2540_MINIDK )

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          // Display device address
          HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
          HalLcdWriteString( "Initialized",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ADVERTISING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        simpleBLEState = BLE_STATE_IDLE;
      }
      break;

    case GAPROLE_CONNECTED:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        simpleBLEState = BLE_STATE_CONNECTED;
      }
      break;

    case GAPROLE_WAITING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ERROR:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    default:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif


}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
//extern uint16 watch_co2;
static void performPeriodicTask( void )
{
  static uint16 thdata;
  uint8 sum;
//  uint16 checksum;
  uint16 i;
  static uint16 BattMeasCT=0xfe, BattMeasNUM=1;
  static uint16 NtcMeasCT;
  static uint8 BattMeasInitCT=0;
//  static uint8 THDataOK=0;
  uint8 current_adv_enabled_status, new_advertising_enable;

//  P0_4 = 0;
  P0_5 = 0;    // prepare for NTC AD
  for(i=0; i<2000; i++);    // wait: 2000=2.2ms; 10000=8ms
  
  //--- battery lever ---
  BattMeasCT++;
  if(BattMeasCT >= 60)//BattMeasNUM)
  {
    BattMeasCT = 0x00;
  
        read_batt_level();
        advertData[5] = bat_ad_avg;
  }
  
  //--- temperature NTC ---
  NtcMeasCT++;
  if(NtcMeasCT >= 1)//  2)
  {
    NtcMeasCT = 0x00;
    read_temperature();
    advertData[2] = (uint8)ntcTemp;
    advertData[3] = (uint8)(ntcTemp>>8); // low byte first
//    P0_4 = 1;
    P0_5 = 1;    // NTC AD ending
  }
  
  sum=0x00;
  for(i=1; i<=5; i++)
  {
    sum +=  advertData[i];
  }
  advertData[6] = sum;
  
  GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

  GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );
  if( current_adv_enabled_status == FALSE )
  {
    new_advertising_enable = TRUE;
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_advertising_enable );
  }
}

/*********************************************************************
 * @fn      performPeriodicADCTask, jcn
 *
 * @brief   Perform a periodic ADC task. 
 *
 * @param   none
 *
 * @return  none
 */
/*
static void performPeriodicADCTask( void )
{
  uint16 tmp;
  uint8 i,j;
  uint8 charValue1[3]={0};
  static uint8 lastValue1[3]={0};
  uint16 tmp_max, tmp_mid, tmp_min;

//  P0_5  =    ~P0_5;
  // Configure ADC and perform a read
  HalAdcSetReference( HAL_ADC_REF_AVDD );
  
  ad_buf[ad_ct] = HalAdcRead( HAL_ADC_CHN_AIN7, HAL_ADC_RESOLUTION_14 );
  ad_ct++;
  if(ad_ct >= AD_NUM)   // AD_NUM = 10
  {
    ad_ct = 0x00;
    
    for(i=0; i<AD_NUM-1; i++)
    {
      for(j=i+1; j<AD_NUM; j++)
      {
        if(ad_buf[i] > ad_buf[j])
        {
          tmp = ad_buf[i];
          ad_buf[i] = ad_buf[j];
          ad_buf[j] = tmp;
        }
      }
    }

    ad_avg  = (ad_buf[3] + ad_buf[4] + ad_buf[5] + ad_buf[6]) >> 2;

    //---------------------------------------------------------
    if(ad_avg >= ntcTempAdBandTop)
    {
      tmp_max = RES_TEMP_TABLE_SIZE-1;
      tmp_min = ntcTempCnt;
    }
    else if(ad_avg <= ntcTempAdBandBtm)
    {
      tmp_max = ntcTempCnt;
      tmp_min = 0;
    }
    else
    {
      return;
    }

    tmp_mid = (tmp_max + tmp_min) >> 1;
    while(ad_avg != ntcTempTable[tmp_mid])
    {
      if(ad_avg > ntcTempTable[tmp_mid])
      {
        tmp_min = tmp_mid;
      }
      else
      {
        tmp_max = tmp_mid;
      }

      if(tmp_max > tmp_min + 1)
      {
        tmp_mid = (tmp_max + tmp_min) >> 1;
      }
      else
      {
        if(ad_avg >= ((ntcTempTable[tmp_max] + ntcTempTable[tmp_min]) >> 1))
        {
          tmp_mid = tmp_max;
        }
        else
        {
          tmp_mid = tmp_min;
        }
        
        break;
      }
    }

//    ntcTemp = tmp_mid*10;       // only for step=1
    ntcTemp = tmp_mid;           // for step=0.1

    if(tmp_mid == 0)
    {
      ntcTempAdBandTop = ntcTempTable[tmp_mid+1];
      ntcTempAdBandBtm = ad_avg;
    }
    else if(tmp_mid == RES_TEMP_TABLE_SIZE-1)
    {
      ntcTempAdBandTop = ad_avg;
      ntcTempAdBandBtm = ntcTempTable[tmp_mid-1];
    }
    else
    {
      ntcTempAdBandTop = ntcTempTable[tmp_mid+1];
      ntcTempAdBandBtm = ntcTempTable[tmp_mid-1];
    }
  }

  asm("nop");
}
*/

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{
  uint8 newValue;
  uint8 newValueBuf[20]={0};

  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR1:
    //  SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue );
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, newValueBuf );
  //    SetRGB(newValueBuf[0], newValueBuf[1], newValueBuf[2]);

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
   //     HalLcdWriteStringValue( "Char 1:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
        HalLcdWriteString((char*)newValueBuf, HAL_LCD_LINE_4 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &newValue );

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 3:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;

    default:
      // should not reach here!
      break;
  }
}

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

//绑定过程中的密码管理回调函数
static void ProcessPasscodeCB(uint8 *deviceAddr,uint16 connectionHandle,uint8 uiInputs,uint8 uiOutputs )
{
  uint32  passcode;
  uint8   str[7];

  //在这里可以设置存储，保存之前设定的密码，这样就可以动态修改配对密码了。
  // Create random passcode
//jcn  LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));
//jcn  passcode %= 1000000;
    passcode  = 111111;   //jcn

  //在lcd上显示当前的密码，这样手机端，根据此密码连接。
  // Display passcode to user
  if ( uiOutputs != 0 )
  {
    HalLcdWriteString( "Passcode:",  HAL_LCD_LINE_1 );
    HalLcdWriteString( (char *) _ltoa(passcode, str, 10),  HAL_LCD_LINE_2 );
  }
  
  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
}

//绑定过程中的状态管理，在这里可以设置标志位，当密码不正确时不允许连接。
static void ProcessPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )/*主机发起连接，会进入开始绑定状态*/
  {
    HalLcdWriteString( "Pairing started", HAL_LCD_LINE_1 );
	gPairStatus = 0;
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )/*当主机提交密码后，会进入完成*/
  {
    if ( status == SUCCESS )
    {
      HalLcdWriteString( "Pairing success", HAL_LCD_LINE_1 );/*密码正确*/
	  gPairStatus = 1;
    }
    else
    {
      HalLcdWriteStringValue( "Pairing fail", status, 10, HAL_LCD_LINE_1 );/*密码不正确，或者先前已经绑定*/
	  if(status ==8){/*已绑定*/
		gPairStatus = 1;
	  }else{
		gPairStatus = 0;
	  }
    }
	//判断配对结果，如果不正确立刻停止连接。
	if(simpleBLEState == BLE_STATE_CONNECTED && gPairStatus !=1){
	  GAPRole_TerminateConnection();
    }
  }
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    if ( status == SUCCESS )
    {
      HalLcdWriteString( "Bonding success", HAL_LCD_LINE_1 );
    }
  }
}

static void read_temperature(void)
{
  uint16 *sorted_data;
  uint16 tmp_max, tmp_mid, tmp_min;
  uint16 last_dig;

  ntc_ad_ct++;
  if(ntc_ad_ct >= NTC_AD_NUM)
  {
    ntc_ad_ct = 0x00;
  }

  // Configure ADC and perform a read
  HalAdcSetReference( HAL_ADC_REF_AVDD );
  ntc_ad_buf[ntc_ad_ct] = HalAdcRead( HAL_ADC_CHN_AIN7, HAL_ADC_RESOLUTION_14 );

  //--- sort ---
  sorted_data = bubble(ntc_ad_buf, NTC_AD_NUM);

  //--- average AD , NTC_AD_NUM == 8 ---
  ntc_ad_avg  = (sorted_data[2] + sorted_data[3] + sorted_data[4] + sorted_data[5]) >> 2;
/*
  //--- out of band ? ---
  if(ntc_ad_avg >= ntcTempAdBandTop)
  {
    tmp_max = RES_TEMP_TABLE_SIZE-1;
    tmp_min = ntcTempCnt;
  }
  else if(ntc_ad_avg <= ntcTempAdBandBtm)
  {
    tmp_max = ntcTempCnt;
    tmp_min = 0;
  }
  else
  {
    return;
  }
*/
/*NG
  if(ntc_ad_avg >= ntcTempTable[tmp_mid])
  {
    tmp_max = RES_TEMP_TABLE_SIZE-1;
    tmp_min = ntcTempCnt;
  }
  else
  {
    tmp_max = ntcTempCnt;
    tmp_min = 0;
  }
*/

  tmp_max = RES_TEMP_TABLE_SIZE-1;
  tmp_min = 0;
  if(ntc_ad_avg >= ntcTempTable[tmp_max])
  {
    tmp_mid = tmp_max;
    tmp_min = tmp_max - 1;
    last_dig = 0;
  }
  else if(ntc_ad_avg <= ntcTempTable[tmp_min])
  {
    tmp_mid = tmp_min;
    tmp_max = tmp_min + 1;
    last_dig = 0;
  }
  else
  {
  tmp_mid = (tmp_max + tmp_min) >> 1;
  while(ntc_ad_avg != ntcTempTable[tmp_mid])
  {
    if(ntc_ad_avg > ntcTempTable[tmp_mid])
    {
      tmp_min = tmp_mid;
    }
    else
    {
      tmp_max = tmp_mid;
    }

    if(tmp_max > tmp_min + 1)
    {
      tmp_mid = (tmp_max + tmp_min) >> 1;
      last_dig = 0;
    }
    else
    {
/*      if(ntc_ad_avg >= ((ntcTempTable[tmp_max] + ntcTempTable[tmp_min]) >> 1))
      {
        tmp_mid = tmp_max;
      }
      else
      {
        tmp_mid = tmp_min;
      }
*/
      tmp_mid = tmp_min;
      if(ntcTempTable[tmp_max] == ntcTempTable[tmp_min])
      {
        last_dig = 0;
      }
      else
      {
        last_dig = (ntc_ad_avg - ntcTempTable[tmp_min]) * 10 / (ntcTempTable[tmp_max] - ntcTempTable[tmp_min]);
      }
      
      break;
    }
  }
  }
  ntcTempCnt = tmp_mid;

  //--- temperature value ---
//    ntcTemp = tmp_mid*10;       // only for step=1
  //  ntcTemp = tmp_mid;           // for step=0.1, #.#
  ntcTemp = tmp_mid*10 + last_dig;           // for step=0.1, #.##

  //--- new band ---
  if(tmp_mid == 0)
  {
    ntcTempAdBandTop = ntcTempTable[1];
    ntcTempAdBandBtm = ntcTempTable[0];
  }
  else if(tmp_mid >= RES_TEMP_TABLE_SIZE-1)
  {
    ntcTempAdBandTop = ntcTempTable[RES_TEMP_TABLE_SIZE-1];
    ntcTempAdBandBtm = ntcTempTable[RES_TEMP_TABLE_SIZE-2];
  }
  else
  {
    ntcTempAdBandTop = ntcTempTable[tmp_mid+1];
    ntcTempAdBandBtm = ntcTempTable[tmp_mid-1];
  }
}

static void read_batt_level(void)
{
  uint16 *sorted_data;

  bat_ad_ct++;
  if(bat_ad_ct >= BAT_AD_NUM)
  {
    bat_ad_ct = 0x00;
  }
  // Configure ADC and perform a read
//  HalAdcSetReference( HAL_ADC_REF_125V );
//  bat_ad_buf[bat_ad_ct] = HalAdcRead( HAL_ADC_CHANNEL_VDD, HAL_ADC_RESOLUTION_10 );

  VOID Batt_MeasLevel();
	
  uint8 level;
  VOID Batt_GetParameter(BATT_PARAM_LEVEL,&level);
  bat_ad_buf[bat_ad_ct] = level;

  sorted_data = bubble(bat_ad_buf, BAT_AD_NUM);

  // BAT_AD_NUM == 8
  bat_ad_avg  = (uint8)((sorted_data[2] + sorted_data[3] + sorted_data[4] + sorted_data[5]) >> 2);
}

uint16 *res;
uint16 *bubble(uint16 *src, uint16 len)
{
  uint16 i,j,tmp;
//  uint16 *res;

//  memcpy(res,src,len);
  for(i=0; i<len; i++)
  {
    res[i] = src[i];
  }

  for(i=0; i<len-1; i++)
  {
    for(j=i+1; j<len; j++)
    {
      if(res[i] > res[j])
      {
        tmp = res[i];
        res[i] = res[j];
        res[j] = tmp;
      }
    }
  }

  return res;
}
/*********************************************************************
*********************************************************************/
