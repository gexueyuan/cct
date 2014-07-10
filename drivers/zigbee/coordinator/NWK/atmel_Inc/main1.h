/* === MACROS ============================================================== */

/** Defines the default channel. */
#if (TAL_TYPE == AT86RF212)
    #ifdef CHINESE_BAND
        #define DEFAULT_CHANNEL                 (0)
        #define DEFAULT_CHANNEL_PAGE            (5)
    #else
        #define DEFAULT_CHANNEL                 (1)
        #define DEFAULT_CHANNEL_PAGE            (0)
    #endif  /* #ifdef CHINESE_BAND */
#else
#define DEFAULT_CHANNEL                 (tal_pib.CurrentChannel)
#define DEFAULT_CHANNEL_PAGE            (0)
#endif  /* #if (TAL_TYPE == AT86RF212) */
/** Defines the PAN ID of the network. */
#define DEFAULT_PAN_ID                 (tal_pib.PANId)// CCPU_ENDIAN_TO_LE16(0xBABE)
/** Defines the short address of the coordinator. */
#define COORD_SHORT_ADDR                (0x0000)
/** Defines the maximum number of devices this coordinator will handle. */
#define MAX_NUMBER_OF_DEVICES           (2)
/** Defines the bit mask of channels that should be scanned. */
#if (TAL_TYPE == AT86RF212)
    #if (DEFAULT_CHANNEL == 0)
        #define SCAN_ALL_CHANNELS       (0x00000001)
    #else
        #define SCAN_ALL_CHANNELS       (0x000007FE)
    #endif
#else
#define SCAN_ALL_CHANNELS             (1<<tal_pib.CurrentChannel)//  (0x0800)//(0x07FFF800)
#endif






/** Defines the short scan duration time. */
#define SCAN_DURATION_SHORT             (5)
/** Defines the long scan duration time. */
#define SCAN_DURATION_LONG              (6)
/** Defines the scan duration time. */
#define SCAN_DURATION_COORDINATOR       (1)
/** Defines Beacon Order for Nobeacon Network. */
#define NOBEACON_BO                     (15)
/** Defines Superframe Order for Nobeacon Network. */
#define NOBEACON_SO                     (15)
/** Defines the time to initiate a indirect data transmission to the device. */
#define APP_INDIRECT_DATA_DURATION_MS   (5000)
/** Define the LED on duration time. */
#define LED_ON_DURATION			(500000)
/** This is the time period in micro seconds for polling transmissions. */
#define APP_POLL_PERIOD_MS              (5000)

#if (NO_OF_LEDS >= 3)
#define LED_START                       (LED_0)
#define LED_NWK_SETUP                   (LED_1)
#define LED_DATA                        (LED_2)
#define LED_LINK                        (LED_1)
#define LED_RF_TX                       (LED_0)
#define LED_RF_RX                       (LED_2)
#elif (NO_OF_LEDS == 2)
#define LED_START                       (LED_0)
#define LED_NWK_SETUP                   (LED_0)
#define LED_DATA                        (LED_1)
#else
#define LED_START                       (LED_0)
#define LED_NWK_SETUP                   (LED_0)
#define LED_DATA                        (LED_0)
#endif
