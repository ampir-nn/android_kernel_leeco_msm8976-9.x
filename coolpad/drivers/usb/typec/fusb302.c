#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/kthread.h>
#include <linux/usb/otg.h>

#include "fusb302.h"

#define PD_SUPPORT
#ifdef PD_SUPPORT
#include "usbpd.h"
#endif

#define FUSB302_I2C_NAME "fusb302"
//#define CUST_EINT_CC_DECODER_NUM 95

int VBUS_5V_EN;
int VBUS_12V_EN;

#define TRUE 1
#define FALSE 0

#define PORT_UFP                0
#define PORT_DFP                1
#define PORT_DRP                2

#define HOST_CUR_NO     0
#define HOST_CUR_USB        1
#define HOST_CUR_1500       2
#define HOST_CUR_3000       3

#define CC_UNKNOW       4
#define CC_RD_3000      3
#define CC_RD_1500      2
#define CC_RD_DEFAULT       1
#define CC_RA           0

/* Configure Definition */
#define FUSB302_I2C_NUM     1
#define FUSB302_PORT_TYPE   USBTypeC_Source
#define FUSB302_HOST_CUR    HOST_CUR_USB
//#define FUSB302_PORT_TYPE PORT_DFP
//#define FUDB302_PORT_TYPE PORT_UFP

#define FUSB_MS_TO_NS(x) (x * 1000 * 1000)
#define Delay10us(x) udelay(x*10);


//#define FUSB302_DEBUG

#ifdef FUSB302_DEBUG
#define FUSB_LOG(fmt, args...)  printk(KERN_ERR"[fusb302]" fmt, ##args)
#else
#define FUSB_LOG(fmt, args...)	pr_debug("[fusb302]" fmt, ##args)
#endif

struct fusb302_i2c_data {
    struct i2c_client       *client;
    struct work_struct      eint_work;
    struct task_struct      *thread;

    spinlock_t      lock;

    #if defined(CONFIG_HAS_EARLYSUSPEND) && defined(USE_EARLY_SUSPEND)
    struct early_suspend        early_drv;
    #endif
};

struct fusb302_i2c_data *fusb_i2c_data;

static DECLARE_WAIT_QUEUE_HEAD(fusb_thread_wq);
int state_changed;

static struct hrtimer state_hrtimer;
static struct hrtimer debounce_hrtimer1;
static struct hrtimer debounce_hrtimer2;
static struct hrtimer toggle_hrtimer;

#ifdef PD_SUPPORT
extern PolicyState_t PolicyState;
extern ProtocolState_t ProtocolState;
extern PDTxStatus_t PDTxStatus;
#endif
/*      Variables accessible outside of the FUSB300 state machine */
FUSB300reg_t            Registers;          // Variable holding the current status of the FUSB300 registers
int                    USBPDActive;        // Variable to indicate whether the USB PD state machine is active or not
int                    USBPDEnabled;       // Variable to indicate whether USB PD is enabled (by the host)
int                  PRSwapTimer;        // Timer used to bail out of a PR_Swap from the Type-C side if necessary

USBTypeCPort            PortType;           // Variable indicating which type of port we are implementing
int                    blnCCPinIsCC1;      // Flag to indicate if the CC1 pin has been detected as the CC pin
int                    blnCCPinIsCC2;      // Flag to indicate if the CC2 pin has been detected as the CC pin
int                    blnSMEnabled;       // Flag to indicate whether the FUSB300 state machine is enabled
ConnectionState         ConnState;          // Variable indicating the current connection state

/*      Variables accessible only inside FUSB300 state machine */
static int             blnSrcPreferred;    // Flag to indicate whether we prefer the Src role when in DRP
static int             blnAccSupport;      // Flag to indicate whether the port supports accessories
static int             blnINTActive;       // Flag to indicate that an interrupt occurred that needs to be handled
static unsigned short           StateTimer;         // Timer used to validate proceeding to next state
static unsigned short           DebounceTimer1;     // Timer used for first level debouncing
static unsigned short           DebounceTimer2;     // Timer used for second level debouncing
static unsigned short           ToggleTimer;        // Timer used for CC swapping in the FUSB302
static CCTermType       CC1TermAct;         // Active CC1 termination value
static CCTermType       CC2TermAct;         // Active CC2 termination value
static CCTermType       CC1TermDeb;         // Debounced CC1 termination value
static CCTermType       CC2TermDeb;         // Debounced CC2 termination value
static USBTypeCCurrent  SinkCurrent;        // Variable to indicate the current capability we have received
static USBTypeCCurrent  SourceCurrent;      // Variable to indicate the current capability we are broadcasting

static char *con_stat[] = {
	"Disabled",
	"ErrorRecovery",
	"Unattached",
	"AttachWaitSink",
	"AttachedSink",
	"AttachWaitSource",
	"AttachedSource",
	"TrySource",
	"TryWaitSink",
	"AudioAccessory",
	"DebugAccessory",
	"AttachWaitAccessory",
	"PoweredAccessory",
	"UnsupportedAccessory",
	"DelayUnattached",
};

static char *cc_term_type[] = {
	"CCTypeNone",
	"CCTypeRa",
	"CCTypeRdUSB",
	"CCTypeRd1p5",
	"CCTypeRd3p0"
};
/* FUSB300 I2C Routines */
int FUSB300Write(unsigned char regAddr, unsigned char length, unsigned char* data)
{
    return i2c_smbus_write_i2c_block_data(fusb_i2c_data->client, regAddr, length, data);
}

int FUSB300Read(unsigned char regAddr, unsigned char length, unsigned char* data)
{
    return i2c_smbus_read_i2c_block_data(fusb_i2c_data->client, regAddr, length, data);
}

/*                     Internal Routines */
void SetStateDelayUnattached(void);
void StateMachineUnattached(void);
void StateMachineAttachWaitSnk(void);
void StateMachineAttachedSink(void);
void StateMachineAttachWaitSrc(void);
void StateMachineAttachedSource(void);
void StateMachineTrySrc(void);
void StateMachineTryWaitSnk(void);
void StateMachineAttachWaitAcc(void);
void StateMachineDelayUnattached(void);

void SetStateUnattached(void);
void SetStateAttachWaitSnk(void);
void SetStateAttachWaitAcc(void);
void SetStateAttachWaitSrc(void);
void SetStateTrySrc(void);
void SetStateAttachedSink(void);
void SetStateAttachedSrc(void);
void UpdateSinkCurrent(CCTermType Termination);
void SetStateTryWaitSnk(void);
void UpdateSourcePowerMode(void);

//static int FUSB300Int_PIN_LVL(void)
//{
//	return 0;
//}

void FUSB302_start_timer(struct hrtimer* timer, unsigned short ms)
{
    ktime_t ktime;
    ktime = ktime_set(0, FUSB_MS_TO_NS(ms));
    hrtimer_start(timer, ktime, HRTIMER_MODE_REL);
}

enum hrtimer_restart func_hrtimer(struct hrtimer *timer)
{
    if (timer == &state_hrtimer)
	{
		FUSB_LOG("state_hrtimer has expired...\n");
        StateTimer = 0;
	}
    else if (timer == &debounce_hrtimer1)
	{
		FUSB_LOG("debounce_hrtimer1 timer has expired...\n");
        DebounceTimer1 = 0;
	}
    else if (timer == &debounce_hrtimer2)
	{
		FUSB_LOG("debounce_hrtimer2 timer has expired...\n");
        DebounceTimer2 = 0;
	}
    else if (timer == &toggle_hrtimer )
	{
		FUSB_LOG("toggle_hrtimer has expired...\n");
        ToggleTimer = 0;
	}
	else
	{
		/* Do nothing */
	}

    state_changed = TRUE;
    wake_up(&fusb_thread_wq);

    return HRTIMER_NORESTART;
}

void wake_up_statemachine(char *caller_name)
{
    state_changed = TRUE;
	FUSB_LOG("%s, wake up the SM\n", caller_name);
    wake_up_interruptible(&fusb_thread_wq);
}

/*******************************************************************************
 * Function:        InitializeFUSB300Variables
 * Input:           None
 * Return:          None
 * Description:     Initializes the FUSB300 state machine variables
 ******************************************************************************/
void InitializeFUSB300Variables(void)
{
    blnSMEnabled = TRUE;                // Disable the FUSB300 state machine by default
    blnAccSupport = FALSE;              // Disable accessory support by default
    blnSrcPreferred = FALSE;            // Clear the source preferred flag by default
    PortType = USBTypeC_DRP;            // Initialize to a dual-role port by default
    ConnState = Disabled;               // Initialize to the disabled state?
	FUSB_LOG("set con_stat ====> %s, %s, %d\n", con_stat[ConnState], __func__, __LINE__);
    blnINTActive = FALSE;               // Clear the handle interrupt flag
    blnCCPinIsCC1 = FALSE;              // Clear the flag to indicate CC1 is CC
    blnCCPinIsCC2 = FALSE;              // Clear the flag to indicate CC2 is CC

    StateTimer = USHRT_MAX;             // Disable the state timer
    DebounceTimer1 = USHRT_MAX;         // Disable the 1st debounce timer
    DebounceTimer2 = USHRT_MAX;         // Disable the 2nd debounce timer
    ToggleTimer = USHRT_MAX;            // Disable the toggle timer
    CC1TermDeb = CCTypeNone;            // Set the CC1 termination type to none initially
    CC2TermDeb = CCTypeNone;            // Set the CC2 termination type to none initially
    CC1TermAct = CC1TermDeb;            // Initialize the active CC1 value
    CC2TermAct = CC2TermDeb;            // Initialize the active CC2 value
    SinkCurrent = utccNone;             // Clear the current advertisement initially
    SourceCurrent = utccDefault;        // Set the current advertisement to the default
    Registers.DeviceID.byte = 0x00;     // Clear
    Registers.Switches.byte[0] = 0x03;  // Only enable the device pull-downs by default
    Registers.Switches.byte[1] = 0x00;  // Disable the BMC transmit drivers
    Registers.Measure.byte = 0x00;      // Clear
    Registers.Slice.byte = SDAC_DEFAULT;// Set the SDAC threshold to ~0.544V by default (from FUSB302)
    Registers.Control.byte[0] = 0x20;   // Set to mask all interrupts by default (from FUSB302)
    Registers.Control.byte[1] = 0x00;   // Clear
    Registers.Control.byte[2] = 0x02;   //
    Registers.Control.byte[3] = 0x06;   //
    Registers.Mask.byte = 0x00;         // Clear
    Registers.Power.byte = 0x07;        // Initialize to everything enabled except oscillator
    Registers.Status.Status = 0;        // Clear status bytes
    Registers.Status.StatusAdv = 0;     // Clear the advanced status bytes
    Registers.Status.Interrupt1 = 0;    // Clear the interrupt1 byte
    Registers.Status.InterruptAdv = 0;  // Clear the advanced interrupt bytes
    USBPDActive = FALSE;                // Clear the USB PD active flag
    USBPDEnabled = TRUE;                // Clear the USB PD enabled flag until enabled by the host
    PRSwapTimer = 0;                    // Clear the PR Swap timer

    state_changed = FALSE;
    hrtimer_init(&state_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    state_hrtimer.function = func_hrtimer;

    hrtimer_init(&debounce_hrtimer1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    debounce_hrtimer1.function = func_hrtimer;

    hrtimer_init(&debounce_hrtimer2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    debounce_hrtimer2.function = func_hrtimer;

    hrtimer_init(&toggle_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    toggle_hrtimer.function = func_hrtimer;
}

void InitializeFUSB300(void)
{
    FUSB300Read(regDeviceID, 2, &Registers.DeviceID.byte);          // Read the device ID
    FUSB300Read(regSlice, 1, &Registers.Slice.byte);                // Read the slice
    Registers.Mask.byte = 0x44;                                     // Mask I_ACTIVITY and I_WAKE to reduce system load
    FUSB300Write(regMask, 1, &Registers.Mask.byte); // Clear all interrupt masks (we want to do something with them)
    // Update the control and power values since they will be written in either the Unattached.UFP or Unattached.DFP states
    Registers.Control.dword = 0x06220004; // Reset all back to default, but clear the INT_MASK bit
    switch (PortType)
    {
        case USBTypeC_Sink:
            Registers.Control.MODE = 0b10;
            break;
        case USBTypeC_Source:
            Registers.Control.MODE = 0b11;
            break;
        default:
            Registers.Control.MODE = 0b01;
            break;
    }
    FUSB300Write(regControl2, 2, &Registers.Control.byte[2]);       // Update the control registers for toggling
    Registers.Control4.TOG_USRC_EXIT = 1;                           // Stop toggling with Ra/Ra
    FUSB300Write(regControl4, 1, &Registers.Control4.byte);         // Commit to the device
    Registers.Power.byte = 0x01;                                    // Initialize such that only the bandgap and wake circuit are enabled by default
    FUSB300Read(regStatus0a, 2, &Registers.Status.byte[0]);         // Read the advanced status registers to make sure we are in sync with the device
    FUSB300Read(regStatus0, 2, &Registers.Status.byte[4]);          // Read the standard status registers to make sure we are in sync with the device
    // Do not read any of the interrupt registers, let the state machine handle those
    SetStateDelayUnattached();
}

void DisableFUSB300StateMachine(void)
{
    blnSMEnabled = FALSE;
    SetStateDisabled();
    //InitializeFUSB300Timer(FALSE);
}

void EnableFUSB300StateMachine(void)
{
    InitializeFUSB300();
    blnSMEnabled = TRUE;
}

void StateMachineFUSB300(void)
{
	int i;

#ifdef PD_SUPPORT
    //FUSB_LOG("connstate=%d, policyState=%d, protocolState=%d, PDTxStatus=%d\n",
     //   ConnState, PolicyState, ProtocolState, PDTxStatus);
#endif
    //if (!blnSMEnabled)
	//{
		//pr_err("SM is not enable\n");
        //return;
	//}

	/* Read 7 byte start from status0a into Registers.Status */
	FUSB300Read(regStatus0a, 7, &Registers.Status.byte[0]);
	for (i = 0; i < 7; i ++)
		pr_debug("reg[0x%02x] = 0x%02x\n", i + 0x3C, Registers.Status.byte[i]);

#ifdef PD_SUPPORT
    if (USBPDActive) // Only call the USB PD routines if we have enabled the block
    {
        USBPDProtocol(); // Call the protocol state machine to handle any timing critical operations
        USBPDPolicyEngine(); // Once we have handled any Type-C and protocol events, call the USB PD Policy Engine
    }
#endif

	FUSB_LOG("con_stat ====> %s\n", con_stat[ConnState]);
    switch (ConnState)
    {
        case Disabled:
            StateMachineDisabled();
            break;
        case ErrorRecovery:
            StateMachineErrorRecovery();
            break;
        case Unattached:
            StateMachineUnattached();
            break;
        case AttachWaitSink:
            StateMachineAttachWaitSnk();
            break;
        case AttachedSink:
            StateMachineAttachedSink();
            break;
        case AttachWaitSource:
            StateMachineAttachWaitSrc();
            break;
        case AttachedSource:
            StateMachineAttachedSource();
            break;
        case TrySource:
            StateMachineTrySrc();
            break;
        case TryWaitSink:
            StateMachineTryWaitSnk();
            break;
        case AudioAccessory:
            StateMachineAudioAccessory();
            break;
        case DebugAccessory:
            StateMachineDebugAccessory();
            break;
        case AttachWaitAccessory:
            StateMachineAttachWaitAcc();
            break;
        case PoweredAccessory:
            StateMachinePoweredAccessory();
            break;
        case UnsupportedAccessory:
            StateMachineUnsupportedAccessory();
            break;
        case DelayUnattached:
            StateMachineDelayUnattached();
            break;
        default:
            SetStateDelayUnattached(); // We shouldn't get here, so go to the unattached state just in case
            break;
    }
    Registers.Status.Interrupt1 = 0; // Clear the interrupt register once we've gone through the state machines
    Registers.Status.InterruptAdv = 0; // Clear the advanced interrupt registers once we've gone through the state machines
}

void StateMachineDisabled(void)
{
    // Do nothing until directed to go to some other state...
}

void StateMachineErrorRecovery(void)
{
    if (StateTimer == 0)
    {
        SetStateDelayUnattached();
    }
}

void StateMachineDelayUnattached(void)
{
    if (StateTimer == 0)
    {
		FUSB_LOG("%s, %d\n", __func__, __LINE__);
        SetStateUnattached();
    }
}

void StateMachineUnattached(void)
{
    if (Registers.Status.I_TOGDONE)
    {
        switch (Registers.Status.TOGSS)
        {
            case 0b101: // Rp detected on CC1
				FUSB_LOG("Port partner CC1 is Rp\n");
                blnCCPinIsCC1 = TRUE;
                blnCCPinIsCC2 = FALSE;
                SetStateAttachWaitSnk(); // Go to the AttachWaitSnk state
                break;
            case 0b110: // Rp detected on CC2
				FUSB_LOG("Port partner CC2 is Rp\n");
                blnCCPinIsCC1 = FALSE;
                blnCCPinIsCC2 = TRUE;
                SetStateAttachWaitSnk(); // Go to the AttachWaitSnk state
                break;
            case 0b001: // Rd detected on CC1
				FUSB_LOG("Port partner CC1 is Rd\n");
                blnCCPinIsCC1 = TRUE;
                blnCCPinIsCC2 = FALSE;

				/* If we are configured as a sink and support accessories */
                if ((PortType == USBTypeC_Sink) && (blnAccSupport))
                    SetStateAttachWaitAcc(); // Go to the AttachWaitAcc state
                else // Otherwise we must be configured as a source or DRP
                    SetStateAttachWaitSrc(); // So go to the AttachWaitSnk state
                break;
            case 0b010: // Rd detected on CC2
				FUSB_LOG("Port partner CC2 is Rd\n");
                blnCCPinIsCC1 = FALSE;
                blnCCPinIsCC2 = TRUE;

				/* If we are configured as a sink and support accessories */
                if ((PortType == USBTypeC_Sink) && (blnAccSupport))
                    SetStateAttachWaitAcc(); // Go to the AttachWaitAcc state
                else // Otherwise we must be configured as a source or DRP
                    SetStateAttachWaitSrc(); // So go to the AttachWaitSnk state
                break;
            case 0b111: // Ra detected on both CC1 and CC2
				printk("Port partner CC1 and CC2 is Ra\n");
                blnCCPinIsCC1 = FALSE;
                blnCCPinIsCC2 = FALSE;
				/* If we are configured as a sink and support accessories */
                if ((PortType == USBTypeC_Sink) && (blnAccSupport))
                    SetStateAttachWaitAcc(); // Go to the AttachWaitAcc state
                else // Otherwise we must be configured as a source or DRP
                    SetStateAttachWaitSrc(); // So go to the AttachWaitSnk state
                break;
            default:    // Shouldn't get here, but just in case reset everything...
				FUSB_LOG("Shouldn't get here, but just in case reset everything\n");
                Registers.Control.TOGGLE = 0; // Disable the toggle in order to clear...
                FUSB300Write(regControl2, 1, &Registers.Control.byte[2]); // Commit the control state
                Delay10us(1);

                /* Re-enable the toggle state machine... (allows us to get another I_TOGDONE interrupt) */
                Registers.Control.TOGGLE = 1;
                FUSB300Write(regControl2, 1, &Registers.Control.byte[2]); // Commit the control state
                break;
        }
    }
	else
	{
		FUSB_LOG("%s, %d,waiting for toggle done status!!\n", __FUNCTION__, __LINE__);
	}
}

void StateMachineAttachWaitSnk(void)
{
	char *caller_name = "AttachWaitSnk";
    // If the both CC lines has been open for tPDDebounce, go to the unattached state
    // If VBUS and the we've been Rd on exactly one pin for 100ms... go to the attachsnk state
    CCTermType CCValue = DecodeCCTermination(caller_name); // Grab the latest CC termination value
    if (Registers.Switches.MEAS_CC1) // If we are looking at CC1
    {
		FUSB_LOG("Use the measure block to monitor or measure the voltage on CC1\n");
        if (CC1TermAct != CCValue) // Check to see if the value has changed...
        {
            CC1TermAct = CCValue; // If it has, update the value
            DebounceTimer1 = tPDDebounceMin; // Restart the debounce timer with tPDDebounce (wait 10ms before detach)
            FUSB302_start_timer(&debounce_hrtimer1, DebounceTimer1);
        }
    }
    else // Otherwise we are looking at CC2
    {
		FUSB_LOG("Use the measure block to monitor or measure the voltage on CC2\n");
        if (CC2TermAct != CCValue) // Check to see if the value has changed...
        {
            CC2TermAct = CCValue; // If it has, update the value
            DebounceTimer1 = tPDDebounceMin; // Restart the debounce timer with tPDDebounce (wait 10ms before detach)
            FUSB302_start_timer(&debounce_hrtimer1, DebounceTimer1);
        }
    }
    if (DebounceTimer1 == 0) // Check to see if our debounce timer has expired...
    {
		/* If it has, disable it so we don't come back in here until we have debounced a change in state */
        DebounceTimer1 = USHRT_MAX;
        if ((CC1TermDeb != CC1TermAct) || (CC2TermDeb != CC2TermAct)) {
			/* Once the CC state is known, start the tCCDebounce timer to validate */
            DebounceTimer2 = tCCDebounceMin - tPDDebounceMin;
			FUSB_LOG("CC state is known\n");
            FUSB302_start_timer(&debounce_hrtimer2, DebounceTimer2);
        }
        CC1TermDeb = CC1TermAct; // Update the CC1 debounced value
        CC2TermDeb = CC2TermAct; // Update the CC2 debounced value
    }
    if (ToggleTimer == 0) // If are toggle timer has expired, it's time to swap detection
    {
		FUSB_LOG("toggle timer has expired, it's time to swap detection\n");
        if (Registers.Switches.MEAS_CC1) // If we are currently on the CC1 pin...
		{
			FUSB_LOG("currently on the CC1 pin,Toggle over to look at CC2\n");
            ToggleMeasureCC2();
		}
        else
		{
			FUSB_LOG("currently on the CC2 pin,Toggle over to look at CC1\n");
            ToggleMeasureCC1();
		}

		/*
		 * Reset the toggle timer to our default toggling
		 * (<tPDDebounce to avoid disconnecting the other side when we remove pull-ups)
		 */
        ToggleTimer = tFUSB302Toggle;
		FUSB302_start_timer(&toggle_hrtimer, ToggleTimer);
    }

	FUSB_LOG("CC1TermDeb = %s, CC2TermDeb = %s\n",cc_term_type[CC1TermDeb], cc_term_type[CC2TermDeb]);
	FUSB_LOG("VBUSOK = 0x%x, DebounceTimer2 = %d\n", Registers.Status.VBUSOK, DebounceTimer2);
	// If we have detected SNK.Open for atleast tPDDebounce on both pins...
    if ((CC1TermDeb == CCTypeRa) && (CC2TermDeb == CCTypeRa))
        SetStateDelayUnattached();// Go to the unattached state
    else if (Registers.Status.VBUSOK && (DebounceTimer2 == 0))// VBUS and Rp detected  for >tCCDebounce
    {
		FUSB_LOG("VBUS and Rp detected for > tCCDebounce\n");
        if ((CC1TermDeb > CCTypeRa) && (CC2TermDeb == CCTypeRa))// If Rp is detected on CC1
        {
			// If we are configured as a DRP and prefer the source role...
            if ((PortType == USBTypeC_DRP) && blnSrcPreferred)
                SetStateTrySrc(); // Go to the Try.Src state
            else // Otherwise we are free to attach as a sink
            {
                blnCCPinIsCC1 = TRUE;                                           // Set the CC pin to CC1
                blnCCPinIsCC2 = FALSE;
                SetStateAttachedSink(); // Go to the Attached.Snk state
            }
        }
        else if ((CC1TermDeb == CCTypeRa) && (CC2TermDeb > CCTypeRa))           // If Rp is detected on CC2
        {
			/* If we are configured as a DRP and prefer the source role */
            if ((PortType == USBTypeC_DRP) && blnSrcPreferred)
                SetStateTrySrc(); // Go to the Try.Src state
            else // Otherwise we are free to attach as a sink
            {
                blnCCPinIsCC1 = FALSE;                                          //
                blnCCPinIsCC2 = TRUE;                                           // Set the CC pin to CC2
                SetStateAttachedSink();                                         // Go to the Attached.Snk State
            }
        }
    }
}

void StateMachineAttachWaitSrc(void)
{
	char *caller_name = "AttachWaitSrc";
    CCTermType CCValue = DecodeCCTermination(caller_name); // Grab the latest CC termination value
    if (Registers.Switches.MEAS_CC1) // If we are looking at CC1
    {
        if (CC1TermAct != CCValue) // Check to see if the value has changed...
        {
            CC1TermAct = CCValue; // If it has, update the value
            DebounceTimer1 = tPDDebounceMin; // Restart the debounce timer (tPDDebounce)
            FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
        }
    }
    else // Otherwise we are looking at CC2
    {
        if (CC2TermAct != CCValue) // Check to see if the value has changed...
        {
            CC2TermAct = CCValue; // If it has, update the value
            DebounceTimer1 = tPDDebounceMin; // Restart the debounce timer (tPDDebounce)
            FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
        }
    }
    if (DebounceTimer1 == 0) // Check to see if our debounce timer has expired...
    {
        DebounceTimer1 = USHRT_MAX; // If it has, disable it so we don't come back in here until we have debounced a change in state
        if ((CC1TermDeb != CC1TermAct) || (CC2TermDeb != CC2TermAct)) {
            DebounceTimer2 = tCCDebounceMin; // Once the CC state is known, start the tCCDebounce timer to validate
            FUSB302_start_timer(&debounce_hrtimer2,DebounceTimer2);
        }
        CC1TermDeb = CC1TermAct; // Update the CC1 debounced value
        CC2TermDeb = CC2TermAct; // Update the CC2 debounced value
    }
    if (ToggleTimer == 0) // If are toggle timer has expired, it's time to swap detection
    {
        if (Registers.Switches.MEAS_CC1) // If we are currently on the CC1 pin...
            ToggleMeasureCC2();                                                 // Toggle over to look at CC2
        else  // Otherwise assume we are using the CC2...
            ToggleMeasureCC1();                                                 // So toggle over to look at CC1
        ToggleTimer = tFUSB302Toggle;                                           // Reset the toggle timer to our default toggling (<tPDDebounce to avoid disconnecting the other side when we remove pull-ups)
	FUSB302_start_timer(&toggle_hrtimer, ToggleTimer);
    }
    if ((CC1TermDeb == CCTypeNone) && (CC2TermDeb == CCTypeNone))               // If our debounced signals are both open, go to the unattached state
    {
        SetStateDelayUnattached();
    }
    else if ((CC1TermDeb == CCTypeNone) && (CC2TermDeb == CCTypeRa))            // If exactly one pin is open and the other is Ra, go to the unattached state
    {
        SetStateDelayUnattached();
    }
    else if ((CC1TermDeb == CCTypeRa) && (CC2TermDeb == CCTypeNone))            // If exactly one pin is open and the other is Ra, go to the unattached state
    {
        SetStateDelayUnattached();
    }
    else if (DebounceTimer2 == 0)                                               // Otherwise, we are checking to see if we have had a solid state for tCCDebounce
    {
        if ((CC1TermDeb == CCTypeRa) && (CC2TermDeb == CCTypeRa))               // If both pins are Ra, it's an audio accessory
            SetStateAudioAccessory();
        else if ((CC1TermDeb > CCTypeRa) && (CC2TermDeb > CCTypeRa))            // If both pins are Rd, it's a debug accessory
            SetStateDebugAccessory();
        else if (CC1TermDeb > CCTypeRa)                                         // If CC1 is Rd and CC2 is not...
        {
            blnCCPinIsCC1 = TRUE;                                               // Set the CC pin to CC1
            blnCCPinIsCC2 = FALSE;
            SetStateAttachedSrc();                                              // Go to the Attached.Src state
        }
        else if (CC2TermDeb > CCTypeRa)                                         // If CC2 is Rd and CC1 is not...
        {
            blnCCPinIsCC1 = FALSE;
            blnCCPinIsCC2 = TRUE;                                               // Set the CC pin to CC2
            SetStateAttachedSrc();                                              // Go to the Attached.Src state
        }
    }
}

void StateMachineAttachWaitAcc(void)
{
	char *caller_name = "AttachWaitAcc";
    CCTermType CCValue = DecodeCCTermination(caller_name);                                 // Grab the latest CC termination value
    if (Registers.Switches.MEAS_CC1)                                            // If we are looking at CC1
    {
        if (CC1TermAct != CCValue) // Check to see if the value has changed...
        {
            CC1TermAct = CCValue;                                               // If it has, update the value
            DebounceTimer1 = tCCDebounceNom; // Restart the debounce timer (tCCDebounce)
            FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
        }
    }
    else // Otherwise we are looking at CC2
    {
        if (CC2TermAct != CCValue) // Check to see if the value has changed...
        {
            CC2TermAct = CCValue;                                               // If it has, update the value
            DebounceTimer1 = tCCDebounceNom; // Restart the debounce timer (tCCDebounce)
            FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
        }
    }
    if (ToggleTimer == 0) // If are toggle timer has expired, it's time to swap detection
    {
        if (Registers.Switches.MEAS_CC1) // If we are currently on the CC1 pin...
            ToggleMeasureCC2();                                                 // Toggle over to look at CC2
        else // Otherwise assume we are using the CC2...
            ToggleMeasureCC1();                                                 // So toggle over to look at CC1
        ToggleTimer = tFUSB302Toggle;                                           // Reset the toggle timer to our default toggling (<tPDDebounce to avoid disconnecting the other side when we remove pull-ups)
        FUSB302_start_timer(&toggle_hrtimer,ToggleTimer);
    }
    if (DebounceTimer1 == 0) // Check to see if the signals have been stable for tCCDebounce
    {
        if ((CC1TermDeb == CCTypeRa) && (CC2TermDeb == CCTypeRa))               // If they are both Ra, it's an audio accessory
            SetStateAudioAccessory();
        else if ((CC1TermDeb > CCTypeRa) && (CC2TermDeb > CCTypeRa))            // If they are both Rd, it's a debug accessory
            SetStateDebugAccessory();
        else if ((CC1TermDeb == CCTypeNone) || (CC2TermDeb == CCTypeNone))      // If either pin is open, it's considered a detach
            SetStateDelayUnattached();
        else if ((CC1TermDeb > CCTypeRa) && (CC2TermDeb == CCTypeRa))           // If CC1 is Rd and CC2 is Ra, it's a powered accessory (CC1 is CC)
        {
            blnCCPinIsCC1 = TRUE;
            blnCCPinIsCC2 = FALSE;
            SetStatePoweredAccessory();
        }
        else if ((CC1TermDeb == CCTypeRa) && (CC2TermDeb > CCTypeRa))           // If CC1 is Ra and CC2 is Rd, it's a powered accessory (CC2 is CC)
        {
            blnCCPinIsCC1 = TRUE;
            blnCCPinIsCC2 = FALSE;
            SetStatePoweredAccessory();
        }
    }
}

void StateMachineAttachedSink(void)
{
	char *caller_name = "AttachedSink";
    CCTermType CCValue = DecodeCCTermination(caller_name);// Grab the latest CC termination value

	// If VBUS is removed and we are not in the middle of a power role swap...
    if ((Registers.Status.VBUSOK == FALSE) && (!PRSwapTimer))
        SetStateDelayUnattached(); // Go to the unattached state
    else
    {
        if (Registers.Switches.MEAS_CC1) // If we are looking at CC1
        {
            if (CCValue != CC1TermAct) // If the CC voltage has changed...
            {
                CC1TermAct = CCValue;                                           // Store the updated value
                DebounceTimer1 = tPDDebounceMin; // Reset the debounce timer to the minimum tPDdebounce
                FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
            }
            else if (DebounceTimer1 == 0) // If the signal has been debounced
            {
                DebounceTimer1 = USHRT_MAX;// Disable the debounce timer until we get a change
                CC1TermDeb = CC1TermAct; // Store the debounced termination for CC1
                UpdateSinkCurrent(CC1TermDeb); // Update the advertised current
            }
        }
        else
        {
            if (CCValue != CC2TermAct) // If the CC voltage has changed...
            {
                CC2TermAct = CCValue; // Store the updated value
                DebounceTimer1 = tPDDebounceMin;// Reset the debounce timer to the minimum tPDdebounce
                FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
            }
            else if (DebounceTimer1 == 0)// If the signal has been debounced
            {
                DebounceTimer1 = USHRT_MAX; // Disable the debounce timer until we get a change
                CC2TermDeb = CC2TermAct; // Store the debounced termination for CC2
                UpdateSinkCurrent(CC2TermDeb); // Update the advertised current
            }
        }
    }
}

void StateMachineAttachedSource(void)
{
	char *caller_name = "AttachedSource";
    CCTermType CCValue = DecodeCCTermination(caller_name);// Grab the latest CC termination value
    if (Registers.Switches.MEAS_CC1)// Did we detect CC1 as the CC pin?
    {
        if (CC1TermAct != CCValue)// If the CC voltage has changed...
        {
            CC1TermAct = CCValue; // Store the updated value
            DebounceTimer1 = tPDDebounceMin;// Reset the debounce timer to the minimum tPDdebounce
            FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
        }
        else if (DebounceTimer1 == 0)// If the signal has been debounced
        {
            DebounceTimer1 = USHRT_MAX;// Disable the debounce timer until we get a change
            CC1TermDeb = CC1TermAct;// Store the debounced termination for CC1
        }

		/* If the debounced CC pin is detected as open and we aren't in the middle of a PR_Swap */
        if ((CC1TermDeb == CCTypeNone) && (!PRSwapTimer))
        {
			// Check to see if we need to go to the TryWait.SNK state...
            if ((PortType == USBTypeC_DRP) && blnSrcPreferred)
                SetStateTryWaitSnk();
            else // Otherwise we are going to the unattached state
                SetStateDelayUnattached();
        }
    }
    else // We must have detected CC2 as the CC pin
    {
        if (CC2TermAct != CCValue) // If the CC voltage has changed...
        {
            CC2TermAct = CCValue;                                               // Store the updated value
            DebounceTimer1 = tPDDebounceMin; // Reset the debounce timer to the minimum tPDdebounce
            FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
        }
        else if (DebounceTimer1 == 0) // If the signal has been debounced
        {
            DebounceTimer1 = USHRT_MAX; // Disable the debounce timer until we get a change
            CC2TermDeb = CC2TermAct; // Store the debounced termination for CC1
        }

		/* If the debounced CC pin is detected as open and we aren't in the middle of a PR_Swap */
        if ((CC2TermDeb == CCTypeNone) && (!PRSwapTimer))
        {
			/* Check to see if we need to go to the TryWait.SNK state */
            if ((PortType == USBTypeC_DRP) && blnSrcPreferred)
                SetStateTryWaitSnk();
            else // Otherwise we are going to the unattached state
                SetStateDelayUnattached();
        }
    }
}

void StateMachineTryWaitSnk(void)
{
	char *caller_name = "TryWaitSnk";
    CCTermType CCValue = DecodeCCTermination(caller_name); // Grab the latest CC termination value
    if (Registers.Switches.MEAS_CC1) // If we are looking at CC1
    {
        if (CC1TermAct != CCValue) // Check to see if the value has changed...
        {
            CC1TermAct = CCValue; // If it has, update the value
            DebounceTimer1 = tPDDebounceMin; // Restart the debounce timer with tPDDebounce (wait 10ms before detach)
            FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
        }
    }
    else // Otherwise we are looking at CC2
    {
        if (CC2TermAct != CCValue) // Check to see if the value has changed...
        {
            CC2TermAct = CCValue; // If it has, update the value
            DebounceTimer1 = tPDDebounceMin; // Restart the debounce timer with tPDDebounce (wait 10ms before detach)
            FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
        }
    }
    if (DebounceTimer1 == 0) // Check to see if our debounce timer has expired...
    {
        DebounceTimer1 = USHRT_MAX; // If it has, disable it so we don't come back in here until we have debounced a change in state
        if ((CC1TermDeb != CC1TermAct) || (CC2TermDeb != CC2TermAct))
        {
            DebounceTimer2 = tCCDebounceMin - tPDDebounceMin; // Once the CC state is known, start the tCCDebounce timer to validate
            //FIXME  handle timer restart.
            FUSB302_start_timer(&debounce_hrtimer2,DebounceTimer2);
        }
        CC1TermDeb = CC1TermAct; // Update the CC1 debounced value
        CC2TermDeb = CC2TermAct; // Update the CC2 debounced value
    }
    if (ToggleTimer == 0) // If are toggle timer has expired, it's time to swap detection
    {
        if (Registers.Switches.MEAS_CC1) // If we are currently on the CC1 pin...
            ToggleMeasureCC2(); // Toggle over to look at CC2
        else // Otherwise assume we are using the CC2...
            ToggleMeasureCC1(); // So toggle over to look at CC1
        ToggleTimer = tFUSB302Toggle; // Reset the toggle timer to our default toggling (<tPDDebounce to avoid disconnecting the other side when we remove pull-ups)
        FUSB302_start_timer(&toggle_hrtimer,ToggleTimer);
    }
    if ((StateTimer == 0) && (CC1TermDeb == CCTypeRa) && (CC2TermDeb == CCTypeRa))  // If tDRPTryWait has expired and we detected open on both pins...
        SetStateDelayUnattached();                                              // Go to the unattached state
    else if (Registers.Status.VBUSOK && (DebounceTimer2 == 0))                  // If we have detected VBUS and we have detected an Rp for >tCCDebounce...
    {
        if ((CC1TermDeb > CCTypeRa) && (CC2TermDeb == CCTypeRa))                // If Rp is detected on CC1
        {
            blnCCPinIsCC1 = TRUE;                                               // Set the CC pin to CC1
            blnCCPinIsCC2 = FALSE;                                              //
            SetStateAttachedSink();                                             // Go to the Attached.Snk state
        }
        else if ((CC1TermDeb == CCTypeRa) && (CC2TermDeb > CCTypeRa))           // If Rp is detected on CC2
        {
            blnCCPinIsCC1 = FALSE;                                              //
            blnCCPinIsCC2 = TRUE;                                               // Set the CC pin to CC2
            SetStateAttachedSink();                                             // Go to the Attached.Snk State
        }
    }
}

void StateMachineTrySrc(void)
{
	char *caller_name = "TrySrc";
    CCTermType CCValue = DecodeCCTermination(caller_name); // Grab the latest CC termination value
    if (Registers.Switches.MEAS_CC1) // If we are looking at CC1
    {
        if (CC1TermAct != CCValue) // Check to see if the value has changed...
        {
            CC1TermAct = CCValue; // If it has, update the value
            DebounceTimer1 = tPDDebounceMin; // Restart the debounce timer (tPDDebounce)
            FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
        }
    }
    else // Otherwise we are looking at CC2
    {
        if (CC2TermAct != CCValue) // Check to see if the value has changed...
        {
            CC2TermAct = CCValue; // If it has, update the value
            DebounceTimer1 = tPDDebounceMin; // Restart the debounce timer (tPDDebounce)
            FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
        }
    }
    if (DebounceTimer1 == 0) // Check to see if our debounce timer has expired...
    {
        DebounceTimer1 = USHRT_MAX; // If it has, disable it so we don't come back in here until a new debounce value is ready
        CC1TermDeb = CC1TermAct; // Update the CC1 debounced value
        CC2TermDeb = CC2TermAct; // Update the CC2 debounced value
    }
    if (ToggleTimer == 0) // If are toggle timer has expired, it's time to swap detection
    {
        if (Registers.Switches.MEAS_CC1) // If we are currently on the CC1 pin...
            ToggleMeasureCC2(); // Toggle over to look at CC2
        else // Otherwise assume we are using the CC2...
            ToggleMeasureCC1();  // So toggle over to look at CC1
        ToggleTimer = tPDDebounceMax; // Reset the toggle timer to the max tPDDebounce to ensure the other side sees the pull-up for the min tPDDebounce
        FUSB302_start_timer(&toggle_hrtimer,ToggleTimer);
    }
    if ((CC1TermDeb > CCTypeRa) && ((CC2TermDeb == CCTypeNone) || (CC2TermDeb == CCTypeRa)))    // If the CC1 pin is Rd for atleast tPDDebounce...
    {
        blnCCPinIsCC1 = TRUE; // The CC pin is CC1
        blnCCPinIsCC2 = FALSE;
        SetStateAttachedSrc(); // Go to the Attached.Src state
    }
    else if ((CC2TermDeb > CCTypeRa) && ((CC1TermDeb == CCTypeNone) || (CC1TermDeb == CCTypeRa)))   // If the CC2 pin is Rd for atleast tPDDebounce...
    {
        blnCCPinIsCC1 = FALSE;                                                  // The CC pin is CC2
        blnCCPinIsCC2 = TRUE;
        SetStateAttachedSrc();                                                  // Go to the Attached.Src state
    }
    else if (StateTimer == 0) // If we haven't detected Rd on exactly one of the pins and we have waited for tDRPTry...
        SetStateTryWaitSnk(); // Move onto the TryWait.Snk state to not get stuck in here
}

void StateMachineDebugAccessory(void)
{
	char *caller_name = "DebugAccessory";
    CCTermType CCValue = DecodeCCTermination(caller_name);                                 // Grab the latest CC termination value
    if (CC1TermAct != CCValue) // If the CC voltage has changed...
    {
        CC1TermAct = CCValue;  // Store the updated value
        DebounceTimer1 = tCCDebounceMin; // Reset the debounce timer to the minimum tCCDebounce
        FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
    }
    else if (DebounceTimer1 == 0) // If the signal has been debounced
    {
        DebounceTimer1 = USHRT_MAX; // Disable the debounce timer until we get a change
        CC1TermDeb = CC1TermAct; // Store the debounced termination for CC1
    }
    if (CC1TermDeb == CCTypeNone) // If we have detected an open for > tCCDebounce
        SetStateDelayUnattached(); // Go to the unattached state
}

void StateMachineAudioAccessory(void)
{
	char *caller_name = "AudioAccessory";
    CCTermType CCValue = DecodeCCTermination(caller_name);                                 // Grab the latest CC termination value
    if (CC1TermAct != CCValue) // If the CC voltage has changed...
    {
        CC1TermAct = CCValue;  // Store the updated value
        DebounceTimer1 = tCCDebounceMin;  // Reset the debounce timer to the minimum tCCDebounce
        FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
    }
    else if (DebounceTimer1 == 0) // If the signal has been debounced
    {
        DebounceTimer1 = USHRT_MAX; // Disable the debounce timer until we get a change
        CC1TermDeb = CC1TermAct; // Store the debounced termination for CC1
    }
    if (CC1TermDeb == CCTypeNone) // If we have detected an open for > tCCDebounce
        SetStateDelayUnattached(); // Go to the unattached state
}

void StateMachinePoweredAccessory(void)
{
	char *caller_name = "PoweredAccessory";
    CCTermType CCValue = DecodeCCTermination(caller_name);                                 // Grab the latest CC termination value
    if (CC1TermAct != CCValue)     // If the CC voltage has changed...
    {
        CC1TermAct = CCValue;                                                   // Store the updated value
        DebounceTimer1 = tPDDebounceMin; // Reset the debounce timer to the minimum tPDdebounce
        FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
    }
    else if (DebounceTimer1 == 0)       // If the signal has been debounced
    {
        DebounceTimer1 = USHRT_MAX;// Disable the debounce timer until we get a change
        CC1TermDeb = CC1TermAct;   // Store the debounced termination for CC1
    }
    if (CC1TermDeb == CCTypeNone)  // If we have detected an open for > tCCDebounce
        SetStateDelayUnattached();                                              // Go to the unattached state
    else if (StateTimer == 0)                                                   // If we have timed out (tAMETimeout) and haven't entered an alternate mode...
        SetStateUnsupportedAccessory();                                         // Go to the Unsupported.Accessory state
}

void StateMachineUnsupportedAccessory(void)
{
	char *caller_name = "UnsupportedAccessory";
    CCTermType CCValue = DecodeCCTermination(caller_name);                                 // Grab the latest CC termination value
    if (CC1TermAct != CCValue)                                                  // If the CC voltage has changed...
    {
        CC1TermAct = CCValue;                                                   // Store the updated value
        DebounceTimer1 = tPDDebounceMin;                                        // Reset the debounce timer to the minimum tPDDebounce
        FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
    }
    else if (DebounceTimer1 == 0)                                               // If the signal has been debounced
    {
        DebounceTimer1 = USHRT_MAX;                                             // Disable the debounce timer until we get a change
        CC1TermDeb = CC1TermAct;                                                // Store the debounced termination for CC1
    }
    if (CC1TermDeb == CCTypeNone)                                               // If we have detected an open for > tCCDebounce
        SetStateDelayUnattached();                                              // Go to the unattached state
}

/////////////////////////////////////////////////////////////////////////////
//                      State Machine Configuration
/////////////////////////////////////////////////////////////////////////////

void SetStateDisabled(void)
{
//	char *caller_name = "SetStateDisabled";
    VBUS_5V_EN = 0;                                                 // Disable the 5V output...
    VBUS_12V_EN = 0;                                                // Disable the 12V output
    Registers.Power.PWR = 0x01;                                     // Enter low power state
    Registers.Control.TOGGLE = 0;                                   // Disable the toggle state machine
    Registers.Control.HOST_CUR = 0x00;                              // Disable the currents for the pull-ups (not used for UFP)
    Registers.Switches.word = 0x0000;                               // Disable all pull-ups and pull-downs on the CC pins and disable the BMC transmitters
    FUSB300Write(regPower, 1, &Registers.Power.byte);               // Commit the power state
    FUSB300Write(regControl0, 3, &Registers.Control.byte[0]);       // Commit the control state
    FUSB300Write(regSwitches0, 2, &Registers.Switches.byte[0]);     // Commit the switch state
#ifdef PD_SUPPORT
    USBPDDisable(FALSE);                                            // Disable the USB PD state machine (no need to write FUSB300 again since we are doing it here)
#endif
    CC1TermDeb = CCTypeNone;                                        // Clear the debounced CC1 state
    CC2TermDeb = CCTypeNone;                                        // Clear the debounced CC2 state
    CC1TermAct = CC1TermDeb;                                        // Clear the active CC1 state
    CC2TermAct = CC2TermDeb;                                        // Clear the active CC2 state
    blnCCPinIsCC1 = FALSE;                                          // Clear the CC1 pin flag
    blnCCPinIsCC2 = FALSE;                                          // Clear the CC2 pin flag
    ConnState = Disabled;                                           // Set the state machine variable to Disabled
	FUSB_LOG("set con_stat ====> %s, %s, %d\n", con_stat[ConnState], __func__, __LINE__);
    StateTimer = USHRT_MAX;                                         // Disable the state timer (not used in this state)
    DebounceTimer1 = USHRT_MAX;                                     // Disable the 1st level debounce timer
    DebounceTimer2 = USHRT_MAX;                                     // Disable the 2nd level debounce timer
    ToggleTimer = USHRT_MAX;                                        // Disable the toggle timer
    //wake_up_statemachine(caller_name);
}

void SetStateErrorRecovery(void)
{
	char *caller_name = "SetStateErrorRecovery";
    VBUS_5V_EN = 0;                                                 // Disable the 5V output...
    VBUS_12V_EN = 0;                                                // Disable the 12V output
    Registers.Power.PWR = 0x01;                                     // Enter low power state
    Registers.Control.TOGGLE = 0;                                   // Disable the toggle state machine
    Registers.Control.HOST_CUR = 0x00;                              // Disable the currents for the pull-ups (not used for UFP)
    Registers.Switches.word = 0x0000;                               // Disable all pull-ups and pull-downs on the CC pins and disable the BMC transmitters
    FUSB300Write(regPower, 1, &Registers.Power.byte);               // Commit the power state
    FUSB300Write(regControl0, 3, &Registers.Control.byte[0]);       // Commit the control state
    FUSB300Write(regSwitches0, 2, &Registers.Switches.byte[0]);     // Commit the switch state
#ifdef PD_SUPPORT
    USBPDDisable(FALSE);                                            // Disable the USB PD state machine (no need to write FUSB300 again since we are doing it here)
#endif
    CC1TermDeb = CCTypeNone;                                        // Clear the debounced CC1 state
    CC2TermDeb = CCTypeNone;                                        // Clear the debounced CC2 state
    CC1TermAct = CC1TermDeb;                                        // Clear the active CC1 state
    CC2TermAct = CC2TermDeb;                                        // Clear the active CC2 state
    blnCCPinIsCC1 = FALSE;                                          // Clear the CC1 pin flag
    blnCCPinIsCC2 = FALSE;                                          // Clear the CC2 pin flag
    ConnState = ErrorRecovery;                                      // Set the state machine variable to ErrorRecovery
	FUSB_LOG("set con_stat ====> %s, %s, %d\n", con_stat[ConnState], __func__, __LINE__);
    StateTimer = tErrorRecovery;                                    // Load the tErrorRecovery duration into the state transition timer
    FUSB302_start_timer(&state_hrtimer, StateTimer);
    DebounceTimer1 = USHRT_MAX;                                     // Disable the 1st level debounce timer
    DebounceTimer2 = USHRT_MAX;                                     // Disable the 2nd level debounce timer
    ToggleTimer = USHRT_MAX;                                        // Disable the toggle timer
    wake_up_statemachine(caller_name);
}

void SetStateDelayUnattached(void)
{
	char *caller_name = "SetStateDelayUnattached";
    // This state is only here because of the precision timing source we have with the FPGA
    // We are trying to avoid having the toggle state machines in sync with each other
    // Causing the tDRPAdvert period to overlap causing the devices to not attach for a period of time
    VBUS_5V_EN = 0;                                                 // Disable the 5V output...
    VBUS_12V_EN = 0;                                                // Disable the 12V output
    Registers.Power.PWR = 0x01;                                     // Enter low power state
    Registers.Control.TOGGLE = 0;                                   // Disable the toggle state machine
    Registers.Control.HOST_CUR = 0x00;                              // Disable the currents for the pull-ups (not used for UFP)
    Registers.Switches.word = 0x0000;                               // Disable all pull-ups and pull-downs on the CC pins and disable the BMC transmitters
    FUSB300Write(regPower, 1, &Registers.Power.byte);               // Commit the power state
    FUSB300Write(regControl0, 3, &Registers.Control.byte[0]);       // Commit the control state
    FUSB300Write(regSwitches0, 2, &Registers.Switches.byte[0]);     // Commit the switch state
#ifdef PD_SUPPORT
    USBPDDisable(FALSE);                                            // Disable the USB PD state machine (no need to write FUSB300 again since we are doing it here)
#endif
    CC1TermDeb = CCTypeNone;                                        // Clear the debounced CC1 state
    CC2TermDeb = CCTypeNone;                                        // Clear the debounced CC2 state
    CC1TermAct = CC1TermDeb;                                        // Clear the active CC1 state
    CC2TermAct = CC2TermDeb;                                        // Clear the active CC2 state
    blnCCPinIsCC1 = FALSE;                                          // Clear the CC1 pin flag
    blnCCPinIsCC2 = FALSE;                                          // Clear the CC2 pin flag
    ConnState = DelayUnattached;                                    // Set the state machine variable to delayed unattached
	FUSB_LOG("set con_stat ====> %s, %s, %d\n", con_stat[ConnState], __func__, __LINE__);
    StateTimer = 10;                                       // Set the state timer to a random value to not synchronize the toggle start (use a multiple of RAND_MAX+1 as the modulus operator)
    FUSB302_start_timer(&state_hrtimer, StateTimer);
    DebounceTimer1 = USHRT_MAX;                                     // Disable the 1st level debounce timer
    DebounceTimer2 = USHRT_MAX;                                     // Disable the 2nd level debounce timer
    ToggleTimer = USHRT_MAX;                                        // Disable the toggle timer
    wake_up_statemachine(caller_name);
}

/*
 * This function configures the Toggle state machine in the FUSB302 to handle all of the unattached states.
 * This allows for the MCU to be placed in a low power mode,
 * until the FUSB302 wakes it up upon detecting something
 */
void SetStateUnattached(void)
{
	char *caller_name = "SetStateUnattached";
    VBUS_5V_EN = 0;                                                 // Disable the 5V output...
    VBUS_12V_EN = 0;                                                // Disable the 12V output
    Registers.Control.HOST_CUR = 0x01; // Enable the defauult host current for the pull-ups (regardless of mode)
    Registers.Control.TOGGLE = 1;                                   // Enable the toggle
    if ((PortType == USBTypeC_DRP) || (blnAccSupport))              // If we are a DRP or supporting accessories
        Registers.Control.MODE = 0b01; // We need to enable the toggling functionality for Rp/Rd
    else if (PortType == USBTypeC_Source)                           // If we are strictly a Source
        Registers.Control.MODE = 0b11;                              // We just need to look for Rd
    else                                                            // Otherwise we are a UFP
        Registers.Control.MODE = 0b10;                              // So we need to only look for Rp

    Registers.Switches.word = 0x0003; // Enable the pull-downs on the CC pins, toggle overrides anyway
    Registers.Power.PWR = 0x07; // Enable everything except internal oscillator
    Registers.Measure.MDAC = MDAC_2P05V;                            // Set up DAC threshold to 2.05V
    FUSB300Write(regPower, 1, &Registers.Power.byte);               // Commit the power state
    FUSB300Write(regControl0, 3, &Registers.Control.byte[0]);       // Commit the control state
    FUSB300Write(regSwitches0, 2, &Registers.Switches.byte[0]);     // Commit the switch state
    FUSB300Write(regMeasure, 1, &Registers.Measure.byte);           // Commit the DAC threshold

#ifdef PD_SUPPORT
	/* Disable the USB PD state machine (no need to write FUSB300 again since we are doing it here) */
    USBPDDisable(FALSE);
#endif

    ConnState = Unattached; // Set the state machine variable to unattached
	FUSB_LOG("set con_stat ====> %s, %s, %d\n", con_stat[ConnState], __func__, __LINE__);
    SinkCurrent = utccNone;
    CC1TermDeb = CCTypeNone;                                        // Clear the termination for this state
    CC2TermDeb = CCTypeNone;                                        // Clear the termination for this state
    CC1TermAct = CC1TermDeb;
    CC2TermAct = CC2TermDeb;
    blnCCPinIsCC1 = FALSE;                                          // Clear the CC1 pin flag
    blnCCPinIsCC2 = FALSE;                                          // Clear the CC2 pin flag
    StateTimer = USHRT_MAX; // Disable the state timer, not used in this state
    DebounceTimer1 = USHRT_MAX; // Disable the 1st level debounce timer, not used in this state
    DebounceTimer2 = USHRT_MAX; // Disable the 2nd level debounce timer, not used in this state
    ToggleTimer = USHRT_MAX; // Disable the toggle timer, not used in this state
    wake_up_statemachine(caller_name);

	/* mimic the DFP turn off the vbus */
	dfp_vbus_switch(0);
}

void SetStateAttachWaitSnk(void)
{
	char *caller_name = "AttachWaitSnk";
    VBUS_5V_EN = 0;                                                 // Disable the 5V output...
    VBUS_12V_EN = 0;                                                // Disable the 12V output
    Registers.Power.PWR = 0x07; // Enable everything except internal oscillator
    Registers.Switches.word = 0x0003;                               // Enable the pull-downs on the CC pins

    if (blnCCPinIsCC1)
        Registers.Switches.MEAS_CC1 = 1;
    else
        Registers.Switches.MEAS_CC2 = 1;

    Registers.Measure.MDAC = MDAC_2P05V;                            // Set up DAC threshold to 2.05V
    Registers.Control.HOST_CUR = 0x00;                              // Disable the host current
    Registers.Control.TOGGLE = 0;                                   // Disable the toggle
    FUSB300Write(regPower, 1, &Registers.Power.byte);               // Commit the power state
    FUSB300Write(regSwitches0, 2, &Registers.Switches.byte[0]);     // Commit the switch state
    FUSB300Write(regMeasure, 1, &Registers.Measure.byte);           // Commit the DAC threshold
    FUSB300Write(regControl0, 3, &Registers.Control.byte[0]);       // Commit the host current
    Delay10us(25); // Delay the reading of the COMP and BC_LVL to allow time for settling
    FUSB300Read(regStatus0, 2, &Registers.Status.byte[4]);// Read the current state of the BC_LVL and COMP
    ConnState = AttachWaitSink; // Set the state machine variable to AttachWait.Snk
	FUSB_LOG("set con_stat ====> %s, %s, %d\n", con_stat[ConnState], __func__, __LINE__);

	/* Set the current advertisment variable to none until we determine what the current is */
    SinkCurrent = utccNone;
    if (Registers.Switches.MEAS_CC1)// If CC1 is what initially got us into the wait state...
    {
		FUSB_LOG("Use the measure block to monitor or measure the voltage on CC1\n");
        CC1TermAct = DecodeCCTermination(caller_name);// Determine what is initially on CC1
        CC2TermAct = CCTypeNone;// Put something that we shouldn't see on the CC2 to force a debouncing
    }
    else
    {
		FUSB_LOG("Use the measure block to monitor or measure the voltage on CC2\n");
        CC1TermAct = CCTypeNone;// Put something that we shouldn't see on the CC1 to force a debouncing
        CC2TermAct = DecodeCCTermination(caller_name);// Determine what is initially on CC2
    }
    CC1TermDeb = CCTypeNone;                                        // Initially set to invalid
    CC2TermDeb = CCTypeNone;                                        // Initially set to invalid
    StateTimer = USHRT_MAX; // Disable the state timer, not used in this state
    DebounceTimer1 = tPDDebounceMax; // Set the tPDDebounce for validating signals to transition to
    FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
    DebounceTimer2 = USHRT_MAX; // Disable the 2nd level debouncing until the first level has been debounced
    ToggleTimer = tFUSB302Toggle; // Set the toggle timer to look at each pin for tFUSB302Toggle duration
    FUSB302_start_timer(&toggle_hrtimer,ToggleTimer);
    wake_up_statemachine(caller_name);
}

void SetStateAttachWaitSrc(void)
{
	char *caller_name = "AttachWaitSrc";
    VBUS_5V_EN = 0;                                                 // Disable the 5V output...
    VBUS_12V_EN = 0;                                                // Disable the 12V output
    Registers.Power.PWR = 0x07;                                     // Enable everything except internal oscillator
    Registers.Switches.word = 0x0000;                               // Clear the register for the case below
    if (blnCCPinIsCC1)                                              // If we detected CC1 as an Rd
        Registers.Switches.word = 0x0044;                           // Enable CC1 pull-up and measure
    else
        Registers.Switches.word = 0x0088;                           // Enable CC2 pull-up and measure
    SourceCurrent = utccDefault;                                    // Set the default current level
    UpdateSourcePowerMode();                                        // Update the settings for the FUSB302
    Registers.Control.TOGGLE = 0;                                   // Disable the toggle
    FUSB300Write(regPower, 1, &Registers.Power.byte);               // Commit the power state
    FUSB300Write(regSwitches0, 2, &Registers.Switches.byte[0]);     // Commit the switch state
    FUSB300Write(regMeasure, 1, &Registers.Measure.byte);           // Commit the DAC threshold
    FUSB300Write(regControl2, 1, &Registers.Control.byte[2]);       // Commit the toggle
    Delay10us(25);                                                  // Delay the reading of the COMP and BC_LVL to allow time for settling
    FUSB300Read(regStatus0, 2, &Registers.Status.byte[4]);          // Read the current state of the BC_LVL and COMP
    ConnState = AttachWaitSource;                                   // Set the state machine variable to AttachWait.Src
	FUSB_LOG("set con_stat ====> %s, %s, %d\n", con_stat[ConnState], __func__, __LINE__);
    SinkCurrent = utccNone;                                         // Not used in Src
    if (Registers.Switches.MEAS_CC1)                                // If CC1 is what initially got us into the wait state...
    {
        CC1TermAct = DecodeCCTermination(caller_name);                         // Determine what is initially on CC1
        CC2TermAct = CCTypeNone;                                    // Assume that the initial value on CC2 is open
    }
    else
    {
        CC1TermAct = CCTypeNone;                                    // Assume that the initial value on CC1 is open
        CC2TermAct = DecodeCCTermination(caller_name);                         // Determine what is initially on CC2
    }
    CC1TermDeb = CCTypeRa;                                          // Initially set both the debounced values to Ra to force the 2nd level debouncing
    CC2TermDeb = CCTypeRa;                                          // Initially set both the debounced values to Ra to force the 2nd level debouncing
    StateTimer = USHRT_MAX;                                         // Disable the state timer, not used in this state
    DebounceTimer1 = tPDDebounceMin;                                // Only debounce the lines for tPDDebounce so that we can debounce a detach condition
    FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
    DebounceTimer2 = USHRT_MAX;                                     // Disable the 2nd level debouncing initially to force completion of a 1st level debouncing
    ToggleTimer = tDRP;                                             // Set the initial toggle time to tDRP to ensure the other end sees the Rp
    FUSB302_start_timer(&toggle_hrtimer,ToggleTimer);
    wake_up_statemachine(caller_name);
}

void SetStateAttachWaitAcc(void)
{
	char *caller_name = "AttachWaitAcc";
    VBUS_5V_EN = 0;                                                 // Disable the 5V output...
    VBUS_12V_EN = 0;                                                // Disable the 12V output
    Registers.Power.PWR = 0x07;                                     // Enable everything except internal oscillator

    Registers.Switches.word = 0x0044;                               // Enable CC1 pull-up and measure
    UpdateSourcePowerMode();
    Registers.Control.TOGGLE = 0;                                   // Disable the toggle
    FUSB300Write(regPower, 1, &Registers.Power.byte);               // Commit the power state
    FUSB300Write(regSwitches0, 2, &Registers.Switches.byte[0]);     // Commit the switch state
    FUSB300Write(regMeasure, 1, &Registers.Measure.byte);           // Commit the DAC threshold
    FUSB300Write(regControl2, 1, &Registers.Control.byte[2]);       // Commit the toggle
    Delay10us(25);                                                  // Delay the reading of the COMP and BC_LVL to allow time for settling
    FUSB300Read(regStatus0, 2, &Registers.Status.byte[4]);          // Read the current state of the BC_LVL and COMP
    ConnState = AttachWaitAccessory;                                // Set the state machine variable to AttachWait.Accessory
	FUSB_LOG("set con_stat ====> %s, %s, %d\n", con_stat[ConnState], __func__, __LINE__);
    SinkCurrent = utccNone;                                         // Not used in accessories
    CC1TermAct = DecodeCCTermination(caller_name);                             // Determine what is initially on CC1
    CC2TermAct = CCTypeNone;                                        // Assume that the initial value on CC2 is open
    CC1TermDeb = CCTypeNone;                                        // Initialize to open
    CC2TermDeb = CCTypeNone;                                        // Initialize to open
    StateTimer = USHRT_MAX;                                         // Disable the state timer, not used in this state
    DebounceTimer1 = tCCDebounceNom;                                // Once in this state, we are waiting for the lines to be stable for tCCDebounce before changing states
    FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
    DebounceTimer2 = USHRT_MAX;                                     // Disable the 2nd level debouncing initially to force completion of a 1st level debouncing
    ToggleTimer = tFUSB302Toggle;                                   // We're looking for the status of both lines of an accessory, no need to keep the line pull-ups on for tPDDebounce
    FUSB302_start_timer(&toggle_hrtimer,ToggleTimer);
    wake_up_statemachine(caller_name);
}

void SetStateAttachedSrc(void)
{
	char *caller_name = "SetStateAttachedSrc";
    VBUS_5V_EN = 1;                                                 // Enable the 5V output...
    VBUS_12V_EN = 0;                                                // Disable the 12V output
    SourceCurrent = utccDefault;                                    // Reset the current to the default advertisement
    UpdateSourcePowerMode();                                        // Update the source power mode
    if (blnCCPinIsCC1 == TRUE)                                      // If CC1 is detected as the CC pin...
        Registers.Switches.word = 0x0064;                           // Configure VCONN on CC2, pull-up on CC1, measure CC1
    else                                                            // Otherwise we are assuming CC2 is CC
        Registers.Switches.word = 0x0098;                           // Configure VCONN on CC1, pull-up on CC2, measure CC2
    Registers.Power.PWR = 0x07;                                     // Enable everything except internal oscillator
#ifdef PD_SUPPORT
    USBPDEnable(FALSE, TRUE);                                       // Enable the USB PD state machine if applicable (no need to write to FUSB300 again), set as DFP
#endif
    FUSB300Write(regPower, 1, &Registers.Power.byte);               // Commit the power state
    FUSB300Write(regSwitches0, 2, &Registers.Switches.byte[0]);     // Commit the switch state
    Delay10us(25);                                                  // Delay the reading of the COMP and BC_LVL to allow time for settling
    FUSB300Read(regStatus0, 2, &Registers.Status.byte[4]);          // Read the current state of the BC_LVL and COMP
    // Maintain the existing CC term values from the wait state
    ConnState = AttachedSource;                                     // Set the state machine variable to Attached.Src
	FUSB_LOG("set con_stat ====> %s, %s, %d\n", con_stat[ConnState], __func__, __LINE__);
    SinkCurrent = utccNone;                                         // Set the Sink current to none (not used in source)
    StateTimer = USHRT_MAX;                                         // Disable the state timer, not used in this state
    DebounceTimer1 = tPDDebounceMin;                                // Set the debounce timer to tPDDebounceMin for detecting a detach
    FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
    DebounceTimer2 = USHRT_MAX;                                     // Disable the 2nd level debouncing, not needed in this state
    ToggleTimer = USHRT_MAX;                                        // Disable the toggle timer, not used in this state
    wake_up_statemachine(caller_name);

	/* mimic the DFP turn on the vbus */
	dfp_vbus_switch(1);
}

void SetStateAttachedSink(void)
{
	char *caller_name = "SetStateAttachedSink";
    VBUS_5V_EN = 0;                                                 // Disable the 5V output...
    VBUS_12V_EN = 0;                                                // Disable the 12V output
    Registers.Power.PWR = 0x07;                                     // Enable everything except internal oscillator
    Registers.Control.HOST_CUR = 0x00;                              // Disable the host current
    Registers.Measure.MDAC = MDAC_2P05V;                            // Set up DAC threshold to 2.05V
//    Registers.Switches.word = 0x0007;                               // Enable the pull-downs on the CC pins, measure CC1 and disable the BMC transmitters
    Registers.Switches.word = 0x0003;                               // Enable the pull-downs on the CC pins
    if (blnCCPinIsCC1)
        Registers.Switches.MEAS_CC1 = 1;
    else
        Registers.Switches.MEAS_CC2 = 1;
#ifdef PD_SUPPORT
    USBPDEnable(FALSE, FALSE);                                      // Enable the USB PD state machine (no need to write FUSB300 again since we are doing it here)
#endif
    FUSB300Write(regPower, 1, &Registers.Power.byte);               // Commit the power state
    FUSB300Write(regControl0, 1, &Registers.Control.byte[0]);       // Commit the host current
    FUSB300Write(regMeasure, 1, &Registers.Measure.byte);           // Commit the DAC threshold
    FUSB300Write(regSwitches0, 2, &Registers.Switches.byte[0]);     // Commit the switch state
    Delay10us(25);                                                  // Delay the reading of the COMP and BC_LVL to allow time for settling
    FUSB300Read(regStatus0, 2, &Registers.Status.byte[4]);          // Read the current state of the BC_LVL and COMP
    ConnState = AttachedSink;                                       // Set the state machine variable to Attached.Sink
	FUSB_LOG("set con_stat ====> %s, %s, %d\n", con_stat[ConnState], __func__, __LINE__);
    SinkCurrent = utccDefault;                                      // Set the current advertisment variable to the default until we detect something different
    // Maintain the existing CC term values from the wait state
    StateTimer = USHRT_MAX;                                         // Disable the state timer, not used in this state
    DebounceTimer1 = tPDDebounceMin;                                // Set the debounce timer to tPDDebounceMin for detecting changes in advertised current
    FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
    DebounceTimer2 = USHRT_MAX;                                     // Disable the 2nd level debounce timer, not used in this state
    ToggleTimer = USHRT_MAX;                                        // Disable the toggle timer, not used in this state
    wake_up_statemachine(caller_name);
}

void RoleSwapToAttachedSink(void)
{
	char *caller_name = "RoleSwapToAttachedSink";
    VBUS_5V_EN = 0;                                                 // Disable the 5V output...
    VBUS_12V_EN = 0;                                                // Disable the 12V output
    Registers.Control.HOST_CUR = 0x00;                              // Disable the host current
    Registers.Measure.MDAC = MDAC_2P05V;                            // Set up DAC threshold to 2.05V
    if (blnCCPinIsCC1)                                              // If the CC pin is CC1...
    {
        Registers.Switches.PU_EN1 = 0;                              // Disable the pull-up on CC1
        Registers.Switches.PDWN1 = 1;                               // Enable the pull-down on CC1
        // No change for CC2, it may be used as VCONN
        CC1TermAct = CCTypeRa;                                      // Initialize the CC term as open
        CC1TermDeb = CCTypeRa;                                      // Initialize the CC term as open
    }
    else
    {
        Registers.Switches.PU_EN2 = 0;                              // Disable the pull-up on CC2
        Registers.Switches.PDWN2 = 1;                               // Enable the pull-down on CC2
        // No change for CC1, it may be used as VCONN
        CC2TermAct = CCTypeRa;                                      // Initialize the CC term as open
        CC2TermDeb = CCTypeRa;                                      // Initialize the CC term as open
    }
    FUSB300Write(regControl0, 1, &Registers.Control.byte[0]);       // Commit the host current
    FUSB300Write(regMeasure, 1, &Registers.Measure.byte);           // Commit the DAC threshold
    FUSB300Write(regSwitches0, 1, &Registers.Switches.byte[0]);     // Commit the switch state
    Delay10us(25);                                                  // Delay the reading of the COMP and BC_LVL to allow time for settling
    FUSB300Read(regStatus0, 2, &Registers.Status.byte[4]);          // Read the current state of the BC_LVL and COMP
    ConnState = AttachedSink;                                       // Set the state machine variable to Attached.Sink
	FUSB_LOG("set con_stat ====> %s, %s, %d\n", con_stat[ConnState], __func__, __LINE__);
    SinkCurrent = utccNone;                                         // Set the current advertisment variable to none until we determine what the current is
    StateTimer = USHRT_MAX;                                         // Disable the state timer, not used in this state
    DebounceTimer1 = tPDDebounceMin;                                // Set the debounce timer to tPDDebounceMin for detecting changes in advertised current
    FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
    DebounceTimer2 = USHRT_MAX;                                     // Disable the 2nd level debounce timer, not used in this state
    ToggleTimer = USHRT_MAX;                                        // Disable the toggle timer, not used in this state
    wake_up_statemachine(caller_name);
}

void RoleSwapToAttachedSource(void)
{
	char *caller_name = "RoleSwapToAttachedSource";
    VBUS_5V_EN = 1;                                                 // Enable the 5V output...
    VBUS_12V_EN = 0;                                                // Disable the 12V output
    UpdateSourcePowerMode();                                        // Update the pull-up currents and measure block
    if (blnCCPinIsCC1)                                              // If the CC pin is CC1...
    {
        Registers.Switches.PU_EN1 = 1;                              // Enable the pull-up on CC1
        Registers.Switches.PDWN1 = 0;                               // Disable the pull-down on CC1
        // No change for CC2, it may be used as VCONN
        CC1TermAct = CCTypeNone;                                    // Initialize the CC term as open
        CC1TermDeb = CCTypeNone;                                    // Initialize the CC term as open
    }
    else
    {
        Registers.Switches.PU_EN2 = 1;                              // Enable the pull-up on CC2
        Registers.Switches.PDWN2 = 0;                               // Disable the pull-down on CC2
        // No change for CC1, it may be used as VCONN
        CC2TermAct = CCTypeNone;                                    // Initialize the CC term as open
        CC2TermDeb = CCTypeNone;                                    // Initialize the CC term as open
    }
    FUSB300Write(regSwitches0, 1, &Registers.Switches.byte[0]);     // Commit the switch state
    Delay10us(25);                                                  // Delay the reading of the COMP and BC_LVL to allow time for settling
    FUSB300Read(regStatus0, 2, &Registers.Status.byte[4]);          // Read the current state of the BC_LVL and COMP
    ConnState = AttachedSource;                                     // Set the state machine variable to Attached.Src
	FUSB_LOG("set con_stat ====> %s, %s, %d\n", con_stat[ConnState], __func__, __LINE__);
    SinkCurrent = utccNone;                                         // Set the Sink current to none (not used in Src)
    StateTimer = USHRT_MAX;                                         // Disable the state timer, not used in this state
    DebounceTimer1 = tPDDebounceMin;                                // Set the debounce timer to tPDDebounceMin for detecting a detach
    FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
    DebounceTimer2 = USHRT_MAX;                                     // Disable the 2nd level debouncing, not needed in this state
    ToggleTimer = USHRT_MAX;                                        // Disable the toggle timer, not used in this state
    wake_up_statemachine(caller_name);
}

void SetStateTryWaitSnk(void)
{
	char *caller_name = "TryWaitSnk";
    VBUS_5V_EN = 0;                                                 // Disable the 5V output...
    VBUS_12V_EN = 0;                                                // Disable the 12V output
    Registers.Switches.word = 0x0007;                               // Enable the pull-downs on the CC pins and measure on CC1
    Registers.Power.PWR = 0x07;                                     // Enable everything except internal oscillator
    Registers.Measure.MDAC = MDAC_2P05V;                            // Set up DAC threshold to 2.05V
    FUSB300Write(regSwitches0, 2, &Registers.Switches.byte[0]);     // Commit the switch state
    FUSB300Write(regPower, 1, &Registers.Power.byte);               // Commit the power state
    FUSB300Write(regMeasure, 1, &Registers.Measure.byte);           // Commit the DAC threshold
    Delay10us(25);                                                  // Delay the reading of the COMP and BC_LVL to allow time for settling
    FUSB300Read(regStatus0, 2, &Registers.Status.byte[4]);          // Read the current state of the BC_LVL and COMP
    ConnState = TryWaitSink;                                        // Set the state machine variable to TryWait.Snk
	FUSB_LOG("set con_stat ====> %s, %s, %d\n", con_stat[ConnState], __func__, __LINE__);
    SinkCurrent = utccNone;                                         // Set the current advertisment variable to none until we determine what the current is
    if (Registers.Switches.MEAS_CC1)
    {
        CC1TermAct = DecodeCCTermination(caller_name);                         // Determine what is initially on CC1
        CC2TermAct = CCTypeNone;                                    // Assume that the initial value on CC2 is open
   }
    else
    {
        CC1TermAct = CCTypeNone ;                                   // Assume that the initial value on CC1 is open
        CC2TermAct = DecodeCCTermination(caller_name);                         // Determine what is initially on CC2
    }
    CC1TermDeb = CCTypeNone;                                        // Initially set the debounced value to none so we don't immediately detach
    CC2TermDeb = CCTypeNone;                                        // Initially set the debounced value to none so we don't immediately detach
    StateTimer = tDRPTryWait;                                       // Set the state timer to tDRPTryWait to timeout if Rp isn't detected
    FUSB302_start_timer(&state_hrtimer,StateTimer);
    DebounceTimer1 = tPDDebounceMin;                                // The 1st level debouncing is based upon tPDDebounce
    FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
    DebounceTimer2 = USHRT_MAX;                                     // Disable the 2nd level debouncing initially until we validate the 1st level
    ToggleTimer = tFUSB302Toggle;                                   // Toggle the measure quickly (tFUSB302Toggle) to see if we detect an Rp on either
    FUSB302_start_timer(&toggle_hrtimer,ToggleTimer);
    wake_up_statemachine(caller_name);
}

void SetStateTrySrc(void)
{
	char *caller_name = "TrySrc";
    VBUS_5V_EN = 0;                                                 // Disable the 5V output...
    VBUS_12V_EN = 0;                                                // Disable the 12V output
    SourceCurrent = utccDefault;                                    // Reset the current to the default advertisement

    Registers.Power.PWR = 0x07;                                     // Enable everything except internal oscillator
    Registers.Switches.word = 0x0000;                               // Disable everything (toggle overrides anyway)
    if (blnCCPinIsCC1)                                              // If we detected CC1 as an Rd
    {
        Registers.Switches.PU_EN1 = 1;                              // Enable the pull-up on CC1
        Registers.Switches.MEAS_CC1 = 1;                            // Measure on CC1
    }
    else
    {
        Registers.Switches.PU_EN2 = 1;                              // Enable the pull-up on CC1\2
        Registers.Switches.MEAS_CC2 = 1;                            // Measure on CC2
    }
    UpdateSourcePowerMode();
    FUSB300Write(regPower, 1, &Registers.Power.byte);               // Commit the power state
    FUSB300Write(regSwitches0, 2, &Registers.Switches.byte[0]);     // Commit the switch state
    Delay10us(25);                                                  // Delay the reading of the COMP and BC_LVL to allow time for settling
    FUSB300Read(regStatus0, 2, &Registers.Status.byte[4]);          // Read the current state of the BC_LVL and COMP
    ConnState = TrySource;                                          // Set the state machine variable to Try.Src
	FUSB_LOG("set con_stat ====> %s, %s, %d\n", con_stat[ConnState], __func__, __LINE__);
    SinkCurrent = utccNone;                                         // Not used in Try.Src
    blnCCPinIsCC1 = FALSE;                                          // Clear the CC1 is CC flag (don't know)
    blnCCPinIsCC2 = FALSE;                                          // Clear the CC2 is CC flag (don't know)
    if (Registers.Switches.MEAS_CC1)
    {
        CC1TermAct = DecodeCCTermination(caller_name);                         // Determine what is initially on CC1
        CC2TermAct = CCTypeNone;                                    // Assume that the initial value on CC2 is open
   }
    else
    {
        CC1TermAct = CCTypeNone;                                    // Assume that the initial value on CC1 is open
        CC2TermAct = DecodeCCTermination(caller_name);                         // Determine what is initially on CC2
    }
    CC1TermDeb = CCTypeNone;                                        // Initially set the debounced value as open until we actually debounce the signal
    CC2TermDeb = CCTypeNone;                                        // Initially set both the active and debounce the same
    StateTimer = tDRPTry;                                           // Set the state timer to tDRPTry to timeout if Rd isn't detected
    FUSB302_start_timer(&state_hrtimer,StateTimer);
    DebounceTimer1 = tPDDebounceMin;                                // Debouncing is based soley off of tPDDebounce
    FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
    DebounceTimer2 = USHRT_MAX;                                     // Disable the 2nd level since it's not needed
    ToggleTimer = tPDDebounceMax;                                   // Keep the pull-ups on for the max tPDDebounce to ensure that the other side acknowledges the pull-up
    FUSB302_start_timer(&toggle_hrtimer,ToggleTimer);
    wake_up_statemachine(caller_name);
}

void SetStateDebugAccessory(void)
{
	char *caller_name = "SetStateDebugAccessory";
    VBUS_5V_EN = 0;                                                 // Disable the 5V output...
    VBUS_12V_EN = 0;                                                // Disable the 12V output
    Registers.Power.PWR = 0x07;                                     // Enable everything except internal oscillator
    Registers.Switches.word = 0x0044;                               // Enable CC1 pull-up and measure
    UpdateSourcePowerMode();
    FUSB300Write(regPower, 1, &Registers.Power.byte);               // Commit the power state
    FUSB300Write(regSwitches0, 2, &Registers.Switches.byte[0]);     // Commit the switch state
    FUSB300Write(regMeasure, 1, &Registers.Measure.byte);           // Commit the DAC threshold
    Delay10us(25);                                                  // Delay the reading of the COMP and BC_LVL to allow time for settling
    FUSB300Read(regStatus0, 2, &Registers.Status.byte[4]);          // Read the current state of the BC_LVL and COMP
    ConnState = DebugAccessory;                                     // Set the state machine variable to Debug.Accessory
	FUSB_LOG("set con_stat ====> %s, %s, %d\n", con_stat[ConnState], __func__, __LINE__);
    SinkCurrent = utccNone;                                         // Not used in accessories
    // Maintain the existing CC term values from the wait state
    StateTimer = USHRT_MAX;                                         // Disable the state timer, not used in this state
    DebounceTimer1 = tCCDebounceNom;                                // Once in this state, we are waiting for the lines to be stable for tCCDebounce before changing states
    FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
    DebounceTimer2 = USHRT_MAX;                                     // Disable the 2nd level debouncing initially to force completion of a 1st level debouncing
    ToggleTimer = USHRT_MAX;                                        // Once we are in the debug.accessory state, we are going to stop toggling and only monitor CC1
    wake_up_statemachine(caller_name);
}

void SetStateAudioAccessory(void)
{
	char *caller_name = "SetStateAudioAccessory";
    VBUS_5V_EN = 0;                                                 // Disable the 5V output...
    VBUS_12V_EN = 0;                                                // Disable the 12V output
    Registers.Power.PWR = 0x07;                                     // Enable everything except internal oscillator
    Registers.Switches.word = 0x0044;                               // Enable CC1 pull-up and measure
    UpdateSourcePowerMode();
    FUSB300Write(regPower, 1, &Registers.Power.byte);               // Commit the power state
    FUSB300Write(regSwitches0, 2, &Registers.Switches.byte[0]);     // Commit the switch state
    FUSB300Write(regMeasure, 1, &Registers.Measure.byte);           // Commit the DAC threshold
    Delay10us(25);                                                  // Delay the reading of the COMP and BC_LVL to allow time for settling
    FUSB300Read(regStatus0, 2, &Registers.Status.byte[4]);          // Read the current state of the BC_LVL and COMP
    ConnState = AudioAccessory;                                     // Set the state machine variable to Audio.Accessory
	printk("set con_stat ====> %s, %s, %d\n", con_stat[ConnState], __func__, __LINE__);
    SinkCurrent = utccNone;                                         // Not used in accessories
    // Maintain the existing CC term values from the wait state
    StateTimer = USHRT_MAX;                                         // Disable the state timer, not used in this state
    DebounceTimer1 = tCCDebounceNom;                                // Once in this state, we are waiting for the lines to be stable for tCCDebounce before changing states
    FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
    DebounceTimer2 = USHRT_MAX;                                     // Disable the 2nd level debouncing initially to force completion of a 1st level debouncing
    ToggleTimer = USHRT_MAX;                                        // Once we are in the audio.accessory state, we are going to stop toggling and only monitor CC1
    wake_up_statemachine(caller_name);
}

void SetStatePoweredAccessory(void)
{
	char *caller_name = "SetStatePoweredAccessory";
    VBUS_5V_EN = 0;                                                 // Disable the 5V output...
    VBUS_12V_EN = 0;                                                // Disable the 12V output
    SourceCurrent = utcc1p5A;                                       // Have the option of 1.5A/3.0A for powered accessories, choosing 1.5A advert
    UpdateSourcePowerMode();                                        // Update the Source Power mode
    if (blnCCPinIsCC1 == TRUE)                                      // If CC1 is detected as the CC pin...
        Registers.Switches.word = 0x0064;                           // Configure VCONN on CC2, pull-up on CC1, measure CC1
    else                                                            // Otherwise we are assuming CC2 is CC
        Registers.Switches.word = 0x0098;                           // Configure VCONN on CC1, pull-up on CC2, measure CC2
    FUSB300Write(regSwitches0, 2, &Registers.Switches.byte[0]);     // Commit the switch state
    Delay10us(25);                                                  // Delay the reading of the COMP and BC_LVL to allow time for settling
    FUSB300Read(regStatus0, 2, &Registers.Status.byte[4]);          // Read the current state of the BC_LVL and COMP
    // Maintain the existing CC term values from the wait state
    // TODO: The line below will be uncommented once we have full support for VDM's and can enter an alternate mode as needed for Powered.Accessories
    // USBPDEnable(TRUE, TRUE);
    ConnState = PoweredAccessory;                                   // Set the state machine variable to powered.accessory
	FUSB_LOG("set con_stat ====> %s, %s, %d\n", con_stat[ConnState], __func__, __LINE__);
    SinkCurrent = utccNone;                                         // Set the Sink current to none (not used in source)
    StateTimer = tAMETimeout;                                       // Set the state timer to tAMETimeout (need to enter alternate mode by this time)
    FUSB302_start_timer(&state_hrtimer,StateTimer);
    DebounceTimer1 = tPDDebounceMin;                                // Set the debounce timer to the minimum tPDDebounce to check for detaches
    FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
    DebounceTimer2 = USHRT_MAX;                                     // Disable the 2nd level debounce timer, not used in this state
    ToggleTimer = USHRT_MAX;                                        // Disable the toggle timer, only looking at the actual CC line
    wake_up_statemachine(caller_name);
}

void SetStateUnsupportedAccessory(void)
{
	char *caller_name = "SetStateUnsupportedAccessory";
    VBUS_5V_EN = 0;                                                 // Disable the 5V output...
    VBUS_12V_EN = 0;                                                // Disable the 12V output
    SourceCurrent = utccDefault;                                    // Reset the current to the default advertisement for this state
    UpdateSourcePowerMode();                                        // Update the Source Power mode
    Registers.Switches.VCONN_CC1 = 0;                               // Make sure VCONN is turned off
    Registers.Switches.VCONN_CC2 = 0;                               // Make sure VCONN is turned off
    FUSB300Write(regSwitches0, 2, &Registers.Switches.byte[0]);     // Commit the switch state
    Delay10us(25);                                                  // Delay the reading of the COMP and BC_LVL to allow time for settling
    FUSB300Read(regStatus0, 2, &Registers.Status.byte[4]);          // Read the current state of the BC_LVL and COMP
    ConnState = UnsupportedAccessory;                               // Set the state machine variable to unsupported.accessory
	FUSB_LOG("set con_stat ====> %s, %s, %d\n", con_stat[ConnState], __func__, __LINE__);
    SinkCurrent = utccNone;                                         // Set the Sink current to none (not used in source)
    StateTimer = USHRT_MAX;                                         // Disable the state timer, not used in this state
    DebounceTimer1 = tPDDebounceMin;                                // Set the debounce timer to the minimum tPDDebounce to check for detaches
    FUSB302_start_timer(&debounce_hrtimer1,DebounceTimer1);
    DebounceTimer2 = USHRT_MAX;                                     // Disable the 2nd level debounce timer, not used in this state
    ToggleTimer = USHRT_MAX;                                        // Disable the toggle timer, only looking at the actual CC line
    wake_up_statemachine(caller_name);
}

void UpdateSourcePowerMode(void)
{
    switch(SourceCurrent)
    {
        case utccDefault:
            Registers.Measure.MDAC = MDAC_1P6V;                     // Set up DAC threshold to 1.6V (default USB current advertisement)
            Registers.Control.HOST_CUR = 0x01;                      // Set the host current to reflect the default USB power
            break;
        case utcc1p5A:
            Registers.Measure.MDAC = MDAC_1P6V;                     // Set up DAC threshold to 1.6V
            Registers.Control.HOST_CUR = 0x02;                      // Set the host current to reflect 1.5A
            break;
        case utcc3p0A:
            Registers.Measure.MDAC = MDAC_2P6V;                     // Set up DAC threshold to 2.6V
            Registers.Control.HOST_CUR = 0x03;                      // Set the host current to reflect 3.0A
            break;
        default:                                                    // This assumes that there is no current being advertised
            Registers.Measure.MDAC = MDAC_1P6V;                     // Set up DAC threshold to 1.6V (default USB current advertisement)
            Registers.Control.HOST_CUR = 0x00;                      // Set the host current to disabled
            break;
    }
    FUSB300Write(regMeasure, 1, &Registers.Measure.byte);           // Commit the DAC threshold
    FUSB300Write(regControl0, 1, &Registers.Control.byte[0]);       // Commit the host current
}

/////////////////////////////////////////////////////////////////////////////
//                        Type C Support Routines
/////////////////////////////////////////////////////////////////////////////

void ToggleMeasureCC1(void)
{
    Registers.Switches.PU_EN1 = Registers.Switches.PU_EN2;                  // If the pull-up was enabled on CC2, enable it for CC1
    Registers.Switches.PU_EN2 = 0;                                          // Disable the pull-up on CC2 regardless, since we aren't measuring CC2 (prevent short)
    Registers.Switches.MEAS_CC1 = 1;                                        // Set CC1 to measure
    Registers.Switches.MEAS_CC2 = 0;                                        // Clear CC2 from measuring
    FUSB300Write(regSwitches0, 1, &Registers.Switches.byte[0]);             // Set the switch to measure
    Delay10us(25);                                                          // Delay the reading of the COMP and BC_LVL to allow time for settling
    FUSB300Read(regStatus0, 2, &Registers.Status.byte[4]);                  // Read back the status to get the current COMP and BC_LVL
}

void ToggleMeasureCC2(void)
{
    Registers.Switches.PU_EN2 = Registers.Switches.PU_EN1;                  // If the pull-up was enabled on CC1, enable it for CC2
    Registers.Switches.PU_EN1 = 0;                                          // Disable the pull-up on CC1 regardless, since we aren't measuring CC1 (prevent short)
    Registers.Switches.MEAS_CC1 = 0;                                        // Clear CC1 from measuring
    Registers.Switches.MEAS_CC2 = 1;                                        // Set CC2 to measure
    FUSB300Write(regSwitches0, 1, &Registers.Switches.byte[0]);             // Set the switch to measure
    Delay10us(25);                                                          // Delay the reading of the COMP and BC_LVL to allow time for settling
    FUSB300Read(regStatus0, 2, &Registers.Status.byte[4]);                  // Read back the status to get the current COMP and BC_LVL
 }

/* Decode self port */
CCTermType DecodeCCTermination(char *caller_name)
{
    CCTermType Termination = CCTypeNone;            // By default set it to nothing
	pr_debug("%s decode CC Termination, Voltage on Sink CC pins\n", caller_name);
    if (Registers.Status.COMP == 0)                 // If COMP is high, the BC_LVL's don't matter
    {
        switch (Registers.Status.BC_LVL)            // Determine which level
        {
            case 0b00:                              // If BC_LVL is lowest... it's an vRa
                Termination = CCTypeRa;
				FUSB_LOG("Termination is Ra, vRa\n");
                break;
            case 0b01:                              // If BC_LVL is 1, it's default
                Termination = CCTypeRdUSB;
				FUSB_LOG("Termination is Default, vRd-Connect and vRd-USB\n");
                break;
            case 0b10:                              // If BC_LVL is 2, it's vRd1p5
                Termination = CCTypeRd1p5;
				FUSB_LOG("Termination is vRd1p5, vRd-Connect and vRd-1.5\n");
                break;
            default:                                // Otherwise it's vRd3p0
                Termination = CCTypeRd3p0;
				FUSB_LOG("Termination is vRd3p0, vRd-Connect and vRd-3.0\n");
                break;
        }
    }
	else
	{
		FUSB_LOG("@@@@@@@@Selected CC line hasn't tripped a threshold into MDAC!!!!\n");
	}
    return Termination;                             // Return the termination type
}

void UpdateSinkCurrent(CCTermType Termination)
{
    switch (Termination)
    {
        case CCTypeRdUSB:                       // If we detect the default...
        case CCTypeRa:                          // Or detect an accessory (vRa)
            SinkCurrent = utccDefault;
			FUSB_LOG("sink current is default!!\n");
            break;
        case CCTypeRd1p5:                       // If we detect 1.5A
            SinkCurrent = utcc1p5A;
			FUSB_LOG("sink current is 1.5A\n");
            break;
        case CCTypeRd3p0:                       // If we detect 3.0A
            SinkCurrent = utcc3p0A;
			FUSB_LOG("sink current is 3.0A\n");
            break;
        default:
            SinkCurrent = utccNone;
			FUSB_LOG("sink current is None\n");
            break;
    }
}

void ConfigurePortType(unsigned char Control)
{
    unsigned char value;
    DisableFUSB300StateMachine();
    value = Control & 0x03;
    switch (value)
    {
        case 1:
            PortType = USBTypeC_Source;
            break;
        case 2:
            PortType = USBTypeC_DRP;
            break;
        default:
            PortType = USBTypeC_Sink;
            break;
    }
    if (Control & 0x04)
        blnAccSupport = TRUE;
    else
        blnAccSupport = FALSE;
    if (Control & 0x08)
        blnSrcPreferred = TRUE;
    else
        blnSrcPreferred = FALSE;
    value = (Control & 0x30) >> 4;
    switch (value)
    {
        case 1:
            SourceCurrent = utccDefault;
            break;
        case 2:
            SourceCurrent = utcc1p5A;
            break;
        case 3:
            SourceCurrent = utcc3p0A;
            break;
        default:
            SourceCurrent = utccNone;
            break;
    }
    if (Control & 0x80)
        EnableFUSB300StateMachine();
}

void UpdateCurrentAdvert(unsigned char Current)
{
    switch (Current)
    {
        case 1:
            SourceCurrent = utccDefault;
            break;
        case 2:
            SourceCurrent = utcc1p5A;
            break;
        case 3:
            SourceCurrent = utcc3p0A;
            break;
        default:
            SourceCurrent = utccNone;
            break;
    }
    if (ConnState == AttachedSource)
        UpdateSourcePowerMode();
}

void GetFUSB300TypeCStatus(unsigned char abytData[])
{
    int intIndex = 0;
    abytData[intIndex++] = GetTypeCSMControl();     // Grab a snapshot of the top level control
    abytData[intIndex++] = ConnState & 0xFF;        // Get the current state
    abytData[intIndex++] = GetCCTermination();      // Get the current CC termination
    abytData[intIndex++] = SinkCurrent;             // Set the sink current capability detected
}

unsigned char GetTypeCSMControl(void)
{
    unsigned char status = 0;
    status |= (PortType & 0x03);            // Set the type of port that we are configured as
    switch(PortType)                        // Set the port type that we are configured as
    {
        case USBTypeC_Source:
            status |= 0x01;                 // Set Source type
            break;
        case USBTypeC_DRP:
            status |= 0x02;                 // Set DRP type
            break;
        default:                            // If we are not DRP or Source, we are Sink which is a value of zero as initialized
            break;
    }
    if (blnAccSupport)                      // Set the flag if we support accessories
        status |= 0x04;
    if (blnSrcPreferred)                    // Set the flag if we prefer Source mode (as a DRP)
        status |= 0x08;
    status |= (SourceCurrent << 4);
    if (blnSMEnabled)                       // Set the flag if the state machine is enabled
        status |= 0x80;
    return status;
}

unsigned char GetCCTermination(void)
{
    unsigned char status = 0;
    status |= (CC1TermDeb & 0x07);          // Set the current CC1 termination
//    if (blnCC1Debounced)                    // Set the flag if the CC1 pin has been debounced
//        status |= 0x08;
    status |= ((CC2TermDeb & 0x07) << 4);   // Set the current CC2 termination
//    if (blnCC2Debounced)                    // Set the flag if the CC2 pin has been debounced
//        status |= 0x80;
    return status;
}

static char *int_name[] = {
	"I_BC_LVL",
	"I_COLLISION",
	"I_WAKE",
	"I_ALERT",
	"I_CRC_CHK",
	"I_COMP_CHNG",
	"I_ACTIVITY",
	"I_VBUSOK"
};

//static char *switches0[] = {
//	"PDWN1",
//	"PDWN2",
//	"MEAS_CC1",
//	"MEAS_CC2",
//	"VCONN_CC1",
//	"VCONN_CC2",
//	"PU_EN1",
//	"PU_EN2"
//};

static irqreturn_t cc_eint_interrupt_handler(int irq, void *handle)
{
	char *caller_name = "INT_handler";

	/* Just for debug register */
    schedule_work(&fusb_i2c_data->eint_work);

	wake_up_statemachine(caller_name);
	return IRQ_HANDLED;
}

static void fusb_eint_work(struct work_struct *work)
{
//	char *caller_name = "fusb_eint_work";
	unsigned char byte;
	int i;
 //   int reg_addr[] = {regDeviceID, regSwitches0, regSwitches1, regMeasure, regSlice,
  //      regControl0, regControl1, regControl2, regControl3, regMask, regPower, regReset,
  //      regOCPreg, regMaska, regMaskb, regControl4, regStatus0a, regStatus1a, regInterrupta,
    //    regInterruptb, regStatus0, regStatus1, regInterrupt };
	unsigned char status0_bit0;
	unsigned char status0_bit1;

	FUSB300Read(regStatus0, 1, &byte);
	pr_debug("regStatus0 = 0x%02x\n", byte);
	status0_bit0 = byte & 0x1;
	status0_bit1 = byte & 0x2;
	if (byte & 0x80)
		FUSB_LOG("VBUSOK,%s, %d\n", __func__, __LINE__);
	if (byte & 0x40)
		FUSB_LOG("ACTIVITY,%s, %d\n", __func__, __LINE__);
	if (byte & 0x20)
		FUSB_LOG("CC is higher then refernce level,%s, %d\n", __func__, __LINE__);
	if (byte & 0x4)
		FUSB_LOG("Device is attempting to attach.%s, %d\n", __func__, __LINE__);
	if (status0_bit0 && status0_bit1)
		FUSB_LOG(">1.23V=======%s, %d\n", __func__, __LINE__);
	else if (status0_bit0 && !status0_bit1)
		FUSB_LOG("[200-660]mV%s, %d\n", __func__, __LINE__);
	else if (status0_bit1 && !status0_bit0)
		FUSB_LOG("[660mV-1.23V]%s, %d\n", __func__, __LINE__);
	else if (!status0_bit1 && !status0_bit0)
		FUSB_LOG("<200mV========%s, %d\n", __func__, __LINE__);
	else
	{
		/* Do nothing. */
	}

	/* find out which interupt bit toogle */
	FUSB300Read(regInterrupt, 1, &byte);
	pr_debug("regInterrupt = 0x%02x\n", byte);
	for (i = 0; i < 8; i++)
		if (byte & (1 << i))
			FUSB_LOG("%s toogle \n", int_name[i]);

	/* togss  */
	FUSB300Read(regStatus1a, 1, &byte);
	pr_debug("regStatus1a = 0x%02x\n", byte);
	byte &= 0x38;
	byte >>= 3;
	if (0 == byte)
		FUSB_LOG("Toggle logic running (processor has previously writen TOGGLE=1)\n");
	else if (1 == byte)
		FUSB_LOG("settled to SRC(DFP)on CC1\n");
	else if (2 == byte)
		FUSB_LOG("settled to SRC(DFP)on CC2\n");
	else if (3 == byte)
		FUSB_LOG("settled to UFP\n");
	else if (5 == byte)
		FUSB_LOG("settled to (SNK)UFP on CC1\n");
	else if (6 == byte)
		FUSB_LOG("settled to (SNK)UFP on CC2\n");
	else if (7 == byte)
		printk("settled to detected AudioAccessory\n");
	else
		FUSB_LOG("ERROR!!! Not Defind\n");

	//wake_up_statemachine(caller_name);
}

int fusb302_state_kthread(void *x)
{
    struct sched_param param = { .sched_priority = 98 };
    FUSB_LOG("***********enter fusb302 state thread!!1 ********************\n");
    sched_setscheduler(current, SCHED_RR, &param);
    while (1) {
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(fusb_thread_wq, state_changed==TRUE);
	state_changed = FALSE;
        set_current_state(TASK_RUNNING);

        StateMachineFUSB300();
    }
    return 0;
}

static void fusb302_device_check(void)
{
    FUSB300Read(regDeviceID, 2, &Registers.DeviceID.byte);
    FUSB_LOG("device id:%2x\n", Registers.DeviceID.byte);
}

/* echo  addr val > fusb302_dump */
static ssize_t fusb302_reg_dump_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len)
{
	/* Default.*/
	//u8 reg_addr = 0x1;
	//u8 reg_val = 0;
	int reg_addr = 0x1;
	int reg_val = 0;
	int ret;
	sscanf(buf, "%d %d", &reg_addr, &reg_val);
	pr_err("write reg[%d] = %d\n", reg_addr, reg_val);

	ret = i2c_smbus_write_byte_data(fusb_i2c_data->client, reg_addr, reg_val);
	if (ret < 0)
		FUSB_LOG(KERN_ERR "%s: failed to write register %x err= %d\n", __func__, reg_addr, ret);

	return len;
}

static ssize_t fusb302_reg_dump(struct device *dev,struct device_attribute *attr, char *buf)
{
    int reg_addr[] = {regDeviceID, regSwitches0, regSwitches1, regMeasure, regSlice,
        regControl0, regControl1, regControl2, regControl3, regMask, regPower, regReset,
        regOCPreg, regMaska, regMaskb, regControl4, regStatus0a, regStatus1a, regInterrupta,
        regInterruptb, regStatus0, regStatus1, regInterrupt };

    int i, len = 0;
    unsigned char byte = 0;
    for (i=0; i< sizeof(reg_addr)/sizeof(reg_addr[0]); i++)
    {
        FUSB300Read(reg_addr[i], 1, &byte);
        len += sprintf(buf+len, "reg[0x%02x] = 0x%02x\n", reg_addr[i], byte);
    }
    return len;
}

static DEVICE_ATTR(fusb302_dump, 0666, fusb302_reg_dump, fusb302_reg_dump_store);

static ssize_t fusb302_state(struct device *dev, struct device_attribute *attr, char *buf)
{
     char data[5];
     GetFUSB300TypeCStatus(data);
     return sprintf(buf, "SMC=%2x, connState=%2d, cc=%2x, current=%d\n", data[0], data[1], data[2], data[3]);
}

 static DEVICE_ATTR(state, 0444, fusb302_state, NULL);
static int fusb302_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
    struct fusb302_i2c_data *fusb;
    int ret_device_file = 0;
	int ret;
	int rc;
	char byte;
	char *reset_val = "0x01";


    fusb = kzalloc(sizeof(struct fusb302_i2c_data), GFP_KERNEL);
    if (!fusb) {
        dev_err(&i2c->dev, "private data alloc fail\n");
		return -1;
    }

    InitializeFUSB300Variables();
#ifdef PD_SUPPORT
    InitializeUSBPDVariables();
#endif

    fusb_i2c_data = fusb;
    i2c_set_clientdata(i2c, fusb);
    fusb->client = i2c;

    ret_device_file = device_create_file(&(i2c->dev), &dev_attr_fusb302_dump);
    ret_device_file = device_create_file(&(i2c->dev), &dev_attr_state);

    fusb302_device_check();
    //Initialize FUSB302
    FUSB300Write(regReset, 1, reset_val);
    InitializeFUSB300();

    fusb->thread = kthread_run(fusb302_state_kthread, NULL, "fusb302_state_kthread");

    #ifdef USE_EARLY_SUSPEND
        fusb->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
        fusb->early_drv.suspend  = fusb302_early_suspend,
        fusb->early_drv.resume   = fusb302_late_resume,
        register_early_suspend(&fusb->early_drv);
    #endif

	/* mask all the int */
	ret = i2c_smbus_write_byte_data(fusb_i2c_data->client, regMask, 0xFF);
	if (ret < 0)
		FUSB_LOG(KERN_ERR "%s: failed to write register %x err= %d\n", __func__, regMask, ret);

	FUSB300Read(regMask, 1, &byte);
	FUSB_LOG("reg[0x%02x] = 0x%02x\n", regMask, byte);

    INIT_WORK(&fusb_i2c_data->eint_work, fusb_eint_work);

	rc = of_get_named_gpio(i2c->dev.of_node, "usb_det", 0);
	if (rc < 0)
		FUSB_LOG("ERROR!! pd detect int\n");
	FUSB_LOG("pd_det gpio num is %d\n", rc - 890);/* 878 is base*/

	ret = gpio_request(rc, "usb_det");
	if (ret < 0)
		FUSB_LOG("ERROR!!gpio request ERROR\n");

	ret = gpio_direction_input(rc);
	if (ret < 0)
		FUSB_LOG("ERROR!!gpio direction input ERROR\n");

	FUSB_LOG("pd line is %s\n", 1 == gpio_get_value(rc) ? "hi" : "lo");
	FUSB_LOG("int name = %s\n", dev_name(&i2c->dev));

	ret = devm_request_irq(&i2c->dev, gpio_to_irq(rc), &cc_eint_interrupt_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_PROBE_SHARED, dev_name(&i2c->dev), NULL);
	if (ret)
		FUSB_LOG("ERROR!!Failed to request irq\n");

	/* unmask all the int */
	ret = i2c_smbus_write_byte_data(fusb_i2c_data->client, regMask, 0x00);
	if (ret < 0)
		FUSB_LOG(KERN_ERR "%s: failed to write register %x err= %d\n", __func__, regMask, ret);

	FUSB300Read(regMask, 1, &byte);
	FUSB_LOG("reg[0x%02x] = 0x%02x\n", regMask, byte);

	FUSB_LOG("probe successfully!\n");
    return 0;
}

static int fusb302_remove(struct i2c_client *i2c)
{
//        i2c_unregister_device(i2c);
        kfree(i2c_get_clientdata(i2c));
        return 0;
}

#ifndef USE_EARLY_SUSPEND
static int fusb302_suspend(struct i2c_client *client, pm_message_t msg)
{
        //wait to do something
        return 0;
}
static int fusb302_resume(struct i2c_client *client)
{
        //wait to do something
        return 0;
}
#else
static void fusb302_early_suspend(struct early_suspend *h)
{
        //wait to do something
}
static void fusb302_late_resume(struct early_suspend *h)
{
        //wait to do something
    //mt_eint_unmask(CUST_EINT_CC_DECODER_NUM);
}
#endif

static const struct i2c_device_id fusb302_id[] = {
        { FUSB302_I2C_NAME, 0 },
        { }
};

static struct i2c_board_info __initdata fusb302_i2c_boardinfo[] = {
        {
                I2C_BOARD_INFO(FUSB302_I2C_NAME, (0x22)),
        },
};

static struct of_device_id fusb302_match_table[] = {
	{.compatible = "fusb302",},
	{},
};
static struct i2c_driver fusb302_i2c_driver = {
        .driver = {
                .name = FUSB302_I2C_NAME,
                .owner = THIS_MODULE,
				.of_match_table = fusb302_match_table,
        },
        .probe          = fusb302_probe,
        .remove         = fusb302_remove,
#if !defined(USE_EARLY_SUSPEND)
        .suspend        = fusb302_suspend,
        .resume         = fusb302_resume,
#endif
        .id_table       = fusb302_id,
};

static int __init fusb302_i2c_init(void)
{
    i2c_register_board_info(FUSB302_I2C_NUM, fusb302_i2c_boardinfo,
        ARRAY_SIZE(fusb302_i2c_boardinfo));
    return i2c_add_driver(&fusb302_i2c_driver);
}

static void __exit fusb302_i2c_exit(void)
{
    i2c_del_driver(&fusb302_i2c_driver);
}

module_init(fusb302_i2c_init);
module_exit(fusb302_i2c_exit);
