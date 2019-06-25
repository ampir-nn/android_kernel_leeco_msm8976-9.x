 /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * Software License Agreement:
 *
 * The software supplied herewith by Fairchild Semiconductor (the Company)
 * is supplied to you, the Company's customer, for exclusive use with its
 * USB Type C / USB PD products.  The software is owned by the Company and/or
 * its supplier, and is protected under applicable copyright laws.
 * All rights are reserved. Any use in violation of the foregoing restrictions
 * may subject the user to criminal sanctions under applicable laws, as well
 * as to civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN AS IS CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *****************************************************************************/
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
//#include <mach/mt_pm_ldo.h>
#include <linux/interrupt.h>
#include <linux/time.h>
//#include <cust_eint.h>
//#include <mach/eint.h>
//#include <cust_eint.h>
#include <linux/kthread.h>
//#include <mach/mt_gpio.h>

#include "callback_defs.h"
#include "vdm_manager.h"
#include "vdm_types.h"
#include "vdm_bitfield_translators.h"
#include "usbpd.h"
#include "fusb302.h"

//#define PD_DEBUG
#ifdef PD_DEBUG
#define PD_LOG(fmt, args...)  printk(KERN_ERR"fusb302 PD --------> " fmt, ##args)
#else
#define PD_LOG(fmt, args...)
#endif

#ifdef PD_DEBUG
static const char *PD_OBJ_TYPE_NAME[] = {
        "pdoTypeFixed",
        "pdoTypeBattery",
        "pdoTypeVariable",
        "pdoTypeReserved"
};

/* Data Message Type */
static const char *DM_message_type[] = {
	"Reserved",
	"SourceCapabilities",
	"Request",
	"BIST",
	"SinkCapabilities",
	"Reserved",
	"Reserved",
	"Reserved",
	"Reserved",
	"Reserved",
	"Reserved",
	"Reserved",
	"Reserved",
	"Reserved",
	"Reserved",
	"VenderDefined"
};

/* Control Message Type */
static const char *CM_message_type[] = {
	"Reserved",
	"GoodCRC",
	"GotoMin",
	"Accept",
	"Reject",
	"Ping",
	"PS_RDY",
	"GetSourceCap",
	"GetSinkCap",
	"DR_Swap",
	"PR_Swap",
	"VCONN_Swap",
	"Wait",
	"SoftReset",
	"Reserved",
	"Reserved"
};

static const char *DMT_message_type[] = {
    "DMTSourceCapabilities",
    "DMTRequest",
    "DMTBIST",
    "DMTSinkCapabilities",
    "DMTVenderDefined"
};

static const char * tx_status[] = {
	"txIdle",
	"txReset",
	"txSend",
	"txBusy",
	"txWait",
	"txSuccess",
	"txError",
	"txCollision"
};

static const char *ProtocolStateName[] = {
	"PRLDisabled",
	"PRLIdle",
	"PRLReset",
	"PRLResetWait",
	"PRLRxWait",
	"PRLTxSendingMessage",
	"PRLTxWaitForPHYResponse",
	"PRLTxVerifyGoodCRC"
};

static const char *PolicyStateName[] = {
	"peDisabled",
	"peErrorRecovery",
	"peSourceHardReset",
	"peSourceSendHardReset",
	"peSourceSoftReset",
	"peSourceSendSoftReset",
	"peSourceStartup",
	"peSourceSendCaps",
	"peSourceDiscovery",
	"peSourceDisabled",
	"peSourceTransitionDefault",
	"peSourceNegotiateCap",
	"peSourceCapabilityResponse",
	"peSourceTransitionSupply",
	"peSourceReady",
	"peSourceGiveSourceCaps",
	"peSourceGetSinkCaps",
	"peSourceSendPing",
	"peSourceGotoMin",
	"peSourceGiveSinkCaps",
	"peSourceGetSourceCaps",
	"peSourceSendDRSwap",
	"peSourceEvaluateDRSwap",
	"peSinkHardReset",
	"peSinkSendHardReset",
	"peSinkSoftReset",
	"peSinkSendSoftReset",
	"peSinkTransitionDefault",
	"peSinkStartup",
	"peSinkDiscovery",
	"peSinkWaitCaps",
	"peSinkEvaluateCaps",
	"peSinkSelectCapability",
	"peSinkTransitionSink",
	"peSinkReady",
	"peSinkGiveSinkCap",
	"peSinkGetSourceCap",
	"peSinkGetSinkCap",
	"peSinkGiveSourceCap",
	"peSinkSendDRSwap",
	"peSinkEvaluateDRSwap",
	"peSourceSendVCONNSwap",
	"peSinkEvaluateVCONNSwap",
	"peSourceSendPRSwap",
	"peSourceEvaluatePRSwap",
	"peSinkSendPRSwap",
	"peSinkEvaluatePRSwap"
};

static const char *TxStatus_name[] = {
	"txIdle",
	"txReset",
	"txSend",
	"txBusy",
	"txWait",
	"txSuccess",
	"txError",
	"txCollision"
};
#endif
/////////////////////////////////////////////////////////////////////////////
//      Variables for use with the USB PD state machine
/////////////////////////////////////////////////////////////////////////////
extern FUSB300reg_t             Registers;                                      // Variable holding the current status of the FUSB300 registers
extern BOOL                     USBPDActive;                                    // Variable to indicate whether the USB PD state machine is active or not
extern BOOL                     USBPDEnabled;                                   // Variable to indicate whether USB PD is enabled (by the host)
extern UINT32                   PRSwapTimer;                                    // Timer used to bail out of a PR_Swap from the Type-C side if necessary
extern USBTypeCPort             PortType;                                       // Variable indicating which type of port we are implementing
extern BOOL                     blnCCPinIsCC1;                                  // Flag to indicate if the CC1 pin has been detected as the CC pin
extern BOOL                     blnCCPinIsCC2;                                  // Flag to indicate if the CC2 pin has been detected as the CC pin
extern BOOL                     blnSMEnabled;                                   // Variable to indicate whether the 300 state machine is enabled
extern ConnectionState          ConnState;                                      // Variable indicating the current Type-C connection state

extern BOOL state_changed;
extern int VBUS_5V_EN;
extern int VBUS_12V_EN;

// Debugging Variables
static UINT8                    USBPDBuf[PDBUFSIZE];                            // Circular buffer of all USB PD messages transferred
static UINT8                    USBPDBufStart;                                  // Pointer to the first byte of the first message
static UINT8                    USBPDBufEnd;                                    // Pointer to the last byte of the last message
static BOOL                     USBPDBufOverflow;                               // Flag to indicate that there was a buffer overflow since last read

// Device Policy Manager Variables
BOOL                            USBPDTxFlag;                                    // Flag to indicate that we need to send a message (set by device policy manager)

sopMainHeader_t                 PDTransmitHeader;                               // Defintion of the PD packet to send
sopMainHeader_t                 CapsHeaderSink;                                 // Definition of the sink capabilities of the device
sopMainHeader_t                 CapsHeaderSource;                               // Definition of the source capabilities of the device
sopMainHeader_t                 CapsHeaderReceived;                             // Last capabilities header received (source or sink)
doDataObject_t                  PDTransmitObjects[7];                           // Data objects to send
doDataObject_t                  CapsSink[7];                                    // Power object definitions of the sink capabilities of the device
doDataObject_t                  CapsSource[7];                                  // Power object definitions of the source capabilities of the device
doDataObject_t                  CapsReceived[7];                                // Last power objects received (source or sink)
doDataObject_t                  USBPDContract;                                  // Current USB PD contract (request object)
doDataObject_t                  SinkRequest;                                    // Sink request message

/* Maximum voltage that the sink will request (12V) <== 240 * 50mv */
static int SinkRequestMaxVoltage = 240;
module_param_named(
	sink_request_max_voltage, SinkRequestMaxVoltage, int, S_IRUSR | S_IWUSR
);

/* Maximum power the sink will request defult 20000 / 2000 = 10W */
static int SinkRequestMaxPower = 20000;
module_param_named(
	sink_request_max_power, SinkRequestMaxPower, int, S_IRUSR | S_IWUSR
);

/* Operating power the sink will request defult 20000 / 2000 = 10W */
static int SinkRequestOpPower = 20000;
module_param_named(
	sink_request_op_power, SinkRequestOpPower, int, S_IRUSR | S_IWUSR
);

BOOL                            SinkGotoMinCompatible;                          // Whether the sink will respond to the GotoMin command
BOOL                            SinkUSBSuspendOperation;                        // Whether the sink wants to continue operation during USB suspend
BOOL                            SinkUSBCommCapable;                             // Whether the sink is USB communications capable
BOOL                            SourceCapsUpdated;                              // Flag to indicate whether we have updated the source caps (for the GUI)

// Policy Variables
 PolicyState_t            PolicyState;                                    // State variable for Policy Engine
static UINT8                    PolicySubIndex;                                 // Sub index for policy states
static BOOL                     PolicyIsSource;                                 // Flag to indicate whether we are acting as a source or a sink
static BOOL                     PolicyIsDFP;                                    // Flag to indicate whether we are acting as a UFP or DFP
static BOOL                     PolicyHasContract;                              // Flag to indicate whether there is a contract in place
static UINT8                    CollisionCounter;                               // Collision counter for the policy engine
static UINT8                    HardResetCounter;                               // The number of times a hard reset has been generated
static UINT8                    CapsCounter;                                    // Number of capabilities messages sent
static UINT32                   PolicyStateTimer;                               // Multi-function timer for the different policy states
static UINT32                   NoResponseTimer;                                // Policy engine no response timer
static sopMainHeader_t          PolicyRxHeader;                                 // Header object for USB PD messages received
static sopMainHeader_t          PolicyTxHeader;                                 // Header object for USB PD messages to send
static doDataObject_t           PolicyRxDataObj[7];                             // Buffer for data objects received
static doDataObject_t           PolicyTxDataObj[7];                             // Buffer for data objects to send

// Protocol Variables
ProtocolState_t          ProtocolState;                                  // State variable for Protocol Layer
PDTxStatus_t             PDTxStatus;                                     // Status variable for current transmission
static UINT8                    MessageIDCounter;                               // Current Tx message ID counter
static UINT8                    MessageID;                                      // Last received message ID
static BOOL                     ProtocolMsgRx;                                  // Flag to indicate if we have received a packet
static BOOL                     ProtocolMsgTx;
static UINT8                    ProtocolTxBytes;                                // Number of bytes for the Tx FIFO
static UINT8                    ProtocolTxBuffer[64];                           // Buffer for FUSB300 Tx FIFO (same size as the 300 tx buffer)
static UINT8                    ProtocolRxBuffer[64];                           // Buffer for FUSB300 Rx FIFO (same size as the 300 rx buffer)
static UINT16                   ProtocolTimer;                                  // Multi-function timer for the different protocol states
static UINT8                    ProtocolCRC[4];

// VDM Manager object
VdmManager                      vdmm;

#define FUSB_MS_TO_NS(x) (x * 1000 * 1000)
#define Delay10us(x) udelay(x*10);

extern void wake_up_statemachine(void);

static struct hrtimer protocol_hrtimer;
static struct hrtimer policystate_hrtimer;
static struct hrtimer noresponse_hrtimer;
static struct hrtimer prswap_hrtimer;

void set_policy_state(PolicyState_t st, const char *caller_name, int line_number)
{
    PolicyState = st;
    state_changed = TRUE;
    PD_LOG("set PolicyState to =======>%s, by %s, %d\n", PolicyStateName[st], caller_name, line_number);
}

void set_protocol_state(ProtocolState_t st, const char *caller_name, int line_number)
{
    ProtocolState = st;
    state_changed = TRUE;
    PD_LOG("set ProtocolState to =======>%s, by %s, %d\n", ProtocolStateName[st], caller_name, line_number);
}

void set_pdtx_state(PDTxStatus_t st, const char *caller_name, int line_number)
{
    PDTxStatus = st;
    state_changed = TRUE;
    PD_LOG("set PDTxStatus to =======>%s, by %s, %d\n", tx_status[st], caller_name, line_number);
}

void set_policy_subindex(UINT8 index)
{
    PolicySubIndex = index;
    state_changed = TRUE;
}

void increase_policy_subindex(void)
{
    PolicySubIndex++;
    state_changed = TRUE;
}

void usbpd_start_timer(struct hrtimer* timer, int ms)
{
    ktime_t ktime;
    ktime = ktime_set(0, FUSB_MS_TO_NS(ms));
    hrtimer_start(timer, ktime, HRTIMER_MODE_REL);
}

enum hrtimer_restart pd_func_hrtimer(struct hrtimer *timer)
{
    if (timer == &protocol_hrtimer)
        ProtocolTimer = 0;
    else if (timer == &policystate_hrtimer)
        PolicyStateTimer = 0;
    else if (timer == &noresponse_hrtimer)
        NoResponseTimer = 0;
    else if (timer == &prswap_hrtimer)
	PRSwapTimer = 0;

    PD_LOG("%s, ProtocolTimer=%d, PolicyStateTimer=%d, NoResponseTimer=%d, PRSwapTimer=%d\n",
        __func__, ProtocolTimer, PolicyStateTimer, NoResponseTimer, PRSwapTimer);

    wake_up_statemachine();

    return HRTIMER_NORESTART;
}

int FUSB300WriteFIFO(unsigned char length, unsigned char *data)
{
    unsigned char llen = length;
    unsigned char i = 0;
    int ret = 0;
    char log[200];
    //PD_LOG("%s, length=%d\n", __func__, length);
    ret = sprintf(log,">>>>");
    for(i=0;i<length;i++)
        ret += sprintf(log+ret, " %2x", data[i]);
    PD_LOG("%s\n", log);

    ret = 0;
    i = 0;

    while (llen > 4 )
    {
        ret = FUSB300Write(regFIFO, 4, &data[i<<2]);
        i++;
        llen -= 4;
    }
    if (llen > 0 )
        ret = FUSB300Write(regFIFO, llen, &data[i<<2]);
    return ret;
}

int FUSB300ReadFIFO(unsigned char length, unsigned char *data)
{
    unsigned char llen = length;
    unsigned char i = 0;
    int ret = 0;
    char log[200];
    //PD_LOG("%s, length=%d\n", __func__, length);

    while (llen > 8 )
    {
        ret = FUSB300Read(regFIFO, 8, &data[i<<3]);
        i++;
        llen -= 8;
    }
    if (llen > 0 )
        ret = FUSB300Read(regFIFO, llen, &data[i<<3]);

    ret = sprintf(log, "<<<<<");
    for(i=0; i<length; i++)
        ret += sprintf(log+ret, " %2x", data[i]);
    PD_LOG("%s\n", log);

    return ret;
}

void InitializeUSBPDVariables(void)
{
    USBPDBufStart = 0;                                                          // Reset the starting pointer for the circular buffer
    USBPDBufEnd = 0;                                                            // Reset the ending pointer for the circular buffer
    USBPDBufOverflow = FALSE;                                                   // Clear the overflow flag for the circular buffer
    SinkGotoMinCompatible = FALSE;                                              // Whether the sink will respond to the GotoMin command
    SinkUSBSuspendOperation = FALSE;                                            // Whether the sink wants to continue operation during USB suspend
    SinkUSBCommCapable = FALSE;                                                 // Whether the sink is USB communications capable
    SourceCapsUpdated = FALSE;                                                  // Set the flag to indicate to the GUI that our source caps have been updated

    CapsHeaderSource.NumDataObjects = 2;                                        // Set the number of power objects to 2
    CapsHeaderSource.PortDataRole = 0;                                          // Set the data role to UFP by default
    CapsHeaderSource.PortPowerRole = 1;                                         // By default, set the device to be a source
    CapsHeaderSource.SpecRevision = 1;                                          // Set the spec revision to 2.0
    CapsSource[0].FPDOSupply.Voltage = 100;                                     // Set 5V for the first supply option
    CapsSource[0].FPDOSupply.MaxCurrent = 100;                                  // Set 1000mA for the first supply option
    CapsSource[0].FPDOSupply.PeakCurrent = 0;                                   // Set peak equal to max
    CapsSource[0].FPDOSupply.DataRoleSwap = TRUE;                               // By default, don't enable DR_SWAP
    CapsSource[0].FPDOSupply.USBCommCapable = FALSE;                            // By default, USB communications is not allowed
    CapsSource[0].FPDOSupply.ExternallyPowered = TRUE;                          // By default, state that we are externally powered
    CapsSource[0].FPDOSupply.USBSuspendSupport = FALSE;                         // By default, don't allow  USB Suspend
    CapsSource[0].FPDOSupply.DualRolePower = TRUE;                              // By default, don't enable PR_SWAP
    CapsSource[0].FPDOSupply.SupplyType = 0;                                    // Fixed supply

    CapsSource[1].FPDOSupply.Voltage = 240;                                     // Set 12V for the second supply option
    CapsSource[1].FPDOSupply.MaxCurrent = 150;                                  // Set 1500mA for the second supply option
    CapsSource[1].FPDOSupply.PeakCurrent = 0;                                   // Set peak equal to max
    CapsSource[1].FPDOSupply.DataRoleSwap = 0;                                  // Not used... set to zero
    CapsSource[1].FPDOSupply.USBCommCapable = 0;                                // Not used... set to zero
    CapsSource[1].FPDOSupply.ExternallyPowered = 0;                             // Not used... set to zero
    CapsSource[1].FPDOSupply.USBSuspendSupport = 0;                             // Not used... set to zero
    CapsSource[1].FPDOSupply.DualRolePower = 0;                                 // Not used... set to zero
    CapsSource[1].FPDOSupply.SupplyType = 0;                                    // Fixed supply

    CapsHeaderSink.NumDataObjects = 2;                                          // Set the number of power objects to 2
    CapsHeaderSink.PortDataRole = 0;                                            // Set the data role to UFP by default
    CapsHeaderSink.PortPowerRole = 0;                                           // By default, set the device to be a sink
    CapsHeaderSink.SpecRevision = 1;                                            // Set the spec revision to 2.0
    CapsSink[0].FPDOSink.Voltage = 100;                                         // Set 5V for the first supply option
    CapsSink[0].FPDOSink.OperationalCurrent = 10;                               // Set that our device will consume 100mA for this object
    CapsSink[0].FPDOSink.DataRoleSwap = 0;                                      // By default, don't enable DR_SWAP
    CapsSink[0].FPDOSink.USBCommCapable = 0;                                    // By default, USB communications is not allowed
    CapsSink[0].FPDOSink.ExternallyPowered = 0;                                 // By default, we are not externally powered
    CapsSink[0].FPDOSink.HigherCapability = FALSE;                              // By default, don't require more than vSafe5V
    CapsSink[0].FPDOSink.DualRolePower = 0;                                     // By default, don't enable PR_SWAP

    CapsSink[1].FPDOSink.Voltage = 240;                                         // Set 12V for the second supply option
    CapsSink[1].FPDOSink.OperationalCurrent = 10;                               // Set that our device will consume 100mA for this object
    CapsSink[1].FPDOSink.DataRoleSwap = 0;                                      // Not used
    CapsSink[1].FPDOSink.USBCommCapable = 0;                                    // Not used
    CapsSink[1].FPDOSink.ExternallyPowered = 0;                                 // Not used
    CapsSink[1].FPDOSink.HigherCapability = 0;                                  // Not used
    CapsSink[1].FPDOSink.DualRolePower = 0;                                     // Not used

    initialize_vdm(&vdmm);                                                          // Initialize VDM Manager memory

    hrtimer_init(&protocol_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    protocol_hrtimer.function = pd_func_hrtimer;

    hrtimer_init(&policystate_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    policystate_hrtimer.function = pd_func_hrtimer;

    hrtimer_init(&noresponse_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    noresponse_hrtimer.function = pd_func_hrtimer;

    hrtimer_init(&prswap_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    prswap_hrtimer.function = pd_func_hrtimer;
}

// ##################### USB PD Enable / Disable Routines ################### //

void USBPDEnable(BOOL FUSB300Update, BOOL TypeCDFP)
{
    UINT8 data[5];
    if (USBPDEnabled == TRUE)
    {
        if (blnCCPinIsCC1)                                                      // If the CC pin is on CC1
            Registers.Switches.TXCC1 = TRUE;                                    // Enable the BMC transmitter on CC1
        else if (blnCCPinIsCC2)                                                 // If the CC pin is on CC2
            Registers.Switches.TXCC2 = TRUE;                                    // Enable the BMC transmitter on CC2
        if (blnCCPinIsCC1 || blnCCPinIsCC2)                                     // If we know what pin the CC signal is...
        {
            USBPDActive = TRUE;                                                 // Set the active flag
            ResetProtocolLayer(FALSE);                                          // Reset the protocol layer by default
            NoResponseTimer = USHRT_MAX;                                        // Disable the no response timer by default
            PolicyIsSource = TypeCDFP;                                          // Set whether we should be initially a source or sink
            PolicyIsDFP = TypeCDFP;                                             // Set the initial data port direction
            if (PolicyIsSource)                                                 // If we are a source...
            {
				PD_LOG("PolicyIsSource\n");
                set_policy_state(peSourceStartup, __FUNCTION__, __LINE__);                                  // initialize the policy engine state to source startup
                Registers.Switches.POWERROLE = 1;                               // Initialize to a SRC
                Registers.Switches.DATAROLE = 1;                                // Initialize to a DFP
            }
            else                                                                // Otherwise we are a sink...
            {
				PD_LOG("PolicyIsSink\n");
                set_policy_state(peSinkStartup, __FUNCTION__, __LINE__);                                    // initialize the policy engine state to sink startup
                Registers.Switches.POWERROLE = 0;                               // Initialize to a SNK
                Registers.Switches.DATAROLE = 0;                                // Initialize to a UFP
            }
            Registers.Switches.AUTO_CRC = 1;
            Registers.Power.PWR |= 0x08;                                        // Enable the internal oscillator for USB PD
            Registers.Control.AUTO_PRE = 0;                                     // Disable AUTO_PRE since we are going to use AUTO_CRC
            Registers.Control.N_RETRIES = 2;                                    // Set the number of retries to 0 (handle all retries in firmware for now)
            Registers.Control.AUTO_RETRY = 1;                                   // Enable AUTO_RETRY to use the I_TXSENT interrupt
            Registers.Slice.SDAC = SDAC_DEFAULT;                                // Set the SDAC threshold ~0.544V
            data[0] = Registers.Slice.byte;                                     // Set the slice byte (building one transaction)
            data[1] = Registers.Control.byte[0] | 0x40;                         // Set the Control0 byte and set the TX_FLUSH bit (auto-clears)
            data[2] = Registers.Control.byte[1] | 0x04;                         // Set the Control1 byte and set the RX_FLUSH bit (auto-clears)
            data[3] = Registers.Control.byte[2];
            data[4] = Registers.Control.byte[3];
            FUSB300Write(regSlice, 5, &data[0]);                                // Commit the slice and control registers
            if (FUSB300Update)
            {
                FUSB300Write(regPower, 1, &Registers.Power.byte);               // Commit the power setting
                FUSB300Write(regSwitches1, 1, &Registers.Switches.byte[1]);     // Commit the switch1 setting
            }
            StoreUSBPDToken(TRUE, pdtAttach);                                   // Store the PD attach token
 //           InitializeUSBPDTimers(TRUE);                                        // Enable the USB PD timers
        }
    }
}

void USBPDDisable(BOOL FUSB300Update)
{
    if (USBPDActive == TRUE) // If we were previously active...
        StoreUSBPDToken(TRUE, pdtDetach);                                       // Store the PD detach token

    USBPDActive = FALSE;                                                        // Clear the USB PD active flag
    set_protocol_state(PRLDisabled, __FUNCTION__, __LINE__);                                                // Set the protocol layer state to disabled
    set_policy_state(peDisabled, __FUNCTION__, __LINE__);                                                   // Set the policy engine state to disabled
    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                                        // Reset the transmitter status
    PolicyIsSource = FALSE;                                                     // Clear the is source flag until we connect again
    PolicyHasContract = FALSE;                                                  // Clear the has contract flag
    SourceCapsUpdated = TRUE;                                                   // Set the source caps updated flag to trigger an update of the GUI
    if (FUSB300Update)
    {
        Registers.Switches.byte[1] = 0;                                         // Disable the BMC transmitter (both CC1 & CC2)
        Registers.Power.PWR &= 0x07;                                            // Disable the internal oscillator
        FUSB300Write(regPower, 1, &Registers.Power.byte);                       // Commit the power setting
        FUSB300Write(regSwitches1, 1, &Registers.Switches.byte[1]);             // Commit the switch setting
    }
}

// ##################### USB PD Policy Engine Routines ###################### //

void USBPDPolicyEngine(void)
{
	PD_LOG("%s: PolicyState is %s\n", __FUNCTION__, PolicyStateName[PolicyState]);
    switch (PolicyState)
    {
        case peErrorRecovery:
            PolicyErrorRecovery();
            break;
        // ###################### Source States  ##################### //
        case peSourceSendHardReset:
            PolicySourceSendHardReset();
            break;
        case peSourceSendSoftReset:
            PolicySourceSendSoftReset();
            break;
        case peSourceSoftReset:
            PolicySourceSoftReset();
            break;
        case peSourceStartup:
            PolicySourceStartup();
            break;
        case peSourceDiscovery:
            PolicySourceDiscovery();
            break;
        case peSourceSendCaps:
            PolicySourceSendCaps();
            break;
        case peSourceDisabled:
            PolicySourceDisabled();
            break;
        case peSourceTransitionDefault:
            PolicySourceTransitionDefault();
            break;
        case peSourceNegotiateCap:
            PolicySourceNegotiateCap();
            break;
        case peSourceCapabilityResponse:
            PolicySourceCapabilityResponse();
            break;
        case peSourceTransitionSupply:
            PolicySourceTransitionSupply();
            break;
        case peSourceReady:
            PolicySourceReady();
            break;
        case peSourceGiveSourceCaps:
            PolicySourceGiveSourceCap();
            break;
        case peSourceGetSinkCaps:
            PolicySourceGetSinkCap();
            break;
        case peSourceSendPing:
            PolicySourceSendPing();
            break;
        case peSourceGotoMin:
            PolicySourceGotoMin();
            break;
        case peSourceGiveSinkCaps:
            PolicySourceGiveSinkCap();
            break;
        case peSourceGetSourceCaps:
            PolicySourceGetSourceCap();
            break;
        case peSourceSendDRSwap:
            PolicySourceSendDRSwap();
            break;
        case peSourceEvaluateDRSwap:
            PolicySourceEvaluateDRSwap();
            break;
        case peSourceSendVCONNSwap:
            PolicySourceSendVCONNSwap();
            break;
        case peSourceSendPRSwap:
            PolicySourceSendPRSwap();
            break;
        case peSourceEvaluatePRSwap:
            PolicySourceEvaluatePRSwap();
            break;
        // ###################### Sink States  ####################### //
        case peSinkStartup:
            PolicySinkStartup();
            break;
        case peSinkSendHardReset:
            PolicySinkSendHardReset();
            break;
        case peSinkSoftReset:
            PolicySinkSoftReset();
            break;
        case peSinkSendSoftReset:
            PolicySinkSendSoftReset();
            break;
        case peSinkTransitionDefault:
            PolicySinkTransitionDefault();
            break;
        case peSinkDiscovery:
            PolicySinkDiscovery();
            break;
        case peSinkWaitCaps:
            PolicySinkWaitCaps();
            break;
        case peSinkEvaluateCaps:
            PolicySinkEvaluateCaps();
            break;
        case peSinkSelectCapability:
            PolicySinkSelectCapability();
            break;
        case peSinkTransitionSink:
            PolicySinkTransitionSink();
            break;
        case peSinkReady:
            PolicySinkReady();
            break;
        case peSinkGiveSinkCap:
            PolicySinkGiveSinkCap();
            break;
        case peSinkGetSourceCap:
            PolicySinkGetSourceCap();
            break;
        case peSinkGetSinkCap:
            PolicySinkGetSinkCap();
            break;
        case peSinkGiveSourceCap:
            PolicySinkGiveSourceCap();
            break;
        case peSinkSendDRSwap:
            PolicySinkSendDRSwap();
            break;
        case peSinkEvaluateDRSwap:
            PolicySinkEvaluateDRSwap();
            break;
        case peSinkEvaluateVCONNSwap:
            PolicySinkEvaluateVCONNSwap();
            break;
        case peSinkSendPRSwap:
            PolicySinkSendPRSwap();
            break;
        case peSinkEvaluatePRSwap:
            PolicySinkEvaluatePRSwap();
            break;
        default:
            break;
    }
    USBPDTxFlag = FALSE;                                                        // Clear the transmit flag after going through the loop to avoid sending a message inadvertantly
}

// ############################# Source States  ############################# //

void PolicyErrorRecovery(void)
{
    SetStateErrorRecovery();
}

void PolicySourceSendHardReset(void)
{
    PolicySendHardReset(peSourceTransitionDefault, tPSHardReset);
}

void PolicySourceSoftReset(void)
{
    PolicySendCommand(CMTAccept, peSourceSendCaps, 0);
}

void PolicySourceSendSoftReset(void)
{
    switch (PolicySubIndex)
    {
        case 0:
            if (PolicySendCommand(CMTSoftReset, peSourceSendSoftReset, 1) == STAT_SUCCESS) // Send the soft reset command to the protocol layer
            {
                PolicyStateTimer = tSenderResponse;                             // Start the sender response timer to wait for an accept message once successfully sent
                usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
            }
            break;
        default:
            if (ProtocolMsgRx)                                                  // If we have received a message
            {
                ProtocolMsgRx = FALSE;                                          // Reset the message received flag since we've handled it here
				PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
                if ((PolicyRxHeader.NumDataObjects == 0) && (PolicyRxHeader.MessageType == CMTAccept))  // And it was the Accept...
                {
                    set_policy_state(peSourceSendCaps, __FUNCTION__, __LINE__);                             // Go to the send caps state
                }
                else                                                            // Otherwise it was a message that we didn't expect, so...
                    set_policy_state(peSourceSendHardReset, __FUNCTION__, __LINE__);                        // Go to the hard reset state
                set_policy_subindex(0);                                             // Reset the sub index
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Clear the transmitter status
            }
            else if (!PolicyStateTimer)                                         // If we didn't get a response to our request before timing out...
            {
                set_policy_state(peSourceSendHardReset, __FUNCTION__, __LINE__);                            // Go to the hard reset state
                set_policy_subindex(0);                                             // Reset the sub index
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Clear the transmitter status
            }
            break;
    }
}

void PolicySourceStartup(void)
{
    PolicyIsSource = TRUE;                                                      // Set the flag to indicate that we are a source (PRSwaps)
    ResetProtocolLayer(TRUE);                                                   // Reset the protocol layer
    PRSwapTimer = 0;                                                            // Clear the swap timer
    CapsCounter = 0;                                                            // Clear the caps counter
    CollisionCounter = 0;                                                       // Reset the collision counter
    PolicyStateTimer = 0;                                                       // Reset the policy state timer
    set_policy_state(peSourceSendCaps, __FUNCTION__, __LINE__);                                             // Go to the source caps
    set_policy_subindex(0);                                                         // Reset the sub index
}

void PolicySourceDiscovery(void)
{
    switch (PolicySubIndex)
    {
        case 0:
            PolicyStateTimer = tTypeCSendSourceCap;                             // Initialize the SourceCapabilityTimer
            usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
            increase_policy_subindex();                                                   // Increment the sub index
            break;
        default:
            if ((HardResetCounter > nHardResetCount) && (NoResponseTimer == 0))
            {
                if (PolicyHasContract)                                          // If we previously had a contract in place...
                    set_policy_state(peErrorRecovery, __FUNCTION__, __LINE__);                              // Go to the error recovery state since something went wrong
                else                                                            // Otherwise...
                    set_policy_state(peSourceDisabled, __FUNCTION__, __LINE__);                             // Go to the disabled state since we are assuming that there is no PD sink attached
                set_policy_subindex(0);                                             // Reset the sub index for the next state
            }
            else if (PolicyStateTimer == 0)                                     // Once the timer expires...
            {
                if (CapsCounter > nCapsCount)                                   // If we have sent the maximum number of capabilities messages...
                    set_policy_state(peSourceDisabled, __FUNCTION__, __LINE__);                             // Go to the disabled state, no PD sink connected
                else                                                            // Otherwise...
                    set_policy_state(peSourceSendCaps, __FUNCTION__, __LINE__);                             // Go to the send source caps state to send a source caps message
                set_policy_subindex(0);                                             // Reset the sub index for the next state
            }
            break;
    }
}

void PolicySourceSendCaps(void)
{
    if ((HardResetCounter > nHardResetCount) && (NoResponseTimer == 0))         // Check our higher level timeout
    {
        if (PolicyHasContract)                                                  // If USB PD was previously established...
            set_policy_state(peErrorRecovery, __FUNCTION__, __LINE__);                                      // Need to go to the error recovery state
        else                                                                    // Otherwise...
            set_policy_state(peSourceDisabled, __FUNCTION__, __LINE__);                                     // We are disabling PD and leaving the Type-C connections alone
    }
    else                                                                        // If we haven't timed out and maxed out on hard resets...
    {
        switch (PolicySubIndex)
        {
            case 0:
                if (PolicySendData(DMTSourceCapabilities, CapsHeaderSource.NumDataObjects, &CapsSource[0], peSourceSendCaps, 1) == STAT_SUCCESS)
                {
                    HardResetCounter = 0;                                       // Clear the hard reset counter
                    CapsCounter = 0;                                            // Clear the caps counter
                    NoResponseTimer = USHRT_MAX;                                // Stop the no response timer
                    PolicyStateTimer = tSenderResponse;                         // Set the sender response timer
                    usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                }
                break;
            default:
                if (ProtocolMsgRx)                                              // If we have received a message
                {
                    ProtocolMsgRx = FALSE;                                      // Reset the message ready flag since we're handling it here
					PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
                    if ((PolicyRxHeader.NumDataObjects == 1) && (PolicyRxHeader.MessageType == DMTRequest)) // Was this a valid request message?
                        set_policy_state(peSourceNegotiateCap, __FUNCTION__, __LINE__);                     // If so, go to the negotiate capabilities state
                    else                                                        // Otherwise it was a message that we didn't expect, so...
                        set_policy_state(peSourceSendHardReset, __FUNCTION__, __LINE__);                    // Go onto issuing a hard reset
                    set_policy_subindex(0);                                         // Reset the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                }
                else if (!PolicyStateTimer)                                     // If we didn't get a response to our request before timing out...
                {
                    ProtocolMsgRx = FALSE;                                      // Reset the message ready flag since we've timed out
					PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
                    set_policy_state(peSourceSendHardReset, __FUNCTION__, __LINE__);                        // Go to the hard reset state
                    set_policy_subindex(0);                                         // Reset the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                }
                break;
        }
    }
}

void PolicySourceDisabled(void)
{
    USBPDContract.object = 0;                                                   // Clear the USB PD contract (output power to 5V default)
    // Wait for a hard reset or detach...
}

void PolicySourceTransitionDefault(void)
{
	PD_LOG("PolicySubIndex = %d, PolicyStateTimer = %d\n", PolicySubIndex, PolicyStateTimer);
    switch (PolicySubIndex)
    {
        case 0:
            VBUS_5V_EN = 0;                                                     // Disable the 5V source
            VBUS_12V_EN = 0;                                                    // Disable the 12V source
            USBPDContract.object = 0; // Clear the USB PD contract (output power to 5V default)
            PolicyStateTimer = tPSSourceOffNom; // Initialize the tPSTransition timer as a starting point for allowing the voltage to drop to 0
            usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
            increase_policy_subindex();
            // Adjust output if necessary and start timer prior to going to startup state?
            break;
        case 1:
            if (PolicyStateTimer ==0)                                           // Once the timer expires...
            {
                VBUS_5V_EN = 1;                                                 // Enable the 5V source
                PolicyStateTimer = tTypeCSendSourceCap;                         // Set the timer to allow for the sink to detect VBUS
                usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                increase_policy_subindex();                                               // Move onto the next state
            }
            break;
        default:
            if (PolicyStateTimer == 0)
            {
                NoResponseTimer = tNoResponse;                                  // Initialize the no response timer
                usbpd_start_timer(&noresponse_hrtimer, NoResponseTimer);
                set_policy_state(peSourceStartup, __FUNCTION__, __LINE__);                                  // Go to the startup state
                set_policy_subindex(0);                                             // Reset the sub index
            }
            break;
    }
}

void PolicySourceNegotiateCap(void)
{
    // This state evaluates if the sink request can be met or not and sets the next state accordingly
    BOOL reqAccept = FALSE;                                                     // Set a flag that indicates whether we will accept or reject the request
    UINT8 objPosition;                                                          // Get the requested object position
    objPosition = PolicyRxDataObj[0].FVRDO.ObjectPosition;                      // Get the object position reference
    if ((objPosition > 0) && (objPosition <= CapsHeaderSource.NumDataObjects))  // Make sure the requested object number if valid, continue validating request
    {
        if (PolicyRxDataObj[0].FVRDO.OpCurrent <= CapsSource[objPosition-1].FPDOSupply.MaxCurrent) // Ensure the default power/current request is available
            reqAccept = TRUE;                                                   // If the source can supply the request, set the flag to respond
    }
    if (reqAccept)                                                              // If we have received a valid request...
    {
        set_policy_state(peSourceTransitionSupply, __FUNCTION__, __LINE__);                                 // Go to the transition supply state

    }
    else                                                                        // Otherwise the request was invalid...
        set_policy_state(peSourceCapabilityResponse, __FUNCTION__, __LINE__);                               // Go to the capability response state to send a reject/wait message
}

void PolicySourceTransitionSupply(void)
{
    UINT8 objPosition;
    PD_LOG("VBUS_12V_EN=%d, VBUS_5V_EN=%d, PolicySubIndex=%d\n", VBUS_12V_EN, VBUS_5V_EN, PolicySubIndex);
    switch (PolicySubIndex)
    {
        case 0:
            PolicySendCommand(CMTAccept, peSourceTransitionSupply, 1);          // Send the Accept message
            break;
        case 1:
            PolicyStateTimer = tSnkTransition;                                  // Initialize the timer to allow for the sink to transition
            usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
            increase_policy_subindex();                                                   // Increment to move to the next sub state
            break;
        case 2:
            if (!PolicyStateTimer) {                                             // If the timer has expired (the sink is ready)...
                increase_policy_subindex();                                               // Increment to move to the next sub state
            }
            break;
        case 3:
            PolicyHasContract = TRUE;                                           // Set the flag to indicate that a contract is in place
            USBPDContract.object = PolicyRxDataObj[0].object;                   // Set the contract to the sink request
            objPosition = USBPDContract.FVRDO.ObjectPosition;
            if ((CapsSource[objPosition-1].FPDOSupply.SupplyType == 0) && (CapsSource[objPosition-1].FPDOSupply.Voltage == 240))
            {
                if (VBUS_12V_EN)                                                // If we already have the 12V enabled
                {
                    set_policy_subindex(5);                                         // go directly to sending the PS_RDY
                }
                else                                                            // Otherwise we need to transition the supply
                {
                    VBUS_5V_EN = 1;                                             // Keep the 5V source enabled so that we don't droop the line
                    VBUS_12V_EN = 1;                                            // Enable the 12V source (takes 4.3ms to enable, 3.0ms to rise)
                    PolicyStateTimer = tFPF2498Transition;                      // Set the policy state timer to allow the load switch time to turn off so we don't short our supplies
                    usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                    increase_policy_subindex();                                           // Move onto the next state to turn on our 12V supply
                }
            }
            else
            {
                if (VBUS_12V_EN)
                {
                    VBUS_5V_EN = 1;                                             // Ensure that the 5V source is enabled
                    VBUS_12V_EN = 0;                                            // disable the 12V source (takes 600us to disable, 2ms for the output to fall)
                    PolicyStateTimer = tFPF2498Transition;                      // Set the policy state timer to allow the load switch time to turn off so we don't short our supplies
                    usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                    increase_policy_subindex();                                           // Move onto the next state to turn on our 12V supply
                }
                else
                {
                    set_policy_subindex(5);                                         // go directly to sending the PS_RDY
                }
            }
            break;
        case 4:
            // Validate the output is ready prior to sending the ready message (only using a timer for now, could validate using an ADC as well)
            if (PolicyStateTimer == 0) {
                increase_policy_subindex();                                               // Increment to move to the next sub state
           }
           break;
        default:
            PolicySendCommand(CMTPS_RDY, peSourceReady, 0);                     // Send the PS_RDY message and move onto the Source Ready state
            break;
    }
}

void PolicySourceCapabilityResponse(void)
{
    if (PolicyHasContract)                                                      // If we currently have a contract, issue the reject and move back to the ready state
        PolicySendCommand(CMTReject, peSourceReady, 0);                         // Send the message and continue onto the ready state
    else                                                                        // If there is no contract in place, issue a hard reset
        PolicySendCommand(CMTReject, peSourceSendHardReset, 0);                 // Send the message and continue onto the hard reset state after success
}

void PolicySourceReady(void)
{
    if (ProtocolMsgRx)                                                          // Have we received a message from the sink?
    {
        ProtocolMsgRx = FALSE;                                                  // Reset the message received flag since we're handling it here
		PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
        if (PolicyRxHeader.NumDataObjects == 0)                                 // If we have received a command
        {
            switch (PolicyRxHeader.MessageType)                                 // Determine which command was received
            {
                case CMTGetSourceCap:
                    set_policy_state(peSourceGiveSourceCaps, __FUNCTION__, __LINE__);                       // Send out the caps
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                case CMTGetSinkCap:                                             // If we receive a get sink capabilities message...
                    set_policy_state(peSourceGiveSinkCaps, __FUNCTION__, __LINE__);                         // Go evaluate whether we are going to send sink caps or reject
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                case CMTDR_Swap:                                                // If we get a DR_Swap message...
                    set_policy_state(peSourceEvaluateDRSwap, __FUNCTION__, __LINE__);                       // Go evaluate whether we are going to accept or reject the swap
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                case CMTPR_Swap:
                    set_policy_state(peSourceEvaluatePRSwap, __FUNCTION__, __LINE__);                       // Go evaluate whether we are going to accept or reject the swap
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                case CMTSoftReset:
                    set_policy_state(peSourceSoftReset, __FUNCTION__, __LINE__);                            // Go to the soft reset state
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                default:                                                        // Send a reject message for all other commands
                    break;
            }
        }
        else                                                                    // If we received a data message... for now just send a soft reset
        {
            switch (PolicyRxHeader.MessageType)
            {
                case DMTRequest:
                    set_policy_state(peSourceNegotiateCap, __FUNCTION__, __LINE__);                         // If we've received a request object, go to the negotiate capabilities state
                    break;
                case DMTVenderDefined:
                    convertAndProcessVdmMessage();
                    break;
                default:                                                        // Otherwise we've received a message we don't know how to handle yet
                    break;
            }
            set_policy_subindex(0);                                                 // Clear the sub index
            set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                                // Clear the transmitter status
        }
    }
    else if (USBPDTxFlag)                                                       // Has the device policy manager requested us to send a message?
    {
        if (PDTransmitHeader.NumDataObjects == 0)
        {
            switch (PDTransmitHeader.MessageType)                               // Determine which command we need to send
            {
                case CMTGetSinkCap:
                    set_policy_state(peSourceGetSinkCaps, __FUNCTION__, __LINE__);                          // Go to the get sink caps state
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                case CMTGetSourceCap:
                    set_policy_state(peSourceGetSourceCaps, __FUNCTION__, __LINE__);                        // Go to the get source caps state
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                case CMTPing:
                    set_policy_state(peSourceSendPing, __FUNCTION__, __LINE__);                             // Go to the send ping state
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                case CMTGotoMin:
                    set_policy_state(peSourceGotoMin, __FUNCTION__, __LINE__);                              // Go to the source goto min state
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                case CMTPR_Swap:
                    if (PortType == USBTypeC_DRP)                               // Only send if we are configured as a DRP
                    {
                        set_policy_state(peSourceSendPRSwap, __FUNCTION__, __LINE__);                       // Issue a DR_Swap message
                        set_policy_subindex(0);                                     // Clear the sub index
                        set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                    // Clear the transmitter status
                    }
                    break;
                case CMTDR_Swap:
                    if (PortType == USBTypeC_DRP)                               // Only send if we are configured as a DRP
                    {
                        set_policy_state(peSourceSendDRSwap, __FUNCTION__, __LINE__);                       // Issue a DR_Swap message
                        set_policy_subindex(0);                                     // Clear the sub index
                        set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                    // Clear the transmitter status
                    }
                    break;
                case CMTVCONN_Swap:
                    set_policy_state(peSourceSendVCONNSwap, __FUNCTION__, __LINE__);                        // Issue a VCONN_Swap message
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                case CMTSoftReset:
                    set_policy_state(peSourceSendSoftReset, __FUNCTION__, __LINE__);                        // Go to the soft reset state
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                default:                                                        // Don't send any commands we don't know how to handle yet
                    break;
            }
        }
        else
        {
            switch (PDTransmitHeader.MessageType)
            {
                case DMTSourceCapabilities:
                    set_policy_state(peSourceSendCaps, __FUNCTION__, __LINE__);
                    set_policy_subindex(0);
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);
                    break;
                case DMTVenderDefined:
                    // not worrying about transitioning states right now - TODO
                    set_policy_subindex(0); // Do I need this here? - Gabe
                    doVdmCommand();
                    break;
                default:
                    break;
            }
        }
    }
}

void PolicySourceGiveSourceCap(void)
{
    PolicySendData(DMTSourceCapabilities, CapsHeaderSource.NumDataObjects, &CapsSource[0], peSourceReady, 0);
}

void PolicySourceGetSourceCap(void)
{
    PolicySendCommand(CMTGetSourceCap, peSourceReady, 0);
}

void PolicySourceGetSinkCap(void)
{
    switch (PolicySubIndex)
    {
        case 0:
            if (PolicySendCommand(CMTGetSinkCap, peSourceGetSinkCaps, 1) == STAT_SUCCESS) // Send the get sink caps command upon entering state
            {
                PolicyStateTimer = tSenderResponse;                             // Start the sender response timer upon receiving the good CRC message
                usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
            }
            break;
        default:
            if (ProtocolMsgRx)                                                  // If we have received a message
            {
                ProtocolMsgRx = FALSE;                                          // Reset the message ready flag since we're handling it here
				PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
                if ((PolicyRxHeader.NumDataObjects > 0) && (PolicyRxHeader.MessageType == DMTSinkCapabilities))
                {
                    UpdateCapabilitiesRx(FALSE);
                    set_policy_state(peSourceReady, __FUNCTION__, __LINE__);                                // Go onto the source ready state
                }
                else                                                            // If we didn't receive a valid sink capabilities message...
                {
                    set_policy_state(peSourceSendHardReset, __FUNCTION__, __LINE__);                        // Go onto issuing a hard reset
                }
                set_policy_subindex(0);                                             // Reset the sub index
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Clear the transmitter status
            }
            else if (!PolicyStateTimer)                                         // If we didn't get a response to our request before timing out...
            {
                set_policy_state(peSourceSendHardReset, __FUNCTION__, __LINE__);                            // Go to the hard reset state
                set_policy_subindex(0);                                             // Reset the sub index
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Clear the transmitter status
            }
            break;
    }
}

void PolicySourceGiveSinkCap(void)
{
    if (PortType == USBTypeC_DRP)
        PolicySendData(DMTSinkCapabilities, CapsHeaderSink.NumDataObjects, &CapsSink[0], peSourceReady, 0);
    else
        PolicySendCommand(CMTReject, peSourceReady, 0);                         // Send the reject message and continue onto the ready state
}

void PolicySourceSendPing(void)
{
    PolicySendCommand(CMTPing, peSourceReady, 0);
}

void PolicySourceGotoMin(void)
{
    if (ProtocolMsgRx)
    {
        ProtocolMsgRx = FALSE;                                                  // Reset the message ready flag since we're handling it here
		PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
        if (PolicyRxHeader.NumDataObjects == 0)                                 // If we have received a control message...
        {
            switch(PolicyRxHeader.MessageType)                                  // Determine the message type
            {
                case CMTSoftReset:
                    set_policy_state(peSourceSoftReset, __FUNCTION__, __LINE__);                            // Go to the soft reset state if we received a reset command
                    set_policy_subindex(0);                                         // Reset the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Reset the transmitter status
                    break;
                default:                                                        // If we receive any other command (including Reject & Wait), just go back to the ready state without changing
                    break;
            }
        }
    }
    else
    {
        switch (PolicySubIndex)
        {
            case 0:
                PolicySendCommand(CMTGotoMin, peSourceGotoMin, 1);                  // Send the GotoMin message
                break;
            case 1:
                PolicyStateTimer = tSnkTransition;                                  // Initialize the timer to allow for the sink to transition
                usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                increase_policy_subindex();                                                   // Increment to move to the next sub state
                break;
            case 2:
                if (!PolicyStateTimer)                                              // If the timer has expired (the sink is ready)...
                    increase_policy_subindex();                                               // Increment to move to the next sub state
                break;
            case 3:
                // Adjust the power supply if necessary...
                increase_policy_subindex();                                                   // Increment to move to the next sub state
                break;
            case 4:
                // Validate the output is ready prior to sending the ready message
                increase_policy_subindex();                                                   // Increment to move to the next sub state
                break;
            default:
                PolicySendCommand(CMTPS_RDY, peSourceReady, 0);                     // Send the PS_RDY message and move onto the Source Ready state
                break;
        }
    }
}

void PolicySourceSendDRSwap(void)
{
    UINT8 Status;
    switch (PolicySubIndex)
    {
        case 0:
            Status = PolicySendCommandNoReset(CMTDR_Swap, peSourceSendDRSwap, 1);   // Send the DR_Swap message
            if (Status == STAT_SUCCESS)                                         // If we received the good CRC message...
            {
                PolicyStateTimer = tSenderResponse;                             // Initialize for SenderResponseTimer
                usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
            }
            else if (Status == STAT_ERROR)                                      // If there was an error...
                set_policy_state(peErrorRecovery, __FUNCTION__, __LINE__);                                  // Go directly to the error recovery state
            break;
        default:
            if (ProtocolMsgRx)
            {
                ProtocolMsgRx = FALSE;                                          // Reset the message ready flag since we're handling it here
				PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
                if (PolicyRxHeader.NumDataObjects == 0)                         // If we have received a control message...
                {
                    switch(PolicyRxHeader.MessageType)                          // Determine the message type
                    {
                        case CMTAccept:
                            PolicyIsDFP = !PolicyIsDFP;                         // Flip the current data role
                            Registers.Switches.DATAROLE = PolicyIsDFP;          // Update the data role
                            FUSB300Write(regSwitches1, 1, &Registers.Switches.byte[1]); // Commit the data role in the 302 for the auto CRC
                            set_policy_state(peSourceReady, __FUNCTION__, __LINE__);                        // Source ready state
                            break;
                        case CMTSoftReset:
                            set_policy_state(peSourceSoftReset, __FUNCTION__, __LINE__);                    // Go to the soft reset state if we received a reset command
                            break;
                        default:                                                // If we receive any other command (including Reject & Wait), just go back to the ready state without changing
                            set_policy_state(peSourceReady, __FUNCTION__, __LINE__);                        // Go to the source ready state
                            break;
                    }
                }
                else                                                            // Otherwise we received a data message...
                {
                    set_policy_state(peSourceReady, __FUNCTION__, __LINE__);                                // Go to the sink ready state if we received a unexpected data message (ignoring message)
                }
                set_policy_subindex(0);                                             // Reset the sub index
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Reset the transmitter status
            }
            else if (PolicyStateTimer == 0)                                     // If the sender response timer times out...
            {
                set_policy_state(peSourceReady, __FUNCTION__, __LINE__);                                    // Go to the source ready state if the SenderResponseTimer times out
                set_policy_subindex(0);                                             // Reset the sub index
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Reset the transmitter status
            }
            break;
    }
}

void PolicySourceEvaluateDRSwap(void)
{
    UINT8 Status;
    if (PortType != USBTypeC_DRP)                                               // For ease, just determine Accept/Reject based on PortType
    {
        PolicySendCommand(CMTReject, peSourceReady, 0);                         // Send the reject if we are not a DRP
    }
    else                                                                        // If we are a DRP, follow through with the swap
    {
        Status = PolicySendCommandNoReset(CMTAccept, peSourceReady, 0);         // Send the Accept message and wait for the good CRC
        if (Status == STAT_SUCCESS)                                             // If we received the good CRC...
        {
            PolicyIsDFP = !PolicyIsDFP;                                         // We're not really doing anything except flipping the bit
            Registers.Switches.DATAROLE = PolicyIsDFP;                          // Update the data role
            FUSB300Write(regSwitches1, 1, &Registers.Switches.byte[1]);         // Commit the data role in the 302 for the auto CRC
        }
        else if (Status == STAT_ERROR)                                          // If we didn't receive the good CRC...
        {
            set_policy_state(peErrorRecovery, __FUNCTION__, __LINE__);                                      // Go to the error recovery state
            set_policy_subindex(0);                                                 // Clear the sub-index
            set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                                // Clear the transmitter status
        }
    }
}

void PolicySourceSendVCONNSwap(void)
{
    switch(PolicySubIndex)
    {
        case 0:
            if (PolicySendCommand(CMTVCONN_Swap, peSourceSendVCONNSwap, 1) == STAT_SUCCESS) // Send the VCONN_Swap message and wait for the good CRC
            {
                PolicyStateTimer = tSenderResponse;                             // Once we receive the good CRC, set the sender response timer
                usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
            }
            break;
        case 1:
            if (ProtocolMsgRx)                                                  // Have we received a message from the source?
            {
                ProtocolMsgRx = FALSE;                                          // Reset the message received flag since we're handling it here
				PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
                if (PolicyRxHeader.NumDataObjects == 0)                         // If we have received a command
                {
                    switch (PolicyRxHeader.MessageType)                         // Determine which command was received
                    {
                        case CMTAccept:                                         // If we get the Accept message...
                            increase_policy_subindex();                                   // Increment the subindex to move onto the next step
                            break;
                        case CMTWait:                                           // If we get either the reject or wait message...
                        case CMTReject:
                            set_policy_state(peSourceReady, __FUNCTION__, __LINE__);                        // Go back to the source ready state
                            set_policy_subindex(0);                                 // Clear the sub index
                            set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                // Clear the transmitter status
                            break;
                        default:                                                // For all other commands received, simply ignore them
                            break;
                    }
                }
            }
            else if (!PolicyStateTimer)                                         // If the SenderResponseTimer times out...
            {
                set_policy_state(peSourceReady, __FUNCTION__, __LINE__);                                    // Go back to the source ready state
                set_policy_subindex(0);                                             // Clear the sub index
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Clear the transmitter status
            }
            break;
        case 2:
            if (Registers.Switches.VCONN_CC1 || Registers.Switches.VCONN_CC2)   // If we are currently sourcing VCONN...
            {
                PolicyStateTimer = tVCONNSourceOn;                              // Enable the VCONNOnTimer and wait for a PS_RDY message
                usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                increase_policy_subindex();                                               // Increment the subindex to move to waiting for a PS_RDY message
            }
            else                                                                // Otherwise we need to start sourcing VCONN
            {
                if (blnCCPinIsCC1)                                              // If the CC pin is CC1...
                    Registers.Switches.VCONN_CC2 = 1;                           // Enable VCONN for CC2
                else                                                            // Otherwise the CC pin is CC2
                    Registers.Switches.VCONN_CC1 = 1;                           // so enable VCONN on CC1
                FUSB300Write(regSwitches0, 1, &Registers.Switches.byte[0]);     // Commit the register setting to the FUSB302
                PolicyStateTimer = tFPF2498Transition;                          // Allow time for the FPF2498 to enable...
                usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                set_policy_subindex(4);                                             // Skip the next state and move onto sending the PS_RDY message after the timer expires            }
            }
            break;
        case 3:
            if (ProtocolMsgRx)                                                  // Have we received a message from the source?
            {
                ProtocolMsgRx = FALSE;                                          // Reset the message received flag since we're handling it here
				PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
                if (PolicyRxHeader.NumDataObjects == 0)                         // If we have received a command
                {
                    switch (PolicyRxHeader.MessageType)                         // Determine which command was received
                    {
                        case CMTPS_RDY:                                         // If we get the PS_RDY message...
                            Registers.Switches.VCONN_CC1 = 0;                   // Disable the VCONN source
                            Registers.Switches.VCONN_CC2 = 0;                   // Disable the VCONN source
                            FUSB300Write(regSwitches0, 1, &Registers.Switches.byte[0]); // Commit the register setting to the FUSB302
                            set_policy_state(peSourceReady, __FUNCTION__, __LINE__);                        // Move onto the Sink Ready state
                            set_policy_subindex(0);                                 // Clear the sub index
                            set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                // Clear the transmitter status
                            break;
                        default:                                                // For all other commands received, simply ignore them
                            break;
                    }
                }
            }
            else if (!PolicyStateTimer)                                         // If the VCONNOnTimer times out...
            {
                set_policy_state(peSourceSendHardReset, __FUNCTION__, __LINE__);                            // Issue a hard reset
                set_policy_subindex(0);                                             // Clear the sub index
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Clear the transmitter status
            }
            break;
        default:
            if (!PolicyStateTimer)
            {
                PolicySendCommand(CMTPS_RDY, peSourceReady, 0);                 // Send the Accept message and wait for the good CRC
            }
            break;
    }
}

void PolicySourceSendPRSwap(void)
{
    UINT8 Status;
    switch(PolicySubIndex)
    {
        case 0: // Send the PRSwap command
            if (PolicySendCommand(CMTPR_Swap, peSourceSendPRSwap, 1) == STAT_SUCCESS) // Send the PR_Swap message and wait for the good CRC
            {
                PolicyStateTimer = tSenderResponse;                             // Once we receive the good CRC, set the sender response timer
                usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
            }
            break;
        case 1:  // Require Accept message to move on or go back to ready state
            if (ProtocolMsgRx)                                                  // Have we received a message from the source?
            {
                ProtocolMsgRx = FALSE;                                          // Reset the message received flag since we're handling it here
				PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
                if (PolicyRxHeader.NumDataObjects == 0)                         // If we have received a command
                {
                    switch (PolicyRxHeader.MessageType)                         // Determine which command was received
                    {
                        case CMTAccept:                                         // If we get the Accept message...
                            PRSwapTimer = tPRSwapBailout;                       // Initialize the PRSwapTimer to indicate we are in the middle of a swap
                            usbpd_start_timer(&prswap_hrtimer, PRSwapTimer);
                            PolicyStateTimer = tSnkTransition;                  // Start the sink transition timer
                            usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                            increase_policy_subindex();                                   // Increment the subindex to move onto the next step
                            break;
                        case CMTWait:                                           // If we get either the reject or wait message...
                        case CMTReject:
                            set_policy_state(peSourceReady, __FUNCTION__, __LINE__);                        // Go back to the source ready state
                            set_policy_subindex(0);                                 // Clear the sub index
                            set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                // Clear the transmitter status
                            break;
                        default:                                                // For all other commands received, simply ignore them
                            break;
                    }
                }
            }
            else if (!PolicyStateTimer)                                         // If the SenderResponseTimer times out...
            {
                set_policy_state(peSourceReady, __FUNCTION__, __LINE__);                                    // Go back to the source ready state
                set_policy_subindex(0);                                             // Clear the sub index
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Clear the transmitter status
            }
            break;
        case 2: // Wait for tSnkTransition and then turn off power (and Rd on/Rp off)
            if (!PolicyStateTimer)
            {
                RoleSwapToAttachedSink();                                        // Initiate the Type-C state machine for a power role swap
                PolicyStateTimer = tPSSourceOffNom;                               // Allow the output voltage to fall before sending the PS_RDY message
                usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                increase_policy_subindex();                                               // Increment the sub-index to move onto the next state
            }
            break;
        case 3: // Allow time for the supply to fall and then send the PS_RDY message
            if (!PolicyStateTimer)
            {
                Status = PolicySendCommandNoReset(CMTPS_RDY, peSourceSendPRSwap, 4);
                if (Status == STAT_SUCCESS)                                     // If we successfully sent the PS_RDY command and received the goodCRC
                {
                    PolicyStateTimer = tPSSourceOnMax;                             // Start the PSSourceOn timer to allow time for the new supply to come up
                    usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                }
                else if (Status == STAT_ERROR)
                    set_policy_state(peErrorRecovery, __FUNCTION__, __LINE__);                              // If we get an error, go to the error recovery state
            }
            break;
        default: // Wait to receive a PS_RDY message from the new DFP
            if (ProtocolMsgRx)                                                  // Have we received a message from the source?
            {
                ProtocolMsgRx = FALSE;                                          // Reset the message received flag since we're handling it here
				PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
                if (PolicyRxHeader.NumDataObjects == 0)                         // If we have received a command
                {
                    switch (PolicyRxHeader.MessageType)                         // Determine which command was received
                    {
                        case CMTPS_RDY:                                         // If we get the PS_RDY message...
                            set_policy_state(peSinkStartup, __FUNCTION__, __LINE__);                        // Go to the sink startup state
                            set_policy_subindex(0);                                 // Clear the sub index
                            PolicyStateTimer = 0;                               // Clear the policy state timer
                            break;
                        default:                                                // For all other commands received, simply ignore them
                            break;
                    }
                }
            }
            else if (!PolicyStateTimer)                                         // If the PSSourceOn times out...
            {
                set_policy_state(peErrorRecovery, __FUNCTION__, __LINE__);                                  // Go to the error recovery state
                set_policy_subindex(0);                                             // Clear the sub index
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Clear the transmitter status
            }
            break;
    }
}

void PolicySourceEvaluatePRSwap(void)
{
    UINT8 Status;
    switch(PolicySubIndex)
    {
        case 0: // Send either the Accept or Reject command
            if (PortType != USBTypeC_DRP)                                       // For ease, just determine Accept/Reject based on PortType
            {
                PolicySendCommand(CMTReject, peSourceReady, 0);                 // Send the reject if we are not a DRP
            }
            else
            {
                if (PolicySendCommand(CMTAccept, peSourceEvaluatePRSwap, 1) == STAT_SUCCESS) // Send the Accept message and wait for the good CRC
                {
                    PRSwapTimer = tPRSwapBailout;                               // Initialize the PRSwapTimer to indicate we are in the middle of a swap
		            usbpd_start_timer(&prswap_hrtimer, PRSwapTimer);
                    RoleSwapToAttachedSink();                                    // If we have received the good CRC... initiate the Type-C state machine for a power role swap
                    PolicyStateTimer = tPSSourceOffNom;                           // Allow the output voltage to fall before sending the PS_RDY message
                    usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                }
            }
            break;
        case 1: // Allow time for the supply to fall and then send the PS_RDY message
            if (!PolicyStateTimer)
            {
                Status = PolicySendCommandNoReset(CMTPS_RDY, peSourceEvaluatePRSwap, 2);    // Send the PS_RDY message
                if (Status == STAT_SUCCESS)                                     // If we successfully sent the PS_RDY command and received the goodCRC
                {
                    PolicyStateTimer = tPSSourceOnMax;                             // Start the PSSourceOn timer to allow time for the new supply to come up
                    usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                }
                else if (Status == STAT_ERROR)
                    set_policy_state(peErrorRecovery, __FUNCTION__, __LINE__);                              // If we get an error, go to the error recovery state
            }
            break;
        default: // Wait to receive a PS_RDY message from the new DFP
            if (ProtocolMsgRx)                                                  // Have we received a message from the source?
            {
                ProtocolMsgRx = FALSE;                                          // Reset the message received flag since we're handling it here
				PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
                if (PolicyRxHeader.NumDataObjects == 0)                         // If we have received a command
                {
                    switch (PolicyRxHeader.MessageType)                         // Determine which command was received
                    {
                        case CMTPS_RDY:                                         // If we get the PS_RDY message...
                            set_policy_state(peSinkStartup, __FUNCTION__, __LINE__);                        // Go to the sink startup state
                            set_policy_subindex(0);                                 // Clear the sub index
                            PolicyStateTimer = 0;                               // Clear the policy state timer
                            break;
                        default:                                                // For all other commands received, simply ignore them
                            break;
                    }
                }
            }
            else if (!PolicyStateTimer)                                         // If the PSSourceOn times out...
            {
                set_policy_state(peErrorRecovery, __FUNCTION__, __LINE__);                                  // Go to the error recovery state
                set_policy_subindex(0);                                             // Clear the sub index
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Clear the transmitter status
            }
            break;
    }
}

// ############################## Sink States  ############################## //

void PolicySinkSendHardReset(void)
{
    PolicySendHardReset(peSinkTransitionDefault, 0);
}

void PolicySinkSoftReset(void)
{
    if (PolicySendCommand(CMTAccept, peSinkWaitCaps, 0) == STAT_SUCCESS)
    {
        PolicyStateTimer = tSinkWaitCap;
        usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
    }
}

void PolicySinkSendSoftReset(void)
{
    switch (PolicySubIndex)
    {
        case 0:
            if (PolicySendCommand(CMTSoftReset, peSinkSendSoftReset, 1) == STAT_SUCCESS)    // Send the soft reset command to the protocol layer
            {
                PolicyStateTimer = tSenderResponse;                             // Start the sender response timer to wait for an accept message once successfully sent
                usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
            }
            break;
        default:
            if (ProtocolMsgRx)                                                  // If we have received a message
            {
                ProtocolMsgRx = FALSE;                                          // Reset the message received flag since we've handled it here
				PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
                if ((PolicyRxHeader.NumDataObjects == 0) && (PolicyRxHeader.MessageType == CMTAccept))  // And it was the Accept...
                {
                    set_policy_state(peSinkWaitCaps, __FUNCTION__, __LINE__);                               // Go to the wait for capabilities state
                    PolicyStateTimer = tSinkWaitCap;                            // Set the state timer to tSinkWaitCap
                    usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                }
                else                                                            // Otherwise it was a message that we didn't expect, so...
                    set_policy_state(peSinkSendHardReset, __FUNCTION__, __LINE__);                          // Go to the hard reset state
                set_policy_subindex(0);                                             // Reset the sub index
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Clear the transmitter status
            }
            else if (!PolicyStateTimer)                                         // If we didn't get a response to our request before timing out...
            {
                set_policy_state(peSinkSendHardReset, __FUNCTION__, __LINE__);                              // Go to the hard reset state
                set_policy_subindex(0);                                             // Reset the sub index
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Clear the transmitter status
            }
            break;
    }
}

void PolicySinkTransitionDefault(void)
{
    USBPDContract.object = 0;                                                   // Clear the USB PD contract (output power to 5V default)
    NoResponseTimer = tNoResponse;                                              // Initialize the no response timer
    usbpd_start_timer(&noresponse_hrtimer, NoResponseTimer);
    PRSwapTimer = tNoResponse;                                                  // Set the swap timer to not detach upon VBUS going away
    usbpd_start_timer(&prswap_hrtimer, PRSwapTimer);
    // Adjust sink power if necessary and start timer prior to going to startup state?
    set_policy_state(peSinkStartup, __FUNCTION__, __LINE__);                                                // Go to the startup state
    set_policy_subindex(0);                                                         // Clear the sub index
    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                                        // Reset the transmitter status
    ResetProtocolLayer(TRUE);
}

void PolicySinkStartup(void)
{
    PolicyIsSource = FALSE;                                                     // Clear the flag to indicate that we are a sink (for PRSwaps)
    ResetProtocolLayer(TRUE);                                                   // Reset the protocol layer
    CapsCounter = 0;                                                            // Clear the caps counter
    CollisionCounter = 0;                                                       // Reset the collision counter
    PolicyStateTimer = 0;                                                       // Reset the policy state timer
    set_policy_state(peSinkDiscovery, __FUNCTION__, __LINE__);                                              // Go to the sink discovery state
    set_policy_subindex(0);                                                         // Reset the sub index
}

void PolicySinkDiscovery(void)
{
    if (Registers.Status.VBUSOK)
    {
		PD_LOG("%s, %d, vbusok\n", __FUNCTION__, __LINE__);
        PRSwapTimer = 0;                                                        // Clear the swap timer
        set_policy_state(peSinkWaitCaps, __FUNCTION__, __LINE__);
        set_policy_subindex(0);
        PolicyStateTimer = tSinkWaitCap;
        //usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
    }
    else if (NoResponseTimer == 0)
    {
		PD_LOG("******* NoResponseTimer = 0 ****** %s\n", __FUNCTION__);
        PRSwapTimer = 0;                                                        // Clear the swap timer
        set_policy_state(peErrorRecovery, __FUNCTION__, __LINE__);
        set_policy_subindex(0);
    }
	else
	{
		PD_LOG("%s, %d Do nothing\n", __FUNCTION__, __LINE__);
	}
}

void PolicySinkWaitCaps(void)
{
    if (ProtocolMsgRx) // If we have received a message...
    {
		PD_LOG("Received a message, %s, %d\n", __FUNCTION__, __LINE__);
		PD_LOG("NumDataObjects = %d, MessageType = %x\n", PolicyRxHeader.NumDataObjects, PolicyRxHeader.MessageType);
		//PD_LOG("NumDataObjects = %d, MessageType = %s\n", PolicyRxHeader.NumDataObjects, message_type[PolicyRxHeader.MessageType]);
        ProtocolMsgRx = FALSE; // Reset the message ready flag since we're handling it here
		PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);

		// Have we received a valid source cap messagei?
        if ((PolicyRxHeader.NumDataObjects > 0) && (PolicyRxHeader.MessageType == DMTSourceCapabilities))
        {
			PD_LOG("received a valid source cap message\n");
            //PD_LOG("policySinkWaitCaps: ----------------------\n");
            UpdateCapabilitiesRx(TRUE); // Update the received capabilities
            set_policy_state(peSinkEvaluateCaps, __FUNCTION__, __LINE__); // Set the evaluate source capabilities state
        }
        else if ((PolicyRxHeader.NumDataObjects == 0) && (PolicyRxHeader.MessageType == CMTSoftReset))
        {
			PD_LOG("Go to the soft reset state\n");
            set_policy_state(peSinkSoftReset, __FUNCTION__, __LINE__); // Go to the soft reset state
        }
		else
		{
			PD_LOG("Do nothing, %s, %d\n", __FUNCTION__, __LINE__);
		}
        set_policy_subindex(0); // Reset the sub index
    }
    else if ((NoResponseTimer == 0) && (HardResetCounter > nHardResetCount))
    {
		PD_LOG("(NoResponseTimer == 0) && (HardResetCounter > nHardResetCount)\n");
        set_policy_state(peErrorRecovery, __FUNCTION__, __LINE__);
        set_policy_subindex(0);
    }
    else if (PolicyStateTimer == 0)
    {
		PD_LOG("PolicyStateTimer == 0\n");
        set_policy_state(peSinkSendHardReset, __FUNCTION__, __LINE__);
        set_policy_subindex(0);
    }
	else
	{
		PD_LOG("%s, %d, Do nothing\n", __FUNCTION__, __LINE__);
	}
}

void PolicySinkEvaluateCaps(void)
{
    // Due to latency with the PC and evaluating capabilities, we are always going to select the first one by default (5V default)
    // This will allow the software time to determine if they want to select one of the other capabilities (user selectable)
    // If we want to automatically show the selection of a different capabilities message, we need to build in the functionality here
    // The evaluate caps
    int i, reqPos;
    unsigned int objVoltage, objCurrent, objPower, MaxPower, SelVoltage, ReqCurrent;
    NoResponseTimer = USHRT_MAX;                                                // Stop the no response timer
    HardResetCounter = 0;                                                       // Reset the hard reset counter
    SelVoltage = 0;
    MaxPower = 0;
    reqPos = 0;                                                                 // Select nothing in case there is an error...
	PD_LOG("NumDataObjects = %d\n", CapsHeaderReceived.NumDataObjects);
    for (i=0; i<CapsHeaderReceived.NumDataObjects; i++)                         // Going to select the highest power object that we are compatible with
    {
		PD_LOG("NDO = %d, %s\n", i, PD_OBJ_TYPE_NAME[CapsReceived[i].PDO.SupplyType]);
        switch (CapsReceived[i].PDO.SupplyType)
        {
            case pdoTypeFixed:
                objVoltage = CapsReceived[i].FPDOSupply.Voltage;                // Get the output voltage of the fixed supply
				PD_LOG("objVoltag = %dV, SinkRequestMaxVoltage = %d, %s, %d\n", objVoltage / 20, SinkRequestMaxVoltage / 20, __func__, __LINE__);
                if (objVoltage > SinkRequestMaxVoltage)                         // If the voltage is greater than our limit...
                    objPower = 0;                                               // Set the power to zero to ignore the object
                else                                                            // Otherwise...
                {
                    objCurrent = CapsReceived[i].FPDOSupply.MaxCurrent;
                    objPower = objVoltage * objCurrent;                         // Calculate the power for comparison
                }
                //PD_LOG("objVoltage=%d\n, objCurrent=%d, objPower=%d, i=%d\n",
                //    objVoltage, objCurrent, objPower, i);
                break;
            case pdoTypeVariable:
				printk("%s, %d\n", __func__, __LINE__);
                objVoltage = CapsReceived[i].VPDO.MaxVoltage;                   // Grab the maximum voltage of the variable supply
                if (objVoltage > SinkRequestMaxVoltage)                         // If the max voltage is greater than our limit...
                    objPower = 0;                                               // Set the power to zero to ignore the object
                else                                                            // Otherwise...
                {
                    objVoltage = CapsReceived[i].VPDO.MinVoltage;               // Get the minimum output voltage of the variable supply
                    objCurrent = CapsReceived[i].VPDO.MaxCurrent;               // Get the maximum output current of the variable supply
                    objPower = objVoltage * objCurrent;                         // Calculate the power for comparison (based on min V/max I)
                }
                break;
            case pdoTypeBattery:                                                // We are going to ignore battery powered sources for now
            default:                                                            // We are also ignoring undefined supply types
                objPower = 0;                                                   // Set the object power to zero so we ignore for now
                break;
        }
        if (objPower > MaxPower)                                                // If the current object has power greater than the previous objects
        {
			PD_LOG("%s, %d\n", __func__, __LINE__);
            MaxPower = objPower;                                                // Store the objects power
            SelVoltage = objVoltage;                                            // Store the objects voltage (used for calculations)
            reqPos = i + 1;                                                     // Store the position of the object
        }
    }
    PD_LOG("reqPos=%d, selVoltage=%d V\n", reqPos, (SelVoltage / 20)); /* 50mv 一个单位 */
    if ((reqPos > 0) && (SelVoltage > 0))
    {
		PD_LOG("%s, %d\n", __func__, __LINE__);
        SinkRequest.FVRDO.ObjectPosition = reqPos & 0x07;                       // Set the object position selected
        SinkRequest.FVRDO.GiveBack = SinkGotoMinCompatible;                     // Set whether we will respond to the GotoMin message
        SinkRequest.FVRDO.NoUSBSuspend = SinkUSBSuspendOperation;               // Set whether we want to continue pulling power during USB Suspend
        SinkRequest.FVRDO.USBCommCapable = SinkUSBCommCapable;                  // Set whether USB communications is active
        ReqCurrent = SinkRequestOpPower / SelVoltage;                           // Calculate the requested operating current
        SinkRequest.FVRDO.OpCurrent = (ReqCurrent & 0x3FF);                     // Set the current based on the selected voltage (in 10mA units)
        ReqCurrent = SinkRequestMaxPower / SelVoltage;                          // Calculate the requested maximum current
        SinkRequest.FVRDO.MinMaxCurrent = (ReqCurrent & 0x3FF);                 // Set the min/max current based on the selected voltage (in 10mA units)
        if (SinkGotoMinCompatible)                                              // If the give back flag is set...
		{
			PD_LOG("sink goto min compatible\n");
            SinkRequest.FVRDO.CapabilityMismatch = FALSE;                       // There can't be a capabilities mismatch
		}
        else                                                                    // Otherwise...
        {
			PD_LOG("%s, %d\n", __func__, __LINE__);
            if (MaxPower < SinkRequestMaxPower)                                 // If the max power available is less than the max power requested...
			{
				PD_LOG("MaxPower is less then the SinkRequestMaxPower\n");
                SinkRequest.FVRDO.CapabilityMismatch = TRUE;                    // flag the source that we need more power
			}
            else                                                                // Otherwise...
			{
				PD_LOG("there is no mismatch in the capabilities\n");
                SinkRequest.FVRDO.CapabilityMismatch = FALSE;                   // there is no mismatch in the capabilities
			}
        }
		PD_LOG("%s, %d\n", __func__, __LINE__);
        set_policy_state(peSinkSelectCapability, __FUNCTION__, __LINE__);                                   // Go to the select capability state
        set_policy_subindex(0);                                                     // Reset the sub index
        PolicyStateTimer = tSenderResponse;                                     // Initialize the sender response timer
        usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
    }
    else
    {
		printk("%s, %d\n", __func__, __LINE__);
        // For now, we are just going to go back to the wait state instead of sending a reject or reset (may change in future)
        set_policy_state(peSinkWaitCaps, __FUNCTION__, __LINE__);                                           // Go to the wait for capabilities state
        PolicyStateTimer = tSinkWaitCap;                                        // Set the state timer to tSinkWaitCap
        usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
    }
}

void PolicySinkSelectCapability(void)
{
	unsigned char oldPolicySunIndex;
	PD_LOG("%s: PolicySubIndex = %d, ProtocolMsgRx = %s, NDO = %d\n", __FUNCTION__, PolicySubIndex, ProtocolMsgRx == 1 ? "TRUE" : "FALSE", PolicyRxHeader.NumDataObjects);

	do {
		oldPolicySunIndex = PolicySubIndex;
		switch (PolicySubIndex)
		{
			case 0:
				if (PolicySendData(DMTRequest, 1, &SinkRequest, peSinkSelectCapability, 1) == STAT_SUCCESS)
				{
					PolicyStateTimer = tSenderResponse;
					usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
				}

				/* Debug sink request */
				PD_LOG("SinkRequest MinMaxCurrent = %dmA\n", SinkRequest.FVRDO.MinMaxCurrent * 10);
				PD_LOG("SinkRequest OpCurrent = %dmA\n", SinkRequest.FVRDO.OpCurrent * 10);
				break;
			default:
				if (ProtocolMsgRx)
				{
					ProtocolMsgRx = FALSE;                                          // Reset the message ready flag since we're handling it here
					PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
					if (PolicyRxHeader.NumDataObjects == 0)                         // If we have received a control message...
					{
						PD_LOG("%s, %s, %d\n", CM_message_type[PolicyRxHeader.MessageType], __FUNCTION__, __LINE__);
						switch(PolicyRxHeader.MessageType)                          // Determine the message type
						{
							case CMTAccept:
								PolicyHasContract = TRUE;                           // Set the flag to indicate that a contract is in place
								USBPDContract.object = SinkRequest.object;          // Set the actual contract that is in place
								PolicyStateTimer = tPSTransition;                   // Set the transition timer here
								usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
								set_policy_state(peSinkTransitionSink, __FUNCTION__, __LINE__);                 // Go to the transition state if the source accepted the request
								break;
							case CMTWait:
							case CMTReject:
								set_policy_state(peSinkReady, __FUNCTION__, __LINE__);                          // Go to the sink ready state if the source rejects or has us wait
								break;
							case CMTSoftReset:
								set_policy_state(peSinkSoftReset, __FUNCTION__, __LINE__);                      // Go to the soft reset state if we received a reset command
								break;
							default:
								PD_LOG("We are going to send a reset message for all other commands received\n");
								set_policy_state(peSinkSendSoftReset, __FUNCTION__, __LINE__);                  // We are going to send a reset message for all other commands received
								break;
						}
					}
					else                                                            // Otherwise we received a data message...
					{
						PD_LOG("%s, %s, %d\n", DMT_message_type[PolicyRxHeader.MessageType], __FUNCTION__, __LINE__);
						switch (PolicyRxHeader.MessageType)
						{
							case DMTSourceCapabilities:                             // If we received a new source capabilities message
								UpdateCapabilitiesRx(TRUE);                         // Update the received capabilities
								set_policy_state(peSinkEvaluateCaps, __FUNCTION__, __LINE__);                   // Go to the evaluate caps state
								break;
							default:
								set_policy_state(peSinkSendSoftReset, __FUNCTION__, __LINE__);                  // Send a soft reset to get back to a known state
								break;
						}
					}
					set_policy_subindex(0);                                             // Reset the sub index
					set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Reset the transmitter status
				}
				else if (PolicyStateTimer == 0)                                     // If the sender response timer times out...
				{
					set_policy_state(peSinkSendHardReset, __FUNCTION__, __LINE__);                              // Go to the hard reset state
					set_policy_subindex(0);                                             // Reset the sub index
					set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Reset the transmitter status
				}
				break;
		}
	} while ((PolicyState == peSinkSelectCapability) && (oldPolicySunIndex < PolicySubIndex));
}

void PolicySinkTransitionSink(void)
{
	PD_LOG("%s, %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __FUNCTION__, __LINE__);
    if (ProtocolMsgRx)
    {
        ProtocolMsgRx = FALSE;                                                  // Reset the message ready flag since we're handling it here
		PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
		PD_LOG("NumDataObject = %d\n", PolicyRxHeader.NumDataObjects);
        if (PolicyRxHeader.NumDataObjects == 0)                                 // If we have received a control message...
        {
			PD_LOG("%s, %s, %d\n", CM_message_type[PolicyRxHeader.MessageType], __FUNCTION__, __LINE__);
            switch(PolicyRxHeader.MessageType)                                  // Determine the message type
            {
                case CMTPS_RDY:
                    set_policy_state(peSinkReady, __FUNCTION__, __LINE__);                                  // Go to the ready state
                    break;
                case CMTSoftReset:
                    set_policy_state(peSinkSoftReset, __FUNCTION__, __LINE__);                              // Go to the soft reset state if we received a reset command
                    break;
                default:
//                    set_policy_state(peSinkSendSoftReset, __FUNCTION__, __LINE__);                          // We are going to send a reset message for all other commands received
                    break;
            }
        }
        else                                                                    // Otherwise we received a data message...
        {
			PD_LOG("%s\n", DMT_message_type[PolicyRxHeader.MessageType]);
            switch (PolicyRxHeader.MessageType)                                 // Determine the message type
            {
                case DMTSourceCapabilities:                                     // If we received new source capabilities...
                    UpdateCapabilitiesRx(TRUE);                                 // Update the source capabilities
                    set_policy_state(peSinkEvaluateCaps, __FUNCTION__, __LINE__);                           // And go to the evaluate capabilities state
                    break;
                default:                                                        // If we receieved an unexpected data message...
                    set_policy_state(peSinkSendSoftReset, __FUNCTION__, __LINE__);                          // Send a soft reset to get back to a known state
                    break;
            }
        }
        set_policy_subindex(0);                                                     // Reset the sub index
        set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                                    // Reset the transmitter status
    }
    else if (PolicyStateTimer == 0)                                             // If the PSTransitionTimer times out...
    {
		PD_LOG("policy state timer\n");
        set_policy_state(peSinkSendHardReset, __FUNCTION__, __LINE__);                                      // Issue a hard reset
        set_policy_subindex(0);                                                     // Reset the sub index
        set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                                    // Reset the transmitter status
    }
}

void PolicySinkReady(void)
{
    if (ProtocolMsgRx)                                                          // Have we received a message from the source?
    {
        ProtocolMsgRx = FALSE;                                                  // Reset the message received flag since we're handling it here
		PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
        if (PolicyRxHeader.NumDataObjects == 0)                                 // If we have received a command
        {
            switch (PolicyRxHeader.MessageType)                                 // Determine which command was received
            {
                case CMTGotoMin:
                    set_policy_state(peSinkTransitionSink, __FUNCTION__, __LINE__);                         // Go to transitioning the sink power
                    PolicyStateTimer = tPSTransition;                           // Set the transition timer here
                    usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                case CMTGetSinkCap:
                    set_policy_state(peSinkGiveSinkCap, __FUNCTION__, __LINE__);                            // Go to sending the sink caps state
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                case CMTGetSourceCap:
                    set_policy_state(peSinkGiveSourceCap, __FUNCTION__, __LINE__);                          // Go to sending the source caps if we are dual-role
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                case CMTDR_Swap:                                                // If we get a DR_Swap message...
                    set_policy_state(peSinkEvaluateDRSwap, __FUNCTION__, __LINE__);                         // Go evaluate whether we are going to accept or reject the swap
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                case CMTPR_Swap:
                    set_policy_state(peSinkEvaluatePRSwap, __FUNCTION__, __LINE__);                         // Go evaluate whether we are going to accept or reject the swap
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                case CMTVCONN_Swap:                                             // If we get a VCONN_Swap message...
                    set_policy_state(peSinkEvaluateVCONNSwap, __FUNCTION__, __LINE__);                      // Go evaluate whether we are going to accept or reject the swap
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                case CMTSoftReset:
                    set_policy_state(peSinkSoftReset, __FUNCTION__, __LINE__);                              // Go to the soft reset state
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                default:                                                        // For all other commands received, simply ignore them
                    break;
            }
        }
        else
        {
            switch (PolicyRxHeader.MessageType)
            {
                case DMTSourceCapabilities:
                    UpdateCapabilitiesRx(TRUE);                                 // Update the received capabilities
                    set_policy_state(peSinkEvaluateCaps, __FUNCTION__, __LINE__);                           // Go to the evaluate capabilities state
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                case DMTVenderDefined:
                    convertAndProcessVdmMessage();
                    break;
                default:                                                        // If we get something we are not expecting... simply ignore them
                    break;
            }
        }
        set_policy_subindex(0);                                                     // Reset the sub index
        set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                                    // Reset the transmitter status
    }
    else if (USBPDTxFlag)                                                       // Has the device policy manager requested us to send a message?
    {
        if (PDTransmitHeader.NumDataObjects == 0)
        {
            switch (PDTransmitHeader.MessageType)
            {
                case CMTGetSourceCap:
                    set_policy_state(peSinkGetSourceCap, __FUNCTION__, __LINE__);                           // Go to retrieve the source caps state
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                   break;
                case CMTGetSinkCap:
                    set_policy_state(peSinkGetSinkCap, __FUNCTION__, __LINE__);                             // Go to retrieve the sink caps state
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                case CMTDR_Swap:
                    if (PortType == USBTypeC_DRP)                               // Only send if we are configured as a DRP
                    {
                        set_policy_state(peSinkSendDRSwap, __FUNCTION__, __LINE__);                         // Issue a DR_Swap message
                        set_policy_subindex(0);                                     // Clear the sub index
                        set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                    // Clear the transmitter status
                    }
                    break;
                case CMTPR_Swap:
                    if (PortType == USBTypeC_DRP)                               // Only send if we are configured as a DRP
                    {
                        set_policy_state(peSinkSendPRSwap, __FUNCTION__, __LINE__);                         // Issue a PR_Swap message
                        set_policy_subindex(0);                                     // Clear the sub index
                        set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                    // Clear the transmitter status
                    }
                    break;
                case CMTSoftReset:
                    set_policy_state(peSinkSendSoftReset, __FUNCTION__, __LINE__);                          // Go to the send soft reset state
                    set_policy_subindex(0);                                         // Clear the sub index
                    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                        // Clear the transmitter status
                    break;
                default:
                    break;
            }
        }
        else
        {
            switch (PDTransmitHeader.MessageType)
            {
                case DMTRequest:
                    SinkRequest.object = PDTransmitObjects[0].object;           // Set the actual object to request
                    set_policy_state(peSinkSelectCapability, __FUNCTION__, __LINE__);                       // Go to the select capability state
                    set_policy_subindex(0);                                         // Reset the sub index
                    PolicyStateTimer = tSenderResponse;                         // Initialize the sender response timer
                    usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                    break;
                case DMTVenderDefined:
                    // not worrying about transitioning states right now - TODO
                    set_policy_subindex(0); // Do I need this here? - Gabe
                    doVdmCommand();
                    break;
                default:
                    break;
            }
        }
    }
}

void PolicySinkGiveSinkCap(void)
{
    PolicySendData(DMTSinkCapabilities, CapsHeaderSink.NumDataObjects, &CapsSink[0], peSinkReady, 0);
}

void PolicySinkGetSinkCap(void)
{
    PolicySendCommand(CMTGetSinkCap, peSinkReady, 0);
}

void PolicySinkGiveSourceCap(void)
{
    if (PortType == USBTypeC_DRP)
        PolicySendData(DMTSourceCapabilities, CapsHeaderSource.NumDataObjects, &CapsSource[0], peSinkReady, 0);
    else
        PolicySendCommand(CMTReject, peSinkReady, 0);                           // Send the reject message and continue onto the ready state
}

void PolicySinkGetSourceCap(void)
{
    PolicySendCommand(CMTGetSourceCap, peSinkReady, 0);
}

void PolicySinkSendDRSwap(void)
{
    UINT8 Status;
    switch (PolicySubIndex)
    {
        case 0:
            Status = PolicySendCommandNoReset(CMTDR_Swap, peSinkSendDRSwap, 1); // Send the DR_Swap command
            if (Status == STAT_SUCCESS)                                         // If we received a good CRC message...
            {
                PolicyStateTimer = tSenderResponse;                             // Initialize for SenderResponseTimer if we received the GoodCRC
                usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
            }
            else if (Status == STAT_ERROR)                                      // If there was an error...
                set_policy_state(peErrorRecovery, __FUNCTION__, __LINE__);                                  // Go directly to the error recovery state
            break;
        default:
            if (ProtocolMsgRx)
            {
                ProtocolMsgRx = FALSE;                                          // Reset the message ready flag since we're handling it here
				PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
                if (PolicyRxHeader.NumDataObjects == 0)                         // If we have received a control message...
                {
                    switch(PolicyRxHeader.MessageType)                          // Determine the message type
                    {
                        case CMTAccept:
                            PolicyIsDFP = !PolicyIsDFP;                         // Flip the current data role
                            Registers.Switches.DATAROLE = PolicyIsDFP;          // Update the data role
                            FUSB300Write(regSwitches1, 1, &Registers.Switches.byte[1]); // Commit the data role in the 302 for the auto CRC
                            set_policy_state(peSinkReady, __FUNCTION__, __LINE__);                          // Sink ready state
                            break;
                        case CMTSoftReset:
                            set_policy_state(peSinkSoftReset, __FUNCTION__, __LINE__);                      // Go to the soft reset state if we received a reset command
                            break;
                        default:                                                // If we receive any other command (including Reject & Wait), just go back to the ready state without changing
                            set_policy_state(peSinkReady, __FUNCTION__, __LINE__);                          // Go to the sink ready state
                            break;
                    }
                }
                else                                                            // Otherwise we received a data message...
                {
                    set_policy_state(peSinkReady, __FUNCTION__, __LINE__);                                  // Go to the sink ready state if we received a unexpected data message (ignoring message)
                }
                set_policy_subindex(0);                                             // Reset the sub index
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Reset the transmitter status
            }
            else if (PolicyStateTimer == 0)                                     // If the sender response timer times out...
            {
                set_policy_state(peSinkReady, __FUNCTION__, __LINE__);                                      // Go to the sink ready state if the SenderResponseTimer times out
                set_policy_subindex(0);                                             // Reset the sub index
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Reset the transmitter status
            }
            break;
    }
}

void PolicySinkEvaluateDRSwap(void)
{
    UINT8 Status;
    if (PortType != USBTypeC_DRP)                                               // For ease, just determine Accept/Reject based on PortType
    {
        PolicySendCommand(CMTReject, peSinkReady, 0);                           // Send the reject if we are not a DRP
    }
    else                                                                        // If we are a DRP, follow through with the swap
    {
        Status = PolicySendCommandNoReset(CMTAccept, peSinkReady, 0);           // Send the Accept message and wait for the good CRC
        if (Status == STAT_SUCCESS)                                             // If we received the good CRC...
        {
            PolicyIsDFP = !PolicyIsDFP;                                         // We're not really doing anything except flipping the bit
            Registers.Switches.DATAROLE = PolicyIsDFP;                          // Update the data role
            FUSB300Write(regSwitches1, 1, &Registers.Switches.byte[1]);         // Commit the data role in the 302 for the auto CRC
        }
        else if (Status == STAT_ERROR)                                          // If we didn't receive the good CRC...
        {
            set_policy_state(peErrorRecovery, __FUNCTION__, __LINE__);                                      // Go to the error recovery state
            set_policy_subindex(0);                                                 // Clear the sub-index
            set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                                // Clear the transmitter status
        }
    }
}

void PolicySinkEvaluateVCONNSwap(void)
{
    switch(PolicySubIndex)
    {
        case 0:
            PolicySendCommand(CMTAccept, peSinkEvaluateVCONNSwap, 1);           // Send the Accept message and wait for the good CRC
            break;
        case 1:
            if (Registers.Switches.VCONN_CC1 || Registers.Switches.VCONN_CC2)   // If we are currently sourcing VCONN...
            {
                PolicyStateTimer = tVCONNSourceOn;                              // Enable the VCONNOnTimer and wait for a PS_RDY message
                usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                increase_policy_subindex();                                               // Increment the subindex to move to waiting for a PS_RDY message
            }
            else                                                                // Otherwise we need to start sourcing VCONN
            {
                if (blnCCPinIsCC1)                                              // If the CC pin is CC1...
                {
                    Registers.Switches.VCONN_CC2 = 1;                           // Enable VCONN for CC2
                    Registers.Switches.PDWN2 = 0;                               // Disable the pull-down on CC2 to avoid sinking unnecessary current
                }
                else                                                            // Otherwise the CC pin is CC2
                {
                    Registers.Switches.VCONN_CC1 = 1;                           // Enable VCONN for CC1
                    Registers.Switches.PDWN1 = 0;                               // Disable the pull-down on CC1 to avoid sinking unnecessary current
                }
                FUSB300Write(regSwitches0, 1, &Registers.Switches.byte[0]);     // Commit the register setting to the FUSB302
                PolicyStateTimer = tFPF2498Transition;                          // Allow time for the FPF2498 to enable...
                usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                set_policy_subindex(3);                                             // Skip the next state and move onto sending the PS_RDY message after the timer expires            }
            }
            break;
        case 2:
            if (ProtocolMsgRx)                                                  // Have we received a message from the source?
            {
                ProtocolMsgRx = FALSE;                                          // Reset the message received flag since we're handling it here
				PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
                if (PolicyRxHeader.NumDataObjects == 0)                         // If we have received a command
                {
                    switch (PolicyRxHeader.MessageType)                         // Determine which command was received
                    {
                        case CMTPS_RDY:                                         // If we get the PS_RDY message...
                            Registers.Switches.VCONN_CC1 = 0;                   // Disable the VCONN source
                            Registers.Switches.VCONN_CC2 = 0;                   // Disable the VCONN source
                            Registers.Switches.PDWN1 = 1;                       // Ensure the pull-down on CC1 is enabled
                            Registers.Switches.PDWN2 = 1;                       // Ensure the pull-down on CC2 is enabled
                            FUSB300Write(regSwitches0, 1, &Registers.Switches.byte[0]); // Commit the register setting to the FUSB302
                            set_policy_state(peSinkReady, __FUNCTION__, __LINE__);                          // Move onto the Sink Ready state
                            set_policy_subindex(0);                                 // Clear the sub index
                            set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                // Clear the transmitter status
                            break;
                        default:                                                // For all other commands received, simply ignore them
                            break;
                    }
                }
            }
            else if (!PolicyStateTimer)                                         // If the VCONNOnTimer times out...
            {
                set_policy_state(peSourceSendHardReset, __FUNCTION__, __LINE__);                            // Issue a hard reset
                set_policy_subindex(0);                                             // Clear the sub index
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Clear the transmitter status
            }
            break;
        default:
            if (!PolicyStateTimer)
            {
                PolicySendCommand(CMTPS_RDY, peSinkReady, 0);                       // Send the Accept message and wait for the good CRC
            }
            break;
    }
}

void PolicySinkSendPRSwap(void)
{
    UINT8 Status;
    switch(PolicySubIndex)
    {
        case 0: // Send the PRSwap command
            if (PolicySendCommand(CMTPR_Swap, peSinkSendPRSwap, 1) == STAT_SUCCESS) // Send the PR_Swap message and wait for the good CRC
            {
                PolicyStateTimer = tSenderResponse;                             // Once we receive the good CRC, set the sender response timer
                usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
            }
            break;
        case 1:  // Require Accept message to move on or go back to ready state
            if (ProtocolMsgRx)                                                  // Have we received a message from the source?
            {
                ProtocolMsgRx = FALSE;                                          // Reset the message received flag since we're handling it here
				PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
                if (PolicyRxHeader.NumDataObjects == 0)                         // If we have received a command
                {
                    switch (PolicyRxHeader.MessageType)                         // Determine which command was received
                    {
                        case CMTAccept:                                         // If we get the Accept message...
                            PRSwapTimer = tPRSwapBailout;                       // Initialize the PRSwapTimer to indicate we are in the middle of a swap
                            usbpd_start_timer(&prswap_hrtimer, PRSwapTimer);
                            PolicyStateTimer = tPSSourceOffMax;                    // Start the sink transition timer
                            usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                            increase_policy_subindex();                                   // Increment the subindex to move onto the next step
                            break;
                        case CMTWait:                                           // If we get either the reject or wait message...
                        case CMTReject:
                            set_policy_state(peSinkReady, __FUNCTION__, __LINE__);                          // Go back to the sink ready state
                            set_policy_subindex(0);                                 // Clear the sub index
                            set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                // Clear the transmitter status
                            break;
                        default:                                                // For all other commands received, simply ignore them
                            break;
                    }
                }
            }
            else if (!PolicyStateTimer)                                         // If the SenderResponseTimer times out...
            {
                set_policy_state(peSinkReady, __FUNCTION__, __LINE__);                                      // Go back to the sink ready state
                set_policy_subindex(0);                                             // Clear the sub index
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Clear the transmitter status
            }
            break;
        case 2:     // Wait for a PS_RDY message to be received to indicate that the original source is no longer supplying VBUS
            if (ProtocolMsgRx)                                                  // Have we received a message from the source?
            {
                ProtocolMsgRx = FALSE;                                          // Reset the message received flag since we're handling it here
				PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
                if (PolicyRxHeader.NumDataObjects == 0)                         // If we have received a command
                {
                    switch (PolicyRxHeader.MessageType)                         // Determine which command was received
                    {
                        case CMTPS_RDY:                                         // If we get the PS_RDY message...
                            RoleSwapToAttachedSource();                            // Initiate the Type-C state machine for a power role swap
                            PolicyStateTimer = tPSSourceOnNom;              // Allow the output voltage to rise before sending the PS_RDY message
                            usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                            increase_policy_subindex();                                   // Increment the sub-index to move onto the next state
                            break;
                        default:                                                // For all other commands received, simply ignore them
                            break;
                    }
                }
            }
            else if (!PolicyStateTimer)                                         // If the PSSourceOn times out...
            {
                set_policy_state(peErrorRecovery, __FUNCTION__, __LINE__);                                  // Go to the error recovery state
                set_policy_subindex(0);                                             // Clear the sub index
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Clear the transmitter status
            }
            break;
        default: // Allow time for the supply to rise and then send the PS_RDY message
            if (!PolicyStateTimer)
            {
                Status = PolicySendCommandNoReset(CMTPS_RDY, peSourceStartup, 0);   // When we get the good CRC, we move onto the source startup state to complete the swap
                if (Status == STAT_ERROR)
                    set_policy_state(peErrorRecovery, __FUNCTION__, __LINE__);                              // If we get an error, go to the error recovery state
            }
            break;
    }
}

void PolicySinkEvaluatePRSwap(void)
{
    UINT8 Status;
    switch(PolicySubIndex)
    {
        case 0: // Send either the Accept or Reject command
            if (PortType != USBTypeC_DRP)                                       // For ease, just determine Accept/Reject based on PortType
            {
                PolicySendCommand(CMTReject, peSinkReady, 0);                   // Send the reject if we are not a DRP
            }
            else
            {
                if (PolicySendCommand(CMTAccept, peSinkEvaluatePRSwap, 1) == STAT_SUCCESS) // Send the Accept message and wait for the good CRC
                {
                    PRSwapTimer = tPRSwapBailout;                               // Initialize the PRSwapTimer to indicate we are in the middle of a swap
                    usbpd_start_timer(&prswap_hrtimer, PRSwapTimer);
                    PolicyStateTimer = tPSSourceOffMax;                            // Start the sink transition timer
                    usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                }
            }
            break;
        case 1: // Wait for the PS_RDY command to come in and indicate the source has turned off VBUS
            if (ProtocolMsgRx)                                                  // Have we received a message from the source?
            {
                ProtocolMsgRx = FALSE;                                          // Reset the message received flag since we're handling it here
				PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
                if (PolicyRxHeader.NumDataObjects == 0)                         // If we have received a command
                {
                    switch (PolicyRxHeader.MessageType)                         // Determine which command was received
                    {
                        case CMTPS_RDY:                                         // If we get the PS_RDY message...
                            RoleSwapToAttachedSource();                            // Initiate the Type-C state machine for a power role swap
                            PolicyStateTimer = tPSSourceOnNom;              // Allow the output voltage to rise before sending the PS_RDY message
                            usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                            increase_policy_subindex();                                   // Increment the sub-index to move onto the next state
                            break;
                        default:                                                // For all other commands received, simply ignore them
                            break;
                    }
                }
            }
            else if (!PolicyStateTimer)                                         // If the PSSourceOn times out...
            {
                set_policy_state(peErrorRecovery, __FUNCTION__, __LINE__);                                  // Go to the error recovery state
                set_policy_subindex(0);                                             // Clear the sub index
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Clear the transmitter status
            }
            break;
        default:    // Wait for VBUS to rise and then send the PS_RDY message
            if (!PolicyStateTimer)
            {
                Status = PolicySendCommandNoReset(CMTPS_RDY, peSourceStartup, 0);   // When we get the good CRC, we move onto the source startup state to complete the swap
                if (Status == STAT_ERROR)
                    set_policy_state(peErrorRecovery, __FUNCTION__, __LINE__);                              // If we get an error, go to the error recovery state
            }
            break;
    }
}

// ########################## General PD Messaging ########################## //

BOOL PolicySendHardReset(PolicyState_t nextState, UINT32 delay)
{
    BOOL Success = FALSE;
    switch (PolicySubIndex)
    {
        case 0:
            switch (PDTxStatus)
            {
                case txReset:
                case txWait:
                    // Do nothing until the protocol layer finishes generating the hard reset signaling
                    // The next state should be either txCollision or txSuccess
                    break;
                case txSuccess:
                    PolicyStateTimer = delay;                                   // Set the amount of time prior to proceeding to the next state
                    usbpd_start_timer(&policystate_hrtimer, PolicyStateTimer);
                    increase_policy_subindex();                                           // Move onto the next state
                    Success = TRUE;
                    break;
                default:                                                        // None of the other states should actually occur, so...
                    set_pdtx_state(txReset, __FUNCTION__, __LINE__);                                       // Set the transmitter status to resend a hard reset
                    break;
            }
            break;
        default:
            if (PolicyStateTimer == 0)                                          // Once tPSHardReset has elapsed...
            {
                HardResetCounter++;                                             // Increment the hard reset counter once successfully sent
                set_policy_state(nextState, __FUNCTION__, __LINE__);                                        // Go to the state to transition to the default sink state
                set_policy_subindex(0);                                             // Clear the sub index
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Clear the transmitter status
            }
            break;
    }
    return Success;
}

UINT8 PolicySendCommand(UINT8 Command, PolicyState_t nextState, UINT8 subIndex)
{
    UINT8 Status = STAT_BUSY;
    if ( ProtocolMsgTx == TRUE)
    {
        set_policy_state(nextState, __FUNCTION__, __LINE__);                                            // Go to the next state requested
        set_policy_subindex(subIndex);
        set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                                // Reset the transmitter status
        Status = STAT_SUCCESS;
        ProtocolMsgTx = FALSE;
    }
    else {
    switch (PDTxStatus)
    {
        case txIdle:
            PolicyTxHeader.word = 0;                                            // Clear the word to initialize for each transaction
            PolicyTxHeader.NumDataObjects = 0;                                  // Clear the number of objects since this is a command
            PolicyTxHeader.MessageType = Command & 0x0F;                        // Sets the message type to the command passed in
            PolicyTxHeader.PortDataRole = PolicyIsDFP;                          // Set whether the port is acting as a DFP or UFP
            PolicyTxHeader.PortPowerRole = PolicyIsSource;                      // Set whether the port is serving as a power source or sink
            PolicyTxHeader.SpecRevision = USBPDSPECREV;                         // Set the spec revision
            set_pdtx_state(txSend, __FUNCTION__, __LINE__);                                                // Indicate to the Protocol layer that there is something to send
            break;
        case txSend:
        case txBusy:
        case txWait:
            // Waiting for GoodCRC or timeout of the protocol
            // May want to put in a second level timeout in case there's an issue with the protocol getting hung
            break;
        case txSuccess:
            set_policy_state(nextState, __FUNCTION__, __LINE__);                                            // Go to the next state requested
            set_policy_subindex(subIndex);
            set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                                // Reset the transmitter status
            Status = STAT_SUCCESS;
            break;
        case txError:                                                           // Didn't receive a GoodCRC message...
            if (PolicyState == peSourceSendSoftReset)                           // If as a source we already failed at sending a soft reset...
                set_policy_state(peSourceSendHardReset, __FUNCTION__, __LINE__);                            // Move onto sending a hard reset (source)
            else if (PolicyState == peSinkSendSoftReset)                        // If as a sink we already failed at sending a soft reset...
                set_policy_state(peSinkSendHardReset, __FUNCTION__, __LINE__);                              // Move onto sending a hard reset (sink)
            else if (PolicyIsSource)                                            // Otherwise, if we are a source...
                set_policy_state(peSourceSendSoftReset, __FUNCTION__, __LINE__);                            // Attempt to send a soft reset (source)
            else                                                                // We are a sink, so...
                set_policy_state(peSinkSendSoftReset, __FUNCTION__, __LINE__);                              // Attempt to send a soft reset (sink)
            set_policy_subindex(0);                                                 // Reset the sub index
            set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                                // Reset the transmitter status
            Status = STAT_ERROR;
            break;
        case txCollision:
            CollisionCounter++;                                                 // Increment the collision counter
            if (CollisionCounter > nRetryCount)                                 // If we have already retried two times
            {
                if (PolicyIsSource)
                    set_policy_state(peSourceSendHardReset, __FUNCTION__, __LINE__);                        // Go to the source hard reset state
                else
                    set_policy_state(peSinkSendHardReset, __FUNCTION__, __LINE__);                          // Go to the sink hard reset state
                set_policy_subindex(0);                                             // Reset the sub index
                set_pdtx_state(txReset, __FUNCTION__, __LINE__);                                           // Set the transmitter status to send a hard reset
                Status = STAT_ERROR;
            }
            else                                                                // Otherwise we are going to try resending the soft reset
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Clear the transmitter status for the next operation
            break;
        default:                                                                // For an undefined case, reset everything (shouldn't get here)
            if (PolicyIsSource)
                set_policy_state(peSourceSendHardReset, __FUNCTION__, __LINE__);                            // Go to the source hard reset state
            else
                set_policy_state(peSinkSendHardReset, __FUNCTION__, __LINE__);                              // Go to the sink hard reset state
            set_policy_subindex(0);                                                 // Reset the sub index
            set_pdtx_state(txReset, __FUNCTION__, __LINE__);                                               // Set the transmitter status to send a hard reset
            Status = STAT_ERROR;
            break;
    }
    }
    return Status;
}

UINT8 PolicySendCommandNoReset(UINT8 Command, PolicyState_t nextState, UINT8 subIndex)
{
    UINT8 Status = STAT_BUSY;
    switch (PDTxStatus)
    {
        case txIdle:
            PolicyTxHeader.word = 0;                                            // Clear the word to initialize for each transaction
            PolicyTxHeader.NumDataObjects = 0;                                  // Clear the number of objects since this is a command
            PolicyTxHeader.MessageType = Command & 0x0F;                        // Sets the message type to the command passed in
            PolicyTxHeader.PortDataRole = PolicyIsDFP;                          // Set whether the port is acting as a DFP or UFP
            PolicyTxHeader.PortPowerRole = PolicyIsSource;                      // Set whether the port is serving as a power source or sink
            PolicyTxHeader.SpecRevision = USBPDSPECREV;                         // Set the spec revision
            set_pdtx_state(txSend, __FUNCTION__, __LINE__);                                                // Indicate to the Protocol layer that there is something to send
            break;
        case txSend:
        case txBusy:
        case txWait:
            // Waiting for GoodCRC or timeout of the protocol
            // May want to put in a second level timeout in case there's an issue with the protocol getting hung
            break;
        case txSuccess:
            set_policy_state(nextState, __FUNCTION__, __LINE__);                                            // Go to the next state requested
            set_policy_subindex(subIndex);
            set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                                // Reset the transmitter status
            Status = STAT_SUCCESS;
            break;
        default:                                                                // For all error cases (and undefined),
            set_policy_state(peErrorRecovery, __FUNCTION__, __LINE__);                                      // Go to the error recovery state
            set_policy_subindex(0);                                                 // Reset the sub index
            set_pdtx_state(txReset, __FUNCTION__, __LINE__);                                               // Set the transmitter status to send a hard reset
            Status = STAT_ERROR;
            break;
    }
    return Status;
}

UINT8 PolicySendData(UINT8 MessageType, UINT8 NumDataObjects, doDataObject_t* DataObjects, PolicyState_t nextState, UINT8 subIndex)
{
    UINT8 Status = STAT_BUSY;
    int i;
    if (ProtocolMsgTx == TRUE )
    {
        set_policy_state(nextState, __FUNCTION__, __LINE__);                                            // Go to the next state requested
        set_policy_subindex(subIndex);
        set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                                // Reset the transmitter status
        Status = STAT_SUCCESS;
        ProtocolMsgTx = FALSE;
    }
    else
    {
    switch (PDTxStatus)
    {
        case txIdle:
            if (NumDataObjects > 7)
                NumDataObjects = 7;
            PolicyTxHeader.word = 0x0000;                                       // Clear the word to initialize for each transaction
            PolicyTxHeader.NumDataObjects = NumDataObjects;                     // Set the number of data objects to send
            PolicyTxHeader.MessageType = MessageType & 0x0F;                    // Sets the message type to the what was passed in
            PolicyTxHeader.PortDataRole = PolicyIsDFP;                          // Set whether the port is acting as a DFP or UFP
            PolicyTxHeader.PortPowerRole = PolicyIsSource;                      // Set whether the port is serving as a power source or sink
            PolicyTxHeader.SpecRevision = USBPDSPECREV;                         // Set the spec revision
            for (i=0; i<NumDataObjects; i++)                                    // Loop through all of the data objects sent
                PolicyTxDataObj[i].object = DataObjects[i].object;              // Set each buffer object to send for the protocol layer
            if (PolicyState == peSourceSendCaps)                                // If we are in the send source caps state...
                CapsCounter++;                                                  // Increment the capabilities counter
            set_pdtx_state(txSend, __FUNCTION__, __LINE__);                                                // Indicate to the Protocol layer that there is something to send
            break;
        case txSend:
        case txBusy:
        case txWait:
            // Waiting for GoodCRC or timeout of the protocol
            // May want to put in a second level timeout in case there's an issue with the protocol getting hung
            break;
        case txSuccess:
            set_policy_state(nextState, __FUNCTION__, __LINE__);                                            // Go to the next state requested
            set_policy_subindex(subIndex);
            set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                                // Reset the transmitter status
            Status = STAT_SUCCESS;
            break;
        case txError:                                                           // Didn't receive a GoodCRC message...
            if (PolicyState == peSourceSendCaps)                                // If we were in the send source caps state when the error occurred...
                set_policy_state(peSourceDiscovery, __FUNCTION__, __LINE__);                                // Go to the discovery state
            else if (PolicyIsSource)                                            // Otherwise, if we are a source...
                set_policy_state(peSourceSendSoftReset, __FUNCTION__, __LINE__);                            // Attempt to send a soft reset (source)
            else                                                                // Otherwise...
                set_policy_state(peSinkSendSoftReset, __FUNCTION__, __LINE__);                              // Go to the soft reset state
            set_policy_subindex(0);                                                 // Reset the sub index
            set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                                // Reset the transmitter status
            Status = STAT_ERROR;
            break;
        case txCollision:
            CollisionCounter++;                                                 // Increment the collision counter
            if (CollisionCounter > nRetryCount)                                 // If we have already retried two times
            {
                if (PolicyIsSource)
                    set_policy_state(peSourceSendHardReset, __FUNCTION__, __LINE__);                        // Go to the source hard reset state
                else
                    set_policy_state(peSinkSendHardReset, __FUNCTION__, __LINE__);                          // Go to the sink hard reset state
                set_pdtx_state(txReset, __FUNCTION__, __LINE__);                                           // Set the transmitter status to send a hard reset
                Status = STAT_ERROR;
            }
            else                                                                // Otherwise we are going to try resending the soft reset
                set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                            // Clear the transmitter status for the next operation
            break;
        default:                                                                // For undefined cases, reset everything
            if (PolicyIsSource)
                set_policy_state(peSourceSendHardReset, __FUNCTION__, __LINE__);                            // Go to the source hard reset state
            else
                set_policy_state(peSinkSendHardReset, __FUNCTION__, __LINE__);                              // Go to the sink hard reset state
            set_policy_subindex(0);                                                 // Reset the sub index
            set_pdtx_state(txReset, __FUNCTION__, __LINE__);                                               // Set the transmitter status to send a hard reset
            Status = STAT_ERROR;
            break;
    }
    }
    return Status;
}

UINT8 PolicySendDataNoReset(UINT8 MessageType, UINT8 NumDataObjects, doDataObject_t* DataObjects, PolicyState_t nextState, UINT8 subIndex)
{
    UINT8 Status = STAT_BUSY;
    int i;
    switch (PDTxStatus)
    {
        case txIdle:
            if (NumDataObjects > 7)
                NumDataObjects = 7;
            PolicyTxHeader.word = 0x0000;                                       // Clear the word to initialize for each transaction
            PolicyTxHeader.NumDataObjects = NumDataObjects;                     // Set the number of data objects to send
            PolicyTxHeader.MessageType = MessageType & 0x0F;                    // Sets the message type to the what was passed in
            PolicyTxHeader.PortDataRole = PolicyIsDFP;                          // Set whether the port is acting as a DFP or UFP
            PolicyTxHeader.PortPowerRole = PolicyIsSource;                      // Set whether the port is serving as a power source or sink
            PolicyTxHeader.SpecRevision = USBPDSPECREV;                         // Set the spec revision
            for (i=0; i<NumDataObjects; i++)                                    // Loop through all of the data objects sent
                PolicyTxDataObj[i].object = DataObjects[i].object;              // Set each buffer object to send for the protocol layer
            if (PolicyState == peSourceSendCaps)                                // If we are in the send source caps state...
                CapsCounter++;                                                  // Increment the capabilities counter
            set_pdtx_state(txSend, __FUNCTION__, __LINE__);                                                // Indicate to the Protocol layer that there is something to send
            break;
        case txSend:
        case txBusy:
        case txWait:
            // Waiting for GoodCRC or timeout of the protocol
            // May want to put in a second level timeout in case there's an issue with the protocol getting hung
            break;
        case txSuccess:
            set_policy_state(nextState, __FUNCTION__, __LINE__);                                            // Go to the next state requested
            set_policy_subindex(subIndex);
            set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                                // Reset the transmitter status
            Status = STAT_SUCCESS;
            break;
        default:                                                                // For error cases (and undefined), ...
            set_policy_state(peErrorRecovery, __FUNCTION__, __LINE__);                                      // Go to the error recovery state
            set_policy_subindex(0);                                                 // Reset the sub index
            set_pdtx_state(txReset, __FUNCTION__, __LINE__);                                               // Set the transmitter status to send a hard reset
            Status = STAT_ERROR;
            break;
    }
    return Status;
}

void UpdateCapabilitiesRx(BOOL IsSourceCaps)
{
    int i;
    SourceCapsUpdated = IsSourceCaps;                                           // Set the flag to indicate that the received capabilities are valid
    CapsHeaderReceived.word = PolicyRxHeader.word;                              // Store the header for the latest capabilities received
    //PD_LOG("header=0x%4x\n", PolicyRxHeader.word);

    for (i=0; i<CapsHeaderReceived.NumDataObjects; i++) {                        // Loop through all of the received data objects
        CapsReceived[i].object = PolicyRxDataObj[i].object;                     // Store each capability
       //PD_LOG("i=%d, object=0x%x\n", i, PolicyRxDataObj[i].object);
   }
   for (i=CapsHeaderReceived.NumDataObjects; i<7; i++)                         // Loop through all of the invalid objects
        CapsReceived[i].object = 0;                                             // Clear each invalid object
}

// ##################### USB PD Protocol Layer Routines ##################### //

void USBPDProtocol(void)
{
    //PD_LOG("ProtocolState =%d\n", ProtocolState);
    //PD_LOG("PDtxState=%d\n", PDTxStatus);
    if (Registers.Status.I_HARDRST)
    {
		PD_LOG("%s, %d\n", __FUNCTION__, __LINE__);
        ResetProtocolLayer(TRUE);                                               // Reset the protocol layer
        if (PolicyIsSource)                                                     // If we are the source...
            set_policy_state(peSourceTransitionDefault, __FUNCTION__, __LINE__);                            // set the source transition to default
        else                                                                    // Otherwise we are the sink...
            set_policy_state(peSinkTransitionDefault, __FUNCTION__, __LINE__);                              // so set the sink transition to default
        set_policy_subindex(0);
        StoreUSBPDToken(FALSE, pdtHardReset);                                   // Store the hard reset
    }
    else
    {
		PD_LOG("ProtocolState is ==> %s\n", ProtocolStateName[ProtocolState]);
        switch (ProtocolState)
        {
            case PRLReset:
                ProtocolSendHardReset();                                        // Send a Hard Reset sequence
                set_pdtx_state(txWait, __FUNCTION__, __LINE__);                                            // Set the transmission status to wait to signal the policy engine
                set_protocol_state(PRLResetWait, __FUNCTION__, __LINE__);                                   // Go to the next state to wait for the reset signaling to complete
                ProtocolTimer = tBMCTimeout;                                    // Set a timeout so that we don't hang waiting for the reset to complete
		usbpd_start_timer(&protocol_hrtimer, ProtocolTimer);
                break;
            case PRLResetWait:                                                  // Wait for the reset signaling to complete
                ProtocolResetWait();
                break;
            case PRLTxSendingMessage:                                           // We have attempted to transmit and are waiting for it to complete or detect a collision
                ProtocolSendingMessage();                                       // Determine which state we should go to next
            case PRLIdle:                                                       // Waiting to send or receive a message
                ProtocolIdle();
                break;
            case PRLTxVerifyGoodCRC:                                            // Wait for message to be received and handle...
                ProtocolVerifyGoodCRC();
                break;
            default:                                                            // In the disabled state, don't do anything
                break;
        }
    }
}

void ProtocolIdle(void)
{
    if (PDTxStatus == txReset) // If we need to send a hard reset...
	{
		PD_LOG("[%s]: Send a hard reset!!\n", __FUNCTION__);
        set_protocol_state(PRLReset, __FUNCTION__, __LINE__); // Set the protocol state to send it
	}
    else if (PDTxStatus == txSend) // Otherwise check to see if there has been a request to send data...
    {
		PD_LOG("[%s]: Sending the request========>\n", __FUNCTION__);
        ProtocolTransmitMessage();                                              // If so, send the message
    }
	// Otherwise check to see if we have received a message and sent a GoodCRC in response
    else if (Registers.Status.I_GCRCSENT || (Registers.Status.RX_EMPTY == 0))
    {
        ProtocolGetRxPacket(); // Grab the received message to pass up to the policy engine
        set_pdtx_state(txIdle, __FUNCTION__, __LINE__); // Reset the transmitter status if we are receiving data (discard any packet to send)
		PD_LOG("[%s]: PDTxStatus ==> %s, Reset the transmitter status!!\n", __FUNCTION__, tx_status[PDTxStatus]);
    }
	else
	{
		PD_LOG("[%s]: PDTxStatus = %s, Do Nothing!!!\n", __FUNCTION__, TxStatus_name[PDTxStatus]);
	}
}

void ProtocolResetWait(void)
{
    if (Registers.Status.I_HARDSENT)                                            // Wait for the reset sequence to complete
    {
        set_protocol_state(PRLIdle, __FUNCTION__, __LINE__);                                                // Have the protocol state go to idle
        set_pdtx_state(txSuccess, __FUNCTION__, __LINE__);                                                 // Alert the policy engine that the reset signaling has completed
    }
    else if (ProtocolTimer == 0)                                                // Wait for the BMCTimeout period before stating success in case the interrupts don't line up
    {
        set_protocol_state(PRLIdle, __FUNCTION__, __LINE__);                                                // Have the protocol state go to idle
        set_pdtx_state(txSuccess, __FUNCTION__, __LINE__);                                                  // Assume that we have successfully sent a hard reset for now (may change in future)
    }
}

void ProtocolGetRxPacket(void)
{
    int i, j;
    UINT8 data[3];
    FUSB300ReadFIFO(3, &data[0]);                                          // Read the Rx token and two header bytes
    PolicyRxHeader.byte[0] = data[1];
    PolicyRxHeader.byte[1] = data[2];

	/* Debug header */
	if (PolicyRxHeader.NumDataObjects)
		PD_LOG("Message Type          	: %s\n", DM_message_type[PolicyRxHeader.MessageType]);
	else
		PD_LOG("Message Type          	: %s\n", CM_message_type[PolicyRxHeader.MessageType]);

	PD_LOG("Port Data Role        	: %s\n", PolicyRxHeader.PortDataRole == 1 ? "DFP" : "UFP");
	PD_LOG("Specification Revision	: %s\n", PolicyRxHeader.SpecRevision == 1 ? "2.0" : "1.0");
	PD_LOG("Port Power Role       	: %s\n", PolicyRxHeader.PortPowerRole == 1 ? "Source" : "Sink");
	PD_LOG("Message ID            	: 0x%x\n", PolicyRxHeader.MessageID);
	PD_LOG("Number of Data Objects	: 0x%x\n", PolicyRxHeader.NumDataObjects);
    // Only setting the Tx header here so that we can store what we expect was sent in our PD buffer for the GUI
    /*
    PolicyTxHeader.word = 0;                                                    // Clear the word to initialize for each transaction
    PolicyTxHeader.NumDataObjects = 0;                                          // Clear the number of objects since this is a command
    PolicyTxHeader.MessageType = CMTGoodCRC;                                    // Sets the message type to GoodCRC
    PolicyTxHeader.PortDataRole = PolicyIsDFP;                                  // Set whether the port is acting as a DFP or UFP
    PolicyTxHeader.PortPowerRole = PolicyIsSource;                              // Set whether the port is serving as a power source or sink
    PolicyTxHeader.SpecRevision = USBPDSPECREV;                                 // Set the spec revision
    PolicyTxHeader.MessageID = PolicyRxHeader.MessageID;                        // Update the message ID for the return packet
    */
    if ((data[0] & 0xE0) == 0xE0)
    {
        if ((PolicyRxHeader.NumDataObjects == 0) && (PolicyRxHeader.MessageType == CMTSoftReset))
        {
            MessageIDCounter = 0;                                                   // Clear the message ID counter for tx
            MessageID = 0xFF;                                                       // Reset the message ID (always alert policy engine of soft reset)
            ProtocolMsgRx = TRUE;                                                   // Set the flag to pass the message to the policy engine
			PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
            SourceCapsUpdated = TRUE;                                               // Set the source caps updated flag to indicate to the GUI to update the display
        }
        else if (PolicyRxHeader.MessageID != MessageID)                             // If the message ID does not match the stored...
        {
            MessageID = PolicyRxHeader.MessageID;                                   // Update the stored message ID
            ProtocolMsgRx = TRUE;                                                   // Set the flag to pass the message to the policy engine
			PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
        }
    }
    if (PolicyRxHeader.NumDataObjects > 0)                                      // Did we receive a data message? If so, we want to retrieve the data
    {
        FUSB300ReadFIFO((PolicyRxHeader.NumDataObjects<<2), &ProtocolRxBuffer[0]);   // Grab the data from the FIFO
        for (i=0; i<PolicyRxHeader.NumDataObjects; i++)                         // Load the FIFO data into the data objects (loop through each object)
        {
            for (j=0; j<4; j++) {                                                // Loop through each byte in the object
                PolicyRxDataObj[i].byte[j] = ProtocolRxBuffer[j + (i<<2)];      // Store the actual bytes
            }

			if (3 == PolicyRxHeader.MessageType)
			{
				PD_LOG("MinMaxCurrent      			0x%03x\n", 	PolicyRxDataObj[i].FVRDO.MinMaxCurrent);
				PD_LOG("OpCurrent          			0x%03x\n", 	PolicyRxDataObj[i].FVRDO.OpCurrent);
				PD_LOG("NoUSBSuspend      			0x%x\n", 	PolicyRxDataObj[i].FVRDO.NoUSBSuspend);
				PD_LOG("USBCommCapable    			0x%x\n", 	PolicyRxDataObj[i].FVRDO.USBCommCapable);
				PD_LOG("CapabilityMismatch			0x%x\n", 	PolicyRxDataObj[i].FVRDO.CapabilityMismatch);
				PD_LOG("GiveBack          			0x%x\n", 	PolicyRxDataObj[i].FVRDO.GiveBack);
				PD_LOG("ObjectPosition    			0x%x\n", 	PolicyRxDataObj[i].FVRDO.ObjectPosition);
			}
			else if (PolicyRxHeader.MessageType > 13 || 0 == PolicyRxHeader.MessageType)
			{
				/* Dump nothing */
				PD_LOG("Reserved message type!\n");
			}
			else
			{
				PD_LOG("MaxCurrent:			%dA\n", PolicyRxDataObj[i].FPDOSupply.MaxCurrent / 100);
				PD_LOG("Voltage:			%dV\n", PolicyRxDataObj[i].FPDOSupply.Voltage / 20);
				PD_LOG("PeakCurrent:		0x%x\n", PolicyRxDataObj[i].FPDOSupply.PeakCurrent);
				PD_LOG("DataRoleSwap:		0x%x\n", PolicyRxDataObj[i].FPDOSupply.DataRoleSwap);
				PD_LOG("USBCommCapable:		0x%x\n", PolicyRxDataObj[i].FPDOSupply.USBCommCapable);
				PD_LOG("ExternallyPowered:	0x%x\n", PolicyRxDataObj[i].FPDOSupply.ExternallyPowered);
				PD_LOG("USBSuspendSupport:	0x%x\n", PolicyRxDataObj[i].FPDOSupply.USBSuspendSupport);
				PD_LOG("DualRolePower:		0x%x\n", PolicyRxDataObj[i].FPDOSupply.DualRolePower);
				PD_LOG("SupplyType:			0x%x\n", PolicyRxDataObj[i].FPDOSupply.SupplyType);
			}
        }
    }
    FUSB300ReadFIFO(4, &ProtocolCRC[0]);                                   // Read out the 4 CRC bytes to move the address to the next packet beginning
	PD_LOG("CRC code is 0x%02x, 0x%02x, 0x%02x, 0x%02x\n", ProtocolCRC[0],  ProtocolCRC[1], ProtocolCRC[2], ProtocolCRC[3]);
    FUSB300Read(regStatus0, 2, &Registers.Status.byte[4]);                      // Read the status bytes to update the ACTIVITY flag (should be clear)
    //ProtocolFlushRxFIFO();
    //StoreUSBPDMessage(PolicyRxHeader, &PolicyRxDataObj[0], FALSE, data[0]);     // Store the received PD message for the device policy manager (VB GUI)
    //if ((data[0] & 0xE0) == 0xE0)                                               // Only store that we send a GoodCRC if we actually send a GoodCRC
    //    StoreUSBPDMessage(PolicyTxHeader, &PolicyTxDataObj[0], TRUE, 0xE0);     // Store the GoodCRC message that we have sent (SOP)
}

void ProtocolTransmitMessage(void)
{
    int i, j;
    ProtocolFlushTxFIFO();                                                      // Flush the Tx FIFO
    ProtocolLoadSOP();
    if ((PolicyTxHeader.NumDataObjects == 0) && (PolicyTxHeader.MessageType == CMTSoftReset))
    {
        MessageIDCounter = 0;                                                   // Clear the message ID counter if transmitting a soft reset
        MessageID = 0xFF;                                                       // Reset the message ID if transmitting a soft reset
        SourceCapsUpdated = TRUE;                                               // Set the flag to indicate to the GUI to update the display
    }
    PolicyTxHeader.MessageID = MessageIDCounter;                                // Update the tx message id to send
    ProtocolTxBuffer[ProtocolTxBytes++] = PACKSYM | (2+(PolicyTxHeader.NumDataObjects<<2));   // Load the PACKSYM token with the number of bytes in the packet
    ProtocolTxBuffer[ProtocolTxBytes++] = PolicyTxHeader.byte[0];               // Load in the first byte of the header
    ProtocolTxBuffer[ProtocolTxBytes++] = PolicyTxHeader.byte[1];               // Load in the second byte of the header
    if (PolicyTxHeader.NumDataObjects > 0)                                      // If this is a data object...
    {
        for (i = 0; i < PolicyTxHeader.NumDataObjects; i++)                         // Load the FIFO data into the data objects (loop through each object)
        {
            for (j = 0; j < 4; j++)                                                 // Loop through each byte in the object
                ProtocolTxBuffer[ProtocolTxBytes++] = PolicyTxDataObj[i].byte[j];  // Load the actual bytes
        }
    }
    ProtocolLoadEOP();                                                          // Load the CRC, EOP and stop sequence

	/* Debug Send */
	for (i = 0; i < ProtocolTxBytes; i++)
		PD_LOG("ProtocolTxBuffer[%d] = 0x%x\n", i, ProtocolTxBuffer[i]);

    FUSB300WriteFIFO(ProtocolTxBytes, &ProtocolTxBuffer[0]);               // Commit the FIFO to the FUSB300
    Registers.Control.TX_START = 1;                                             // Set the bit to enable the transmitter
    FUSB300Write(regControl0, 1, &Registers.Control.byte[0]);                   // Commit TX_START to the FUSB300
    Registers.Control.TX_START = 0;                                             // Clear this bit, to avoid inadvertently resetting
    set_pdtx_state(txBusy, __FUNCTION__, __LINE__);                                                        // Set the transmitter status to busy
    set_protocol_state(PRLTxSendingMessage, __FUNCTION__, __LINE__);                                        // Set the protocol state to wait for the transmission to complete
    ProtocolTimer = tBMCTimeout;                                                // Set the protocol timer for ~2.5ms to allow the BMC to finish transmitting before timing out
    usbpd_start_timer(&protocol_hrtimer, ProtocolTimer);
    StoreUSBPDMessage(PolicyTxHeader, &PolicyTxDataObj[0], TRUE, 0xE0);         // Store all messages that we attempt to send for debugging (SOP)
}

void ProtocolSendingMessage(void)
{
    PD_LOG("enter %s\n", __func__);
    if (Registers.Status.I_TXSENT)
    {
        ProtocolFlushTxFIFO();                                                  // Flush the Tx FIFO
        ProtocolVerifyGoodCRC();
    }
    else if (Registers.Status.I_COLLISION)                                      // If there was a collision on the bus...
    {
        ProtocolFlushTxFIFO();                                              // Discard the message and flush the Tx FIFO (enable the auto preamble)
        set_pdtx_state(txCollision, __FUNCTION__, __LINE__);                                               // Indicate to the policy engine that there was a collision with the last transmission
        ProtocolTimer = tBMCTimeout;                                            // Set a timeout so that we don't hang waiting for a packet
	usbpd_start_timer(&protocol_hrtimer, ProtocolTimer);
        set_protocol_state(PRLRxWait, __FUNCTION__, __LINE__);                                              // Go to the RxWait state to receive whatever message is incoming...
    }
    else if (ProtocolTimer == 0)                                                // If we have timed out waiting for the transmitter to complete...
    {
        ProtocolFlushTxFIFO();                                                  // Discard the message and flush the Tx FIFO
        ProtocolFlushRxFIFO();                                                  // Flush the Rx FIFO
        set_pdtx_state(txError, __FUNCTION__, __LINE__);                                                   // Set the transmission status to error to signal the policy engine
        set_protocol_state(PRLIdle, __FUNCTION__, __LINE__);                                                // Set the state variable to the idle state
    }
}

void ProtocolVerifyGoodCRC(void)
{
    int i, j;
    UINT8 data[3];
	PD_LOG("%s, %d\n", __FUNCTION__, __LINE__);
    FUSB300ReadFIFO(3, &data[0]);                                          // Read the Rx token and two header bytes
    PolicyRxHeader.byte[0] = data[1];
    PolicyRxHeader.byte[1] = data[2];
	PD_LOG("NumDataObjects = %d, MessageType = %s, MessageID = %d, MIDC = %d\n", PolicyRxHeader.NumDataObjects, CM_message_type[PolicyRxHeader.MessageType], PolicyRxHeader.MessageID, MessageIDCounter);
    if ((PolicyRxHeader.NumDataObjects == 0) && (PolicyRxHeader.MessageType == CMTGoodCRC))
    {
        if (PolicyRxHeader.MessageID != MessageIDCounter)                       // If the message ID doesn't match...
        {
			PD_LOG("Warning: ========> message ID doesn't match\n");
            FUSB300ReadFIFO(4, &ProtocolCRC[0]);                           // Read out the 4 CRC bytes to move the address to the next packet beginning
            StoreUSBPDToken(FALSE, pdtBadMessageID);                            // Store that there was a bad message ID received in the buffer
            set_pdtx_state(txError, __FUNCTION__, __LINE__);                                                   // Set the transmission status to error to signal the policy engine
            set_protocol_state(PRLIdle, __FUNCTION__, __LINE__);                                                // Set the state variable to the idle state
        }
        else                                                                    // Otherwise, we've received a good CRC response to our message sent
        {
			PD_LOG("received a good CRC response to our message sent\n");
            MessageIDCounter++;                                                 // Increment the message ID counter
            MessageIDCounter &= 0x07;                                           // Rollover the counter so that it fits
            set_protocol_state(PRLIdle, __FUNCTION__, __LINE__);                                            // Set the idle state
            set_pdtx_state(txSuccess, __FUNCTION__, __LINE__);                                             // Set the transmission status to success to signal the policy engine
            ProtocolMsgTx = TRUE;
            FUSB300ReadFIFO(4, &ProtocolCRC[0]);                                              // Flush the Rx FIFO to make sure it's clean upon the next read
            StoreUSBPDMessage(PolicyRxHeader, &PolicyRxDataObj[0], FALSE, data[0]);   // Store the received PD message for the device policy manager (VB GUI)
        }
    }
    else
    {
		PD_LOG("CRC error, let the policy engine decide next steps\n");
        set_protocol_state(PRLIdle, __FUNCTION__, __LINE__);                                                // Set the idle protocol state (let the policy engine decide next steps)
        set_pdtx_state(txError, __FUNCTION__, __LINE__);                                                   // Flag the policy engine that we didn't successfully transmit
        if ((PolicyRxHeader.NumDataObjects == 0) && (PolicyRxHeader.MessageType == CMTSoftReset))
        {
            FUSB300ReadFIFO(4, &ProtocolCRC[0]);                           // Read out the 4 CRC bytes to move the address to the next packet beginning
            MessageIDCounter = 0;                                               // Clear the message ID counter for tx
            MessageID = 0xFF;                                                   // Reset the message ID (always alert policy engine of soft reset)
            ProtocolMsgRx = TRUE;                                               // Set the flag to pass the message to the policy engine
			PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
            SourceCapsUpdated = TRUE;                                           // Set the flag to indicate to the GUI to update the display
        }
        else if (PolicyRxHeader.MessageID != MessageID)                         // If the message ID does not match the stored...
        {
            FUSB300ReadFIFO(4, &ProtocolCRC[0]);                           // Read out the 4 CRC bytes to move the address to the next packet beginning
            MessageID = PolicyRxHeader.MessageID;                               // Update the stored message ID
            ProtocolMsgRx = TRUE;                                               // Set the flag to pass the message to the policy engine
			PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
        }
        if (PolicyRxHeader.NumDataObjects > 0)                                  // If this is a data message, grab the data objects
        {
           FUSB300ReadFIFO(PolicyRxHeader.NumDataObjects<<2, &ProtocolRxBuffer[0]);    // Grab the data from the FIFO
            for (i=0; i<PolicyRxHeader.NumDataObjects; i++)                     // Load the FIFO data into the data objects (loop through each object)
            {
                for (j=0; j<4; j++)                                             // Loop through each byte in the object
                    PolicyRxDataObj[i].byte[j] = ProtocolRxBuffer[j + (i<<2)];  // Store the actual bytes
            }
        }
        StoreUSBPDMessage(PolicyRxHeader, &PolicyRxDataObj[0], FALSE, data[0]); // Store the received PD message for the device policy manager (VB GUI)
    }
 //   ProtocolFlushRxFIFO();
}

void ProtocolLoadSOP(void)
{
    ProtocolTxBytes = 0;                                                        // Clear the Tx byte counter
    //USBPDTxBuffer[ProtocolTxBytes++] = TXON;                                  // Load the PD start sequence (turn on the transmitter) (NO COLLISION DETECTION?)
    ProtocolTxBuffer[ProtocolTxBytes++] = SOP1;                                 // Load in the Sync-1 pattern
    ProtocolTxBuffer[ProtocolTxBytes++] = SOP1;                                 // Load in the Sync-1 pattern
    ProtocolTxBuffer[ProtocolTxBytes++] = SOP1;                                 // Load in the Sync-1 pattern
    ProtocolTxBuffer[ProtocolTxBytes++] = SOP2;                                 // Load in the Sync-2 pattern
}

void ProtocolLoadEOP(void)
{
    ProtocolTxBuffer[ProtocolTxBytes++] = JAM_CRC;                              // Load in the token to calculate and add the CRC
    ProtocolTxBuffer[ProtocolTxBytes++] = EOP;                                  // Load in the EOP pattern
    ProtocolTxBuffer[ProtocolTxBytes++] = TXOFF;                                // Load in the PD stop sequence (turn off the transmitter)
}

void ProtocolSendHardReset(void)
{
    UINT8 data;
    data = Registers.Control.byte[3] | 0x40;                                    // Set the send hard reset bit
    FUSB300Write(regControl3, 1, &data);                                        // Send the hard reset
    StoreUSBPDToken(TRUE, pdtHardReset);                                        // Store the hard reset
}

void ProtocolFlushRxFIFO(void)
{
    UINT8 data;
    data = Registers.Control.byte[1];                                           // Grab the current control word
    data |= 0x04;                                                               // Set the RX_FLUSH bit (auto-clears)
    FUSB300Write(regControl1, 1, &data);                                        // Commit the flush to the FUSB300
}

void ProtocolFlushTxFIFO(void)
{
    UINT8 data;
    data = Registers.Control.byte[0];                                           // Grab the current control word
    data |= 0x40;                                                               // Set the TX_FLUSH bit (auto-clears)
    FUSB300Write(regControl0, 1, &data);                                        // Commit the flush to the FUSB300
}

void ResetProtocolLayer(BOOL ResetPDLogic)
{
    int i;
    UINT8 data = 0x02;
    if (ResetPDLogic)
        FUSB300Write(regReset, 1, &data);                                       // Reset the PD logic
    ProtocolFlushRxFIFO();                                                      // Flush the Rx FIFO
    ProtocolFlushTxFIFO();                                                  // Flush the Tx FIFO (enable Auto Preamble)
    set_protocol_state(PRLIdle, __FUNCTION__, __LINE__);                                                    // Initialize the protocol layer to the idle state
    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                                        // Initialize the transmitter status
    ProtocolTimer = 0;                                                          // Reset the protocol state timer
    ProtocolTxBytes = 0;                                                        // Clear the byte count for the Tx FIFO
    MessageIDCounter = 0;                                                       // Clear the message ID counter
    MessageID = 0xFF;                                                           // Reset the message ID (invalid value to indicate nothing received yet)
    ProtocolMsgRx = FALSE;                                                      // Reset the message ready flag
	PD_LOG("ProtocolMsgRx is set to %s, %d\n", ProtocolMsgRx == 1 ? "TRUE" : "FALSE", __LINE__);
    ProtocolMsgTx = FALSE;
    USBPDTxFlag = FALSE;                                                        // Clear the flag to make sure we don't send something by accident
    PolicyHasContract = FALSE;                                                  // Clear the flag that indicates we have a PD contract
    USBPDContract.object = 0;                                                   // Clear the actual USBPD contract request object
    SourceCapsUpdated = TRUE;                                                   // Update the source caps flag to trigger an update of the GUI
    CapsHeaderReceived.word = 0;                                                // Clear any received capabilities messages
    for (i=0; i<7; i++)                                                         // Loop through all the received capabilities objects
        CapsReceived[i].object = 0;                                             // Clear each object
}

// ####################### USB PD Debug Buffer Routines ##################### //

BOOL StoreUSBPDToken(BOOL transmitter, USBPD_BufferTokens_t token)
{
    UINT8 header1 = 1;                                                          // Declare and set the message size
    if (ClaimBufferSpace(2) == FALSE)                                           // Attempt to claim the number of bytes required in the buffer
        return FALSE;                                                           // If there was an error, return that we failed
    if (transmitter)                                                            // If we are the transmitter...
        header1 |= 0x40;                                                        // set the transmitter bit
    USBPDBuf[USBPDBufEnd++] = header1;                                          // Set the first header byte (Token type, direction and size)
    USBPDBufEnd %= PDBUFSIZE;                                                   // Wrap the pointer if it is too large
    token &= 0x0F;                                                              // Build the 2nd header byte
    USBPDBuf[USBPDBufEnd++] = token;                                            // Set the second header byte (actual token)
    USBPDBufEnd %= PDBUFSIZE;                                                   // Wrap the pointer if it is too large
    return TRUE;
}

BOOL StoreUSBPDMessage(sopMainHeader_t Header, doDataObject_t* DataObject, BOOL transmitter, UINT8 SOPToken)
{
    int i, j, required;
    UINT8 header1;
    required = Header.NumDataObjects * 4 + 2 + 2;                               // Determine how many bytes are needed for the buffer
    if (ClaimBufferSpace(required) == FALSE)                                    // Attempt to claim the number of bytes required in the buffer
        return FALSE;                                                           // If there was an error, return that we failed
    header1 = (0x1F & (required-1)) | 0x80;
    if (transmitter)                                                            // If we were the transmitter
        header1 |= 0x40;                                                        // Set the flag to indicate to the host
    USBPDBuf[USBPDBufEnd++] = header1;                                          // Set the first header byte (PD message flag, direction and size)
    USBPDBufEnd %= PDBUFSIZE;                                                   // Wrap the pointer if it is too large
    SOPToken &= 0xE0;                                                           // Build the 2nd header byte
    SOPToken >>= 5;                                                             // Shift the token into place
    USBPDBuf[USBPDBufEnd++] = SOPToken;                                         // Set the second header byte (PD message type)
    USBPDBufEnd %= PDBUFSIZE;                                                   // Wrap the pointer if it is too large
    USBPDBuf[USBPDBufEnd++] = Header.byte[0];                                   // Set the first byte and increment the pointer
    USBPDBufEnd %= PDBUFSIZE;                                                   // Wrap the pointer if it is too large
    USBPDBuf[USBPDBufEnd++] = Header.byte[1];                                   // Set the second byte and increment the pointer
    USBPDBufEnd %= PDBUFSIZE;                                                   // Wrap the pointer if it is too large
    for (i=0; i<Header.NumDataObjects; i++)                                     // Loop through all the data objects
    {
        for (j=0; j<4; j++)
        {
            USBPDBuf[USBPDBufEnd++] = DataObject[i].byte[j];                    // Set the byte of the data object and increment the pointer
            USBPDBufEnd %= PDBUFSIZE;                                           // Wrap the pointer if it is too large
        }
    }
    return TRUE;
}

UINT8 GetNextUSBPDMessageSize(void)
{
    UINT8 numBytes;
    if (USBPDBufStart == USBPDBufEnd)                                           // If the start and end are equal, the buffer is empty
        numBytes = 0;                                                           // Clear the number of bytes so that we return 0
    else                                                                        // otherwise there is data in the buffer...
        numBytes = (USBPDBuf[USBPDBufStart] & 0x1F) + 1;                        // Get the number of bytes associated with the message
    return numBytes;
}

UINT8 GetUSBPDBufferNumBytes(void)
{
    UINT8 bytes;
    if (USBPDBufStart == USBPDBufEnd)                                           // If the buffer is empty (using the keep one slot open approach)
        bytes = 0;                                                              // return 0
    else if (USBPDBufEnd > USBPDBufStart)                                       // If the buffer hasn't wrapped...
        bytes = USBPDBufEnd - USBPDBufStart;                                    // simply subtract the end from the beginning
    else                                                                        // Otherwise it has wrapped...
        bytes = USBPDBufEnd + (PDBUFSIZE - USBPDBufStart);                      // calculate the available this way
    return bytes;
}

BOOL ClaimBufferSpace(int intReqSize)
{
    int available;
    UINT8 numBytes;
    if (intReqSize >= PDBUFSIZE)                                                // If we cannot claim enough space...
        return FALSE;                                                           // Get out of here
    if (USBPDBufStart == USBPDBufEnd)                                           // If the buffer is empty (using the keep one slot open approach)
        available = PDBUFSIZE;                                                  // Buffer is empty...
    else if (USBPDBufStart > USBPDBufEnd)                                       // If the buffer has wrapped...
        available = USBPDBufStart - USBPDBufEnd;                                // calculate this way
    else                                                                        // Otherwise
        available = PDBUFSIZE - (USBPDBufEnd - USBPDBufStart);                  // calculate the available this way
    do
    {
        if (intReqSize >= available)                                            // If we don't have enough room in the buffer, we need to make room (always keep 1 spot open)
        {
            USBPDBufOverflow = TRUE;                                            // Set the overflow flag to alert the GUI that we are overwriting data
            numBytes = GetNextUSBPDMessageSize();                               // Get the size of the next USB PD message in the buffer
            if (numBytes == 0)                                                  // If the buffer is empty...
                return FALSE;                                                   // Return FALSE since the data cannot fit in the available buffer size (nothing written)
            available += numBytes;                                              // Add the recovered bytes to the number available
            USBPDBufStart += numBytes;                                          // Adjust the pointer to the new starting address
            USBPDBufStart %= PDBUFSIZE;                                         // Wrap the pointer if necessary
        }
        else
            break;
    } while (1);                                                                // Loop until we have enough bytes
    return TRUE;
}

// ##################### PD Status Routines #################### //

void GetUSBPDStatus(unsigned char abytData[])
{
    int i, j;
    int intIndex = 0;
    abytData[intIndex++] = GetUSBPDStatusOverview();                            // Grab a snapshot of the top level status
    abytData[intIndex++] = GetUSBPDBufferNumBytes();                            // Get the number of bytes in the PD buffer
    abytData[intIndex++] = PolicyState;                                         // Get the current policy engine state
    abytData[intIndex++] = PolicySubIndex;                                      // Get the current policy sub index
    abytData[intIndex++] = (ProtocolState << 4) | PDTxStatus;                   // Get the protocol state and transmitter status
    for (i=0;i<4;i++)
            abytData[intIndex++] = USBPDContract.byte[i];                       // Get each byte of the current contract
    if (PolicyIsSource)
    {
        abytData[intIndex++] = CapsHeaderSource.byte[0];                        // Get the first byte of the received capabilities header
        abytData[intIndex++] = CapsHeaderSource.byte[1];                        // Get the second byte of the received capabilities header
        for (i=0;i<7;i++)                                                       // Loop through each data object
        {
            for (j=0;j<4;j++)                                                   // Loop through each byte of the data object
                abytData[intIndex++] = CapsSource[i].byte[j];                   // Get each byte of each power data object
        }
    }
    else
    {
        abytData[intIndex++] = CapsHeaderReceived.byte[0];                      // Get the first byte of the received capabilities header
        abytData[intIndex++] = CapsHeaderReceived.byte[1];                      // Get the second byte of the received capabilities header
        for (i=0;i<7;i++)                                                       // Loop through each data object
        {
            for (j=0;j<4;j++)                                                   // Loop through each byte of the data object
                abytData[intIndex++] = CapsReceived[i].byte[j];                 // Get each byte of each power data object
        }
    }

//    abytData[43] = ProtocolTxBytes;
//    for (i=0;i<14;i++)
//    {
//        abytData[42+i] = ProtocolTxBuffer[i];
//    }

    // We are going to return the Registers for now for debugging purposes
    // These will be removed eventually and a new command will likely be added
    // For now, offset the registers by 16 from the beginning to get them out of the way
    intIndex = 44;
    abytData[intIndex++] = Registers.DeviceID.byte;     // 52
    abytData[intIndex++] = Registers.Switches.byte[0];  // 53
    abytData[intIndex++] = Registers.Switches.byte[1];
    abytData[intIndex++] = Registers.Measure.byte;
    abytData[intIndex++] = Registers.Slice.byte;
    abytData[intIndex++] = Registers.Control.byte[0];   // 57
    abytData[intIndex++] = Registers.Control.byte[1];
    abytData[intIndex++] = Registers.Mask.byte;
    abytData[intIndex++] = Registers.Power.byte;
    abytData[intIndex++] = Registers.Status.byte[4];    // Status0 - 61
    abytData[intIndex++] = Registers.Status.byte[5];    // Status1 - 62
    abytData[intIndex++] = Registers.Status.byte[6];    // Interrupt1 - 63
}

UINT8 GetUSBPDStatusOverview(void)
{
    unsigned char status = 0;
    if (USBPDEnabled)
        status |= 0x01;
    if (USBPDActive)
        status |= 0x02;
    if (PolicyIsSource)
        status |= 0x04;
    if (PolicyIsDFP)
        status |= 0x08;
    if (PolicyHasContract)
        status |= 0x10;
    if (SourceCapsUpdated)
        status |= 0x20;
    SourceCapsUpdated = FALSE;
    if (USBPDBufOverflow)
        status |= 0x80;
    return status;
}

UINT8 ReadUSBPDBuffer(unsigned char* pData, unsigned char bytesAvail)
{
    UINT8 i, msgSize, bytesRead;
    bytesRead = 0;
    do
    {
        msgSize = GetNextUSBPDMessageSize();                                    // Grab the next message size
//        *pData++ = USBPDBufStart;                                               // 6
//        *pData++ = USBPDBufEnd;                                                 // 7
//        *pData++ = (USBPDBufStart + 1) % PDBUFSIZE;                             // 8
//       *pData++ = USBPDBuf[(USBPDBufStart + 1) % PDBUFSIZE];                   // 9
//        *pData++ = (USBPDBuf[(USBPDBufStart + 1) % PDBUFSIZE] & 0x70) >> 2;     // 10
//        *pData++ = ((USBPDBuf[(USBPDBufStart + 1) % PDBUFSIZE] & 0x70) >> 2) + 2; // 11
//        for (i=0; i<32; i++)
//            *pData++ = USBPDBuf[i];
        if ((msgSize != 0) && (msgSize <= bytesAvail))                          // If there is data and the message will fit...
        {
            for (i=0; i<msgSize; i++)                                           // Loop through all of the bytes for the message
            {
                *pData++ = USBPDBuf[USBPDBufStart++];                           // Retrieve the bytes, increment both pointers
                USBPDBufStart %= PDBUFSIZE;                                     // Wrap the start pointer if necessary
            }
            bytesAvail -= msgSize;                                              // Decrement the number of bytes available
            bytesRead += msgSize;                                               // Increment the number of bytes read
        }
        else                                                                    // If there is no data or the message won't fit...
            break;                                                              // Break out of the loop
    } while (1);
    return bytesRead;
}

void EnableUSBPD(void)
{
    BOOL enabled = blnSMEnabled;                                                // Store the current 300 state machine enabled state
    if (USBPDEnabled)                                                           // If we are already enabled...
        return;                                                                 // return since we don't have to do anything
    else
    {
        DisableFUSB300StateMachine();                                           // Disable the FUSB300 state machine to get to a known state
        USBPDEnabled = TRUE;                                                    // Set the USBPD state machine to enabled
        if (enabled)                                                            // If we were previously enabled...
            EnableFUSB300StateMachine();                                        // Re-enable the FUSB300 state machine
    }
}

void DisableUSBPD(void)
{
    BOOL enabled = blnSMEnabled;                                                // Store the current 300 state machine enabled state
    if (!USBPDEnabled)                                                          // If we are already disabled...
        return;                                                                 // return since we don't need to do anything
    else
    {
        DisableFUSB300StateMachine();                                           // Disable the FUSB300 state machine to get to a known state
        USBPDEnabled = FALSE;                                                   // Set the USBPD state machine to enabled
        if (enabled)                                                            // If we were previously enabled...
            EnableFUSB300StateMachine();                                        // Re-enable the state machine
    }
}

void SendUSBPDMessage(unsigned char* abytData)
{
    int i, j;
    PDTransmitHeader.byte[0] = *abytData++;                                     // Set the 1st PD header byte
    PDTransmitHeader.byte[1] = *abytData++;                                     // Set the 2nd PD header byte
    for (i=0; i<PDTransmitHeader.NumDataObjects; i++)                           // Loop through all the data objects
    {
        for (j=0; j<4; j++)                                                     // Loop through each byte of the object
        {
            PDTransmitObjects[i].byte[j] = *abytData++;                         // Set the actual bytes
        }
    }
    USBPDTxFlag = TRUE;                                                         // Set the flag to send when appropriate
}

void WriteSourceCapabilities(unsigned char* abytData)
{
    int i, j;
    sopMainHeader_t Header;
    Header.byte[0] = *abytData++;                                               // Set the 1st PD header byte
    Header.byte[1] = *abytData++;                                               // Set the 2nd PD header byte
    if ((Header.NumDataObjects > 0) && (Header.MessageType == DMTSourceCapabilities))   // Only do anything if we decoded a source capabilities message
    {
        CapsHeaderSource.word = Header.word;                                    // Set the actual caps source header
        for (i=0; i<CapsHeaderSource.NumDataObjects; i++)                       // Loop through all the data objects
        {
            for (j=0; j<4; j++)                                                 // Loop through each byte of the object
                CapsSource[i].byte[j] = *abytData++;                            // Set the actual bytes
        }
        if (PolicyIsSource)                                                     // If we are currently acting as the source...
        {
            PDTransmitHeader.word = CapsHeaderSource.word;                      // Set the message type to capabilities to trigger sending the caps (only need the header to trigger)
            USBPDTxFlag = TRUE;                                                 // Set the flag to send the new caps when appropriate...
            SourceCapsUpdated = TRUE;                                           // Set the flag to indicate to the software that the source caps were updated
        }
    }
}

void ReadSourceCapabilities(unsigned char* abytData)
{
    int i, j;
    *abytData++ = CapsHeaderSource.byte[0];
    *abytData++ = CapsHeaderSource.byte[1];
    for (i=0; i<CapsHeaderSource.NumDataObjects; i++)
    {
        for (j=0; j<4; j++)
            *abytData++ = CapsSource[i].byte[j];
    }
}

void WriteSinkCapabilities(unsigned char* abytData)
{
    int i, j;
    sopMainHeader_t Header;
    Header.byte[0] = *abytData++;                                               // Set the 1st PD header byte
    Header.byte[1] = *abytData++;                                               // Set the 2nd PD header byte
    if ((Header.NumDataObjects > 0) && (Header.MessageType == DMTSinkCapabilities))   // Only do anything if we decoded a source capabilities message
    {
        CapsHeaderSink.word = Header.word;                                      // Set the actual caps sink header
        for (i=0; i<CapsHeaderSink.NumDataObjects; i++)                         // Loop through all the data objects
        {
            for (j=0; j<4; j++)                                                 // Loop through each byte of the object
                CapsSink[i].byte[j] = *abytData++;                              // Set the actual bytes
        }
        // We could also trigger sending the caps or re-evaluating, but we don't do anything with this info here...
    }
}

void ReadSinkCapabilities(unsigned char* abytData)
{
    int i, j;
    *abytData++ = CapsHeaderSink.byte[0];
    *abytData++ = CapsHeaderSink.byte[1];
    for (i=0; i<CapsHeaderSink.NumDataObjects; i++)
    {
        for (j=0; j<4; j++)
            *abytData++ = CapsSink[i].byte[j];
    }
}

void WriteSinkRequestSettings(unsigned char* abytData)
{
    UINT32 uintPower;
    SinkGotoMinCompatible = *abytData & 0x01 ? TRUE : FALSE;
    SinkUSBSuspendOperation = *abytData & 0x02 ? TRUE : FALSE;
    SinkUSBCommCapable = *abytData++ & 0x04 ? TRUE : FALSE;
    SinkRequestMaxVoltage = (UINT32) *abytData++;
    SinkRequestMaxVoltage |= ((UINT32) (*abytData++) << 8);                     // Voltage resolution is 50mV
    uintPower = (UINT32) *abytData++;
    uintPower |= ((UINT32) (*abytData++) << 8);
    uintPower |= ((UINT32) (*abytData++) << 16);
    uintPower |= ((UINT32) (*abytData++) << 24);
    SinkRequestOpPower = uintPower;                                             // Power resolution is 0.5mW
    uintPower = (UINT32) *abytData++;
    uintPower |= ((UINT32) (*abytData++) << 8);
    uintPower |= ((UINT32) (*abytData++) << 16);
    uintPower |= ((UINT32) (*abytData++) << 24);
    SinkRequestMaxPower = uintPower;                                            // Power resolution is 0.5mW
    // We could try resetting and re-evaluating the source caps here, but lets not do anything until requested by the user (soft reset or detach)
}

void ReadSinkRequestSettings(unsigned char* abytData)
{
    *abytData = SinkGotoMinCompatible ? 0x01 : 0;
    *abytData |= SinkUSBSuspendOperation ? 0x02 : 0;
    *abytData++ |= SinkUSBCommCapable ? 0x04 : 0;
    *abytData++ = (UINT8) (SinkRequestMaxVoltage & 0xFF);
    *abytData++ = (UINT8) ((SinkRequestMaxVoltage & 0xFF) >> 8);
    *abytData++ = (UINT8) (SinkRequestOpPower & 0xFF);
    *abytData++ = (UINT8) ((SinkRequestOpPower >> 8) & 0xFF);
    *abytData++ = (UINT8) ((SinkRequestOpPower >> 16) & 0xFF);
    *abytData++ = (UINT8) ((SinkRequestOpPower >> 24) & 0xFF);
    *abytData++ = (UINT8) (SinkRequestMaxPower & 0xFF);
    *abytData++ = (UINT8) ((SinkRequestMaxPower >> 8) & 0xFF);
    *abytData++ = (UINT8) ((SinkRequestMaxPower >> 16) & 0xFF);
    *abytData++ = (UINT8) ((SinkRequestMaxPower >> 24) & 0xFF);
}

void SendUSBPDHardReset(void)
{
    if (PolicyIsSource)                                                         // If we are the source...
        set_policy_state(peSourceSendHardReset, __FUNCTION__, __LINE__);                                    // set the source state to send a hard reset
    else                                                                        // Otherwise we are the sink...
        set_policy_state(peSinkSendHardReset, __FUNCTION__, __LINE__);                                      // so set the sink state to send a hard reset
    set_policy_subindex(0);
    set_pdtx_state(txIdle, __FUNCTION__, __LINE__);                                                        // Reset the transmitter status
    //mPORTASetBits(BIT_0|BIT_1);
}

void InitializeVdmManager() {
	initialize_vdm(&vdmm);
#if 0
	// configure callbacks
	vdmm.req_id_info 		= sim_RequestIdentityInfo;
	vdmm.req_svid_info 		= sim_RequestSvidInfo;
	vdmm.req_modes_info 		= sim_RequestModesInfo;
	vdmm.enter_mode_result	= sim_EnterModeResult;
	vdmm.exit_mode_result 	= sim_ExitModeResult;
	vdmm.inform_id 			= sim_InformIdentity;
	vdmm.inform_svids 		= sim_InformSvids;
	vdmm.inform_modes 		= sim_InformModes;
	vdmm.inform_attention 	= sim_InformAttention;
	vdmm.req_mode_entry		= sim_ModeEntryRequest;
	vdmm.req_mode_exit		= sim_ModeExitRequest;
#endif
}


void convertAndProcessVdmMessage() {
    int i, j;
    // form the word arrays that VDM block expects
    // note: may need to rethink the interface. but this is quicker to develop right now.
    UINT32 vdm_arr[7];
    for (i = 0; i < PolicyRxHeader.NumDataObjects; i++) {
        vdm_arr[i] = 0;
        for (j = 0; j < 4; j++) {
            vdm_arr[i] <<= 8;                           // make room for next byte
            vdm_arr[i] &=  0xFFFFFF00;                  // clear bottom byte
            vdm_arr[i] |= (PolicyRxDataObj[i].byte[j]); // OR next byte in
        }
    }

    // note: assuming only SOP for now. TODO
    processVdmMessage(&vdmm, SOP, vdm_arr, PolicyRxHeader.NumDataObjects);
}

void sendVdmMessage(VdmManager* vdmm, SopType sop, UINT32* arr, unsigned int length) {
    unsigned int i, j;

    // set up transmit header
    PolicyTxHeader.MessageType = DMTVenderDefined;
    PolicyTxHeader.NumDataObjects = length;

    for (i = 0; i < PolicyTxHeader.NumDataObjects; i++) {
        for (j = 0; j < 4; j++) {
            PolicyTxDataObj[i].byte[j] = (arr[i] >> (8*j));
        }
    }

    set_pdtx_state(txSend, __FUNCTION__, __LINE__);
}

void doVdmCommand() {
    UINT32 svdm_header_bits;
    //StructuredVdmHeader svdm_header;
//    unsigned int i;
    unsigned int command;
    unsigned int svid;

    svdm_header_bits = 0;
    /*
    for (i = 0; i < 4; i++) {
        svdm_header_bits <<= 8;
        svdm_header_bits |= PDTransmitObjects[0].byte[i];
    }*/

    //svdm_header = getStructuredVdmHeader(svdm_header_bits);

    command = PDTransmitObjects[0].byte[0] & 0x1F;
    svid = 0;
    svid |= (PDTransmitObjects[0].byte[3] << 8);
    svid |= (PDTransmitObjects[0].byte[2] << 0);

    if (command == 1) {
        discoverIdentities(&vdmm, SOP);
    } else if (command == DISCOVER_SVIDS) {
        discoverSvids(&vdmm, SOP);
    } else if (command == DISCOVER_MODES) {
        discoverModes(&vdmm, SOP, svid);
    }

}
