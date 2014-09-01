#ifndef __MIC_H__
#define __MIC_H__

#ifdef __cplusplus
extern "C" {
#endif

// Size of buffer for device driver
//    12KB =  Approx. 139msec worth @ 44100Hz
//         =  Approx. 279msec worth @ 22050Hz
//         =  Approx. 557msec worth @ 11025Hz
#define MIC_RINGBUFF_SIZE         12*1024

// Returned values, tests, etc.
#define MIC_RESULT_UNLOCKED           1
#define MIC_RESULT_READY              0
#define MIC_RESULT_BUSY              -1
#define MIC_RESULT_WRONGDEVICE       -2
#define MIC_RESULT_NOCARD            -3
#define MIC_RESULT_INVALID_STATE     -4
#define MIC_RESULT_FATAL_ERROR     -128

// Button bits
#define MIC_BUTTON_0             0x0001
#define MIC_BUTTON_1             0x0002
#define MIC_BUTTON_2             0x0004
#define MIC_BUTTON_3             0x0008
#define MIC_BUTTON_4             0x0010
#define MIC_BUTTON_TALK    MIC_BUTTON_4

#define MIC_BMC              0x00000000

typedef void (*MICCallback)(s32 chan, s32 result);

void MICInit(void);
s32 MICProbeEx(s32 chan);
s32 MICGetResultCode(s32 chan);
u32 MICGetErrorCount(s32 chan);

s32 MICMountAsync(s32 chan, s16* buffer, s32 size, MICCallback detachCallback, MICCallback attachCallback);
s32 MICMount(s32 chan, s16* buffer, s32 size, MICCallback detachCallback);
s32 MICUnmount(s32 chan);
s32 MICGetRingbuffsize(s32 chan, s32* size);

s32 MICSetStatusAsync(s32 chan, u32 status, MICCallback setCallback);
s32 MICSetStatus(s32 chan, u32 status);
s32 MICGetStatus(s32 chan, u32* status);

s32 MICSetParamsAsync(s32 chan, s32 size, s32 rate, s32 gain, MICCallback setCallback);
s32 MICSetParams(s32 chan, s32 size, s32 rate, s32 gain);
s32 MICGetParams(s32 chan, s32* size, s32* rate, s32* gain);

s32 MICSetBuffsizeAsync(s32 chan, s32 size, MICCallback setCallback);
s32 MICSetBuffsize(s32 chan, s32 size);
s32 MICGetBuffsize(s32 chan, s32* size);

s32 MICSetRateAsync(s32 chan, s32 rate, MICCallback setCallback);
s32 MICSetRate(s32 chan, s32 rate);
s32 MICGetRate(s32 chan, s32* rate);

s32 MICSetGainAsync(s32 chan, s32 gain, MICCallback setCallback);
s32 MICSetGain(s32 chan, s32 gain);
s32 MICGetGain(s32 chan, s32* gain);

s32 MICGetButton(s32 chan, u32* button);
s32 MICGetDeviceID(s32 chan, u32* id);

s32 MICSetOutAsync(s32 chan, u32 pattern, MICCallback setCallback);
s32 MICSetOut(s32 chan, u32 pattern);
s32 MICGetOut(s32 chan, u32* pattern);

s32 MICStartAsync(s32 chan, MICCallback startCallback);
s32 MICStart(s32 chan);

s32 MICStopAsync(s32 chan, MICCallback stopCallback );
s32 MICStop(s32 chan);

s32 MICResetAsync(s32 chan, MICCallback stopCallback );
s32 MICReset(s32 chan);

BOOL MICIsActive(s32 chan);
BOOL MICIsAttached(s32 chan);

s32 MICGetCurrentTop(s32 chan);
s32 MICUpdateIndex(s32 chan, s32 index, s32 samples);
s32 MICGetSamplesLeft(s32 chan, s32 index);
s32 MICGetSamples(s32 chan, s16* buffer, s32 index, s32 samples);

MICCallback MICSetExiCallback(s32 chan, MICCallback exiCallback);
MICCallback MICSetTxCallback (s32 chan, MICCallback txCallback );


#ifdef __cplusplus
}
#endif

#endif
