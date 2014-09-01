#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <ogcsys.h>

#include "asm.h"
#include "processor.h"
#include "exi.h"
#include "lwp.h"
#include "lwp_watchdog.h"
#include "system.h"
#include "semaphore.h"

#include "mic.h"


#define MIC_EXI_ID	0x0a000000

#define MIC_DMA_DATA		0x20
#define MIC_READ_STATUS		0x40
#define MIC_WRITE_STATUS	0x80
#define MIC_RESET			0xff

#define MIC_STATUS_BUFOVRFLW	0x0200

#define MIC_STATUS_GAIN0		0
#define MIC_STATUS_GAIN15		0x0400

#define MIC_STATUS_11025Hz		0
#define MIC_STATUS_22050Hz		0x0800
#define MIC_STATUS_44100Hz		0x1000

#define MIC_STATUS_32BYTES		0
#define MIC_STATUS_64BYTES		0x2000
#define MIC_STATUS_128BYTES		0x4000

#define MIC_STATUS_ACTIVE		0x8000


struct MICControlBlock
{
	s32 result_code;
	
	BOOL is_attached;
	BOOL is_active;
	
	lwpq_t thread_queue;
	
	MICCallback exi_callback;
	MICCallback tx_callback;
	MICCallback detach_callback;
	MICCallback attach_callback;
	MICCallback mount_callback;
	MICCallback set_callback;
	
	u32 last_status;
	u32 status;
	
	// Number of times the hw ringbuffer has been observed overwriting itself
	// without the driver first reading the data
	u32 error_count;
	
	// The driver manages two ringbuffers: the hardware buffer in the mic which
	// is small (32|64|128), and the user-supplied buffer which is filled from
	// the hardware buffer and used as the source of GetSamples.
	
	// Hardware parameters
	// The hw ringbuffer size which generates exi interrupts
	u32 hw_buff_size;
	u32 sample_rate;
	u32 gain;
	
	// User-supplied buffer management
	s16 *buff_ring_base;// aligned up from user-supplied ptr
	u32 buff_size;		// size usable (buff_ring_base to end of given buffer)
	u32 buff_ring_size;	// size usable (buff_ring_base to max multiple of hw_buff_size)
	u32 buff_ring_cur;	// current byte in ringbuffer
	
	u32 button;
	u32 last_button;
	u32 button_time_delta;
	u32 button_time_last;
	
	// Trigger __MICTimeoutCallback if an EXI transfer doesn't complete in time
	struct timespec timeout;
} static __MICBlock[2];


extern int clock_gettime(struct timespec *tp);
static syswd_t __alarm;
static syswd_t __timeout[2];

static BOOL __init = FALSE;

#define secs_to_nanosecs_f(sec) \
	((sec) * 1000000000.)
#define CalcTimeout(b, h) \
	(u64)(secs_to_nanosecs_f((1. / h) * (b / sizeof(s16))))

static const u32 __MICTimeoutTable[] = {
	CalcTimeout(32, 11025), CalcTimeout(64, 11025), CalcTimeout(128, 11025), CalcTimeout(128, 11025),
	CalcTimeout(32, 22050), CalcTimeout(64, 22050), CalcTimeout(128, 22050), CalcTimeout(128, 22050),
	CalcTimeout(32, 44100), CalcTimeout(64, 44100), CalcTimeout(128, 44100), CalcTimeout(128, 44100),
	CalcTimeout(32, 44100), CalcTimeout(64, 44100), CalcTimeout(128, 44100), CalcTimeout(128, 44100),
};


s32 __MICDoMount(s32 chan);
void __MICDoUnmount(s32 chan, s32 result);
s32 __MICUnlockedCallback(s32 chan, s32 dev);
void __MICMountCallback(s32 chan, s32 result);
void __MICSetCallback(s32 chan, s32 result);
void __MICResetCallback(s32 chan, s32 result);
s32 __MICSync(s32 chan);
void __MICSyncCallback(s32 chan, s32 result);
s32 __MICExtHandler(s32 chan, s32 dev);
s32 __MICExiHandler(s32 chan, s32 dev);
s32 __MICTxHandler(s32 chan, s32 dev);
void __MICAlarmCallback(syswd_t alarm, void *cb_arg);
void __MICTimeoutCallback(syswd_t alarm, void *cb_arg);
s32 __MICRawReset(s32 chan);
s32 __MICRawReadStatus(s32 chan, u32 *status);
s32 __MICRawWriteStatus(s32 chan, u32 status);
s32 __MICRawReadDataAsync(s32 chan, s16 *data, u32 len, EXICallback DMACompletion);
s32 __MICGetControlBlock(s32 chan, BOOL skip_active_check, struct MICControlBlock **micblock);
void __MICPutControlBlock(struct MICControlBlock *micblock, s32 result);
BOOL __MICUpdateStatus(s32 chan, u32 status, BOOL dunno);
void __MICUpdateButton(s32 chan);


s32 __MICDoMount(s32 chan)
{
	s32 result;
	struct MICControlBlock *cb = &__MICBlock[chan];
	
	u32 level = IRQ_Disable();
	
	cb->is_attached = TRUE;
	
	u32 exiID;
	if (!EXI_GetID(chan, EXI_DEVICE_0, &exiID))
	{
		result = MIC_RESULT_NOCARD;
	}
	else if (exiID != MIC_EXI_ID)
	{
		result = MIC_RESULT_WRONGDEVICE;
	}
	else
	{
		result = MIC_RESULT_READY;
	}
	
	u32 status;
	
	if ((result >= MIC_RESULT_READY) &&
		((result = __MICRawReset(chan)) >= MIC_RESULT_READY) &&
		((result = __MICRawReadStatus(chan, &status)) >= MIC_RESULT_READY))
	{
		__MICUpdateStatus(chan, status, FALSE);
		
		cb->button = 0;
		cb->last_button = 0;
		cb->button_time_delta = 0;
		cb->button_time_last = gettick();
	
		if (EXI_Probe(chan))
		{
			EXI_RegisterEXICallback(chan, __MICExiHandler);
			EXI_Unlock(chan);
			IRQ_Restore(level);
			
			if (cb->attach_callback)
			{
				MICCallback attach = cb->attach_callback;
				cb->attach_callback = NULL;
				attach(chan, result);
			}
			__MICPutControlBlock(cb, result);
			return MIC_RESULT_READY;
		}
		else
			result = MIC_RESULT_NOCARD;
	}
	
	EXI_Unlock(chan);
	__MICDoUnmount(chan, result);
	IRQ_Restore(level);
	return result;
}

void __MICDoUnmount(s32 chan, s32 result)
{
	struct MICControlBlock *cb = &__MICBlock[chan];
	
	u32 level = IRQ_Disable();
	
	if (cb->is_attached)
	{
		EXI_RegisterEXICallback(chan, NULL);
		EXI_Detach(chan);
		
		cb->result_code = result;
		cb->is_attached = FALSE;
		cb->error_count = 0;
	}
	else
	{
		cb->result_code = MIC_RESULT_NOCARD;
	}
	
	IRQ_Restore(level);
	
	if (cb->attach_callback)
	{
		MICCallback unmount = cb->attach_callback;
		cb->attach_callback = NULL;
		unmount(chan, result);
	}
}

s32 __MICUnlockedCallback(s32 chan, s32 dev)
{
	struct MICControlBlock *cb = &__MICBlock[chan];
	
	MICCallback unlocked = cb->mount_callback;
	if (cb->mount_callback)
	{
		cb->mount_callback = NULL;
		unlocked(chan, EXI_Probe(chan) ? MIC_RESULT_NOCARD : MIC_RESULT_UNLOCKED);
	}
	
	// This is used as lckd->unlockcb, which does not have a checked return value
	return 0;
}

void __MICMountCallback(s32 chan, s32 result)
{
	struct MICControlBlock *cb = &__MICBlock[chan];
	
	if (result == MIC_RESULT_UNLOCKED)
	{
		cb->mount_callback = __MICMountCallback;
		if (EXI_Lock(chan, EXI_DEVICE_0, __MICUnlockedCallback))
		{
			cb->mount_callback = NULL;
			__MICDoMount(chan);
		}
	}
	else if (result == MIC_RESULT_NOCARD)
	{
		__MICDoUnmount(chan, result);
	}
}

void __MICSetCallback(s32 chan, s32 result)
{
	u32 status = __MICBlock[chan].status;
	
	if (__MICRawWriteStatus(chan, status) >= MIC_RESULT_READY)
		__MICUpdateStatus(chan, status, FALSE);
}

void __MICResetCallback(s32 chan, s32 result)
{
	u32 status;
	
	if (__MICRawReset(chan) >= MIC_RESULT_READY)
		if (__MICRawReadStatus(chan, &status) >= MIC_RESULT_READY)
			__MICUpdateStatus(chan, status, FALSE);
}

s32 __MICSync(s32 chan)
{
	struct MICControlBlock *cb = &__MICBlock[chan];
	u32 level = IRQ_Disable();
	
	while (cb->result_code == MIC_RESULT_BUSY)
	{
		LWP_ThreadSleep(cb->thread_queue);
	}
	
	IRQ_Restore(level);
	return cb->result_code;
}

void __MICSyncCallback(s32 chan, s32 result)
{
	LWP_ThreadBroadcast(__MICBlock[chan].thread_queue);
}

s32 __MICExtHandler(s32 chan, s32 dev)
{
	struct MICControlBlock *cb = &__MICBlock[chan];
	
	if (cb->is_attached)
	{
		EXI_RegisterEXICallback(chan, NULL);
		cb->result_code = MIC_RESULT_NOCARD;
		cb->is_attached = FALSE;
		cb->is_active = FALSE;
		cb->error_count = 0;
		
		MICCallback attach = cb->attach_callback;
		if (attach)
		{
			cb->attach_callback = NULL;
			attach(chan, MIC_RESULT_NOCARD);
		}
		
		MICCallback detach = cb->detach_callback;
		if (detach)
		{
			cb->detach_callback = NULL;
			detach(chan, MIC_RESULT_NOCARD);
		}
	}
	
	// This is used as exi->CallbackEXT, which does not have checked return value
	return 0;
}

s32 __MICExiHandler(s32 chan, s32 dev)
{
	SYS_CancelAlarm(__timeout[chan]);
	
	struct MICControlBlock *cb = &__MICBlock[chan];
	
	if (cb->is_attached && cb->is_active)
	{
		s32 result_code = MIC_RESULT_FATAL_ERROR;
		
		if (EXI_Lock(chan, EXI_DEVICE_0, NULL))
		{
			u32 status;
			if (((result_code = __MICRawReadStatus(chan, &status)) >= MIC_RESULT_READY) &&
				__MICUpdateStatus(chan, status, TRUE))
			{
				if (status & MIC_STATUS_BUFOVRFLW)
				{
					cb->error_count++;
					status &= ~MIC_STATUS_BUFOVRFLW;
				}
				
				__MICUpdateButton(chan);
				
				if (cb->set_callback)
					cb->set_callback(chan, MIC_RESULT_READY);
				
				if (cb->is_active)
				{
					result_code = __MICRawReadDataAsync(chan,
						cb->buff_ring_base + cb->buff_ring_cur / sizeof(s16),
						cb->hw_buff_size, __MICTxHandler);
					
					if (result_code >= MIC_RESULT_READY)
					{
						goto skip_unlock;
					}
				}
				else if (!EXI_Probe(chan))
				{
					result_code = MIC_RESULT_NOCARD;
				}
			}
			
			EXI_Unlock(chan);
		}
	skip_unlock:
		
		if (cb->exi_callback)
		{
			cb->exi_callback(chan, result_code);
		}
		
		if (cb->set_callback)
		{
			cb->set_callback = NULL;
			
			if (cb->attach_callback)
			{
				cb->attach_callback(chan, result_code);
				cb->attach_callback = NULL;
			}
			
			__MICPutControlBlock(cb, result_code);
		}
	}
	
	// This is used as exi->CallbackEXI, which does not have checked return value
	return 0;
}

s32 __MICTxHandler(s32 chan, s32 dev)
{
	s32 result_code = MIC_RESULT_NOCARD;
	struct MICControlBlock *cb = &__MICBlock[chan];
	
	cb->buff_ring_cur += cb->hw_buff_size;
	
	if (cb->buff_ring_cur >= cb->buff_ring_size)
		cb->buff_ring_cur = 0;
	
	if (EXI_Deselect(chan))
	{
		u32 status;
		if (((result_code = __MICRawReadStatus(chan, &status)) >= MIC_RESULT_READY) &&
			 __MICUpdateStatus(chan, status, TRUE))
		{
			if (status & MIC_STATUS_BUFOVRFLW)
			{
				cb->error_count++;
				status &= ~MIC_STATUS_BUFOVRFLW;
			}
			
			__MICUpdateButton(chan);
			
			if (EXI_Probe(chan))
			{
				SYS_SetAlarm(__timeout[chan], &cb->timeout, __MICTimeoutCallback, NULL);
				result_code = MIC_RESULT_READY;
			}
		}
	}
	
	EXI_Unlock(chan);
	
	if (cb->tx_callback)
		cb->tx_callback(chan, result_code);
	
	// This is used as exi->CallbackTC, which does not have checked return value
	return 0;
}

void __MICAlarmCallback(syswd_t alarm, void *cb_arg)
{
	int chan;
	for (chan = 0; chan < 2; chan++)
	{
		struct MICControlBlock *cb = &__MICBlock[chan];
		
		if (cb->is_attached && !cb->is_active &&
			EXI_Lock(chan, EXI_DEVICE_0, NULL))
		{
			u32 status;
			s32 result;
			
			if ((result = __MICRawReadStatus(chan, &status)) >= MIC_RESULT_READY &&
				__MICUpdateStatus(chan, status, TRUE))
			{
				__MICUpdateButton(chan);
				
				if (cb->set_callback)
					cb->set_callback(chan, result);
			}
			
			EXI_Unlock(chan);
			
			if (cb->set_callback)
			{
				cb->set_callback = NULL;
				
				if (cb->attach_callback)
				{
					cb->attach_callback(chan, result);
					cb->attach_callback = NULL;
				}
				
				__MICPutControlBlock(cb, result);
			}
		}
	}
}

void __MICTimeoutCallback(syswd_t alarm, void *cb_arg)
{
	s32 chan = (alarm == __timeout[0]) ? 0 : 1;
	struct MICControlBlock *cb = &__MICBlock[chan];
	
	if (cb->is_attached)
	{
		if (EXI_Lock(chan, EXI_DEVICE_0, NULL))
		{
			cb->is_active = FALSE;
			__MICRawReset(chan);
			u32 status;
			__MICRawReadStatus(chan, &status);
			__MICUpdateStatus(chan, status, FALSE);
			EXI_Unlock(chan);
		}
		else
		{
			SYS_SetAlarm(__timeout[chan], &cb->timeout, __MICTimeoutCallback, NULL);
		}
	}
}

s32 __MICRawReset(s32 chan)
{
	s32 result = MIC_RESULT_NOCARD;
	u32 level = IRQ_Disable();
	
	if (EXI_Select(chan, EXI_DEVICE_0, EXI_SPEED16MHZ))
	{
		BOOL exi_fail = FALSE;
		
		u16 cmd = MIC_RESET << 8;
		exi_fail |= !EXI_Imm(chan, &cmd, 1, EXI_WRITE, NULL);
		exi_fail |= !EXI_Sync(chan);
		
		exi_fail |= !EXI_Deselect(chan);
		
		if (!exi_fail)
			result = MIC_RESULT_READY;
	}
	
	IRQ_Restore(level);
	return result;
}

s32 __MICRawReadStatus(s32 chan, u32 *status)
{
	s32 result = MIC_RESULT_NOCARD;
	u32 level = IRQ_Disable();
	
	if (EXI_Select(chan, EXI_DEVICE_0, EXI_SPEED16MHZ))
	{
		BOOL exi_fail = FALSE;
		
		u16 data = MIC_READ_STATUS << 8;
		exi_fail |= !EXI_Imm(chan, &data, 1, EXI_WRITE, NULL);
		exi_fail |= !EXI_Sync(chan);
		
		exi_fail |= !EXI_Imm(chan, &data, 2, EXI_READ, NULL);
		exi_fail |= !EXI_Sync(chan);
		
		exi_fail |= !EXI_Deselect(chan);
		
		*status = data;
		
		if (!exi_fail)
			result = MIC_RESULT_READY;
	}
	
	IRQ_Restore(level);
	return result;
}

s32 __MICRawWriteStatus(s32 chan, u32 status)
{
	s32 result = MIC_RESULT_NOCARD;
	u32 level = IRQ_Disable();
	
	if (EXI_Select(chan, EXI_DEVICE_0, EXI_SPEED16MHZ))
	{
		BOOL exi_fail = FALSE;
		
		u32 data = (MIC_WRITE_STATUS << 24) | ((status & 0xffff) << 8);
		exi_fail |= !EXI_Imm(chan, &data, 3, EXI_WRITE, NULL);
		exi_fail |= !EXI_Sync(chan);
		
		exi_fail |= !EXI_Deselect(chan);
		
		if (!exi_fail)
			result = MIC_RESULT_READY;
	}
	
	IRQ_Restore(level);
	return result;
}

s32 __MICRawReadDataAsync(s32 chan, s16 *data, u32 len, EXICallback DMACompletion)
{
	s32 result = MIC_RESULT_NOCARD;
	
	DCInvalidateRange(data, len);
	
	u32 level = IRQ_Disable();
	
	if (EXI_Select(chan, EXI_DEVICE_0, EXI_SPEED16MHZ))
	{
		BOOL exi_fail = FALSE;
		
		u16 cmd = MIC_DMA_DATA << 8;
		exi_fail |= !EXI_Imm(chan, &cmd, 1, EXI_WRITE, NULL);
		exi_fail |= !EXI_Sync(chan);
		
		exi_fail |= !EXI_Dma(chan, data, len, EXI_READ, DMACompletion);
		
		if (!exi_fail)
			result = MIC_RESULT_READY;
		else
			EXI_Deselect(chan);
	}
	
	IRQ_Restore(level);
	return result;
}

s32 __MICGetControlBlock(s32 chan, BOOL skip_active_check, struct MICControlBlock **micblock)
{
	s32 result = MIC_RESULT_NOCARD;
	struct MICControlBlock *cb = &__MICBlock[chan];
	
	u32 level = IRQ_Disable();
	
	if (cb->is_attached)
	{
		if (skip_active_check || !cb->is_active)
		{
			if (cb->result_code != MIC_RESULT_BUSY)
			{
				cb->result_code = MIC_RESULT_BUSY;
				result = MIC_RESULT_READY;
				cb->attach_callback = NULL;
				*micblock = cb;
			}
			else
				result = MIC_RESULT_BUSY;
		}
		else
			result = MIC_RESULT_INVALID_STATE;
	}
	
	IRQ_Restore(level);
	return result;
}

void __MICPutControlBlock(struct MICControlBlock *micblock, s32 result)
{
	u32 level = IRQ_Disable();
	
	if (micblock->is_attached ||
		(micblock->result_code == MIC_RESULT_BUSY))
	{
		micblock->result_code = result;
	}
	
	IRQ_Restore(level);
}

BOOL __MICUpdateStatus(s32 chan, u32 status, BOOL dunno)
{
	struct MICControlBlock *cb = &__MICBlock[chan];
	
	u32 level = IRQ_Disable();
	
	switch ((status >> 13) & 3)
	{
	case 0:
		cb->hw_buff_size = 32;
		break;
	case 1:
		cb->hw_buff_size = 64;
		break;
	default:
		cb->hw_buff_size = 128;
		break;
	}
	
	switch ((status >> 11) & 3)
	{
	case 0:
		cb->sample_rate = 11025;
		break;
	case 1:
		cb->sample_rate = 22050;
		break;
	default:
		cb->sample_rate = 44100;
		break;
	}
	
	cb->gain = (status & MIC_STATUS_GAIN15) ? 15 : 0;
	
	cb->buff_ring_size = cb->hw_buff_size * (cb->buff_size / cb->hw_buff_size);
	
	if (status & MIC_STATUS_ACTIVE)
	{
		if (!cb->is_active)
		{
			SYS_SetAlarm(__timeout[chan], &cb->timeout, __MICTimeoutCallback, NULL);
		}
		cb->is_active = TRUE;
	}
	else
		cb->is_active = FALSE;
	
	if (dunno && ((cb->last_status ^ status) & 0xfc0f)) // checks all bits except buff_ovrflw and buttons
	{
		cb->is_active = FALSE;
		__MICRawReset(chan);
		__MICRawReadStatus(chan, &status);
		__MICUpdateStatus(chan, status, FALSE);
		IRQ_Restore(level);
		return FALSE;
	}
	else
	{
		cb->last_status = status;
		IRQ_Restore(level);
		return TRUE;
	}
}

void __MICUpdateButton(s32 chan)
{
	struct MICControlBlock *cb = &__MICBlock[chan];
	u32 level = IRQ_Disable();
	
	u32 ticks = gettick();
	u32 delta = (ticks - cb->button_time_last) + cb->button_time_delta;
	
	if (delta >= 10000)
	{
		u32 button_bits = (cb->last_status >> 4) & 0x1f;
		button_bits &= ~1; // Mask off the "DeviceID" bit (always 0)
		
		u32 button_changed = cb->button ^ cb->last_button;
		u32 new_button = (button_bits & ~button_changed) | (cb->button & button_changed);
		
		cb->button = new_button;
		cb->last_button = button_bits;
		
		delta = 0;
	}
	
	cb->button_time_last = ticks;
	cb->button_time_delta = delta;
	
	IRQ_Restore(level);
}

void MICInit(void)
{
	if (__init == FALSE)
	{
		int i;
		for (i = 0; i < 2; i++)
		{
			__MICBlock[i].result_code = MIC_RESULT_NOCARD;
			__MICBlock[i].is_attached = FALSE;
			__MICBlock[i].is_active = FALSE;
			__MICBlock[i].exi_callback = NULL;
			__MICBlock[i].tx_callback = NULL;
			__MICBlock[i].detach_callback = NULL;
			__MICBlock[i].attach_callback = NULL;
			__MICBlock[i].mount_callback = NULL;
			__MICBlock[i].set_callback = NULL;
			__MICBlock[i].error_count = 0;
			LWP_InitQueue(&__MICBlock[i].thread_queue);
		}
		
		SYS_CreateAlarm(&__alarm);
		
		struct timespec period;
		period.tv_sec = 0;
		period.tv_nsec = 5 * TB_NSPERMS;
		SYS_SetPeriodicAlarm(__alarm, &period, &period, __MICAlarmCallback, NULL);
		
		SYS_CreateAlarm(&__timeout[0]);
		SYS_CreateAlarm(&__timeout[1]);
		
		__init = TRUE;
	}
}

s32 MICProbeEx(s32 chan)
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (chan >= 0 && chan <= 1)
	{
		s32 irq = IRQ_Disable();
		
		s32 probe = EXI_ProbeEx(chan);
		if (probe == -1)
		{
			result = MIC_RESULT_NOCARD;
			goto restoreInterrupts;
		}
		else if (probe == 0)
		{
			result = MIC_RESULT_BUSY;
			goto restoreInterrupts;
		}
		
		if (__MICBlock[chan].is_attached)
		{
			result = MIC_RESULT_READY;
			goto restoreInterrupts;
		}
		
		if (EXI_GetState(chan) & EXI_FLAG_ATTACH)
		{
			result = MIC_RESULT_WRONGDEVICE;
			goto restoreInterrupts;
		}
		
		u32 exiID;
		EXI_GetID(chan, EXI_DEVICE_0, &exiID);
		if (exiID == 0)
		{
			result = MIC_RESULT_BUSY;
			goto restoreInterrupts;
		}
		else if (exiID != MIC_EXI_ID)
		{
			result = MIC_RESULT_WRONGDEVICE;
			goto restoreInterrupts;
		}
		else
		{
			result = MIC_RESULT_READY;
		}
		
	restoreInterrupts:
		IRQ_Restore(irq);
	}
	
	return result;
}

s32 MICGetResultCode(s32 chan)
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init &&
		chan >= 0 && chan <= 1)
	{
		result = __MICBlock[chan].result_code;
	}
	
	return result;
}

u32 MICGetErrorCount(s32 chan)
{
	s32 result = 0;
	
	if (__init &&
		chan >= 0 && chan <= 1)
	{
		result = __MICBlock[chan].error_count;
	}
	
	return result;
}


s32 MICMountAsync(s32 chan, s16* buffer, s32 size, MICCallback detachCallback, MICCallback attachCallback)
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init &&
		chan >= 0 && chan <= 1)
	{
		struct MICControlBlock *cb = &__MICBlock[chan];
		u32 level = IRQ_Disable();
		
		if (cb->result_code == MIC_RESULT_BUSY)
		{
			IRQ_Restore(level);
			return MIC_RESULT_BUSY;
		}
		
		if (!cb->is_attached)
		{
			if ((EXI_GetState(chan) & EXI_FLAG_ATTACH) == 0)
			{
				if (cb->is_attached || EXI_Attach(chan, __MICExtHandler))
				{
					cb->result_code = MIC_RESULT_BUSY;
					cb->is_active = FALSE;
					cb->exi_callback = NULL;
					cb->tx_callback = NULL;
					cb->detach_callback = detachCallback;
					cb->attach_callback = attachCallback;
					cb->mount_callback = NULL;
					cb->set_callback = NULL;
					cb->buff_ring_base = (s16*)(((u32)buffer + 31) & ~31);
					cb->buff_size = size - (cb->buff_ring_base - buffer);
					cb->buff_ring_cur = 0;

					EXI_RegisterEXICallback(chan, NULL);
					IRQ_Restore(level);

					cb->mount_callback = __MICMountCallback;

					if (EXI_Lock(chan, EXI_DEVICE_0, __MICUnlockedCallback))
					{
						cb->mount_callback = NULL;
						return __MICDoMount(chan);
					}

					return MIC_RESULT_READY;
				}
				else
					result = MIC_RESULT_NOCARD;
			}
			else
				result = MIC_RESULT_WRONGDEVICE;
		}
		else
			result = MIC_RESULT_INVALID_STATE;
		
		cb->result_code = result;
		IRQ_Restore(level);
	}
	
	return result;
}

s32 MICMount(s32 chan, s16* buffer, s32 size, MICCallback detachCallback)
{
	s32 result = MICMountAsync(chan, buffer, size, detachCallback, __MICSyncCallback);
	if (result >= MIC_RESULT_READY)
		return __MICSync(chan);
	else
		return result;
}

s32 MICUnmount(s32 chan)
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init &&
		chan >= 0 && chan <= 1)
	{
		struct MICControlBlock *cb = NULL;
		if ((result = __MICGetControlBlock(chan, FALSE, &cb)) >= MIC_RESULT_READY)
		{
			__MICDoUnmount(chan, MIC_RESULT_NOCARD);
			result = MIC_RESULT_READY;
		}
	}
	
	return result;
}

s32 MICGetRingbuffsize(s32 chan, s32* size)
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init == TRUE &&
		chan >= 0 && chan <= 1 &&
		size != NULL)
	{
		struct MICControlBlock *cb = &__MICBlock[chan];
		
		u32 level = IRQ_Disable();
		
		if (cb->is_attached)
		{
			*size = cb->buff_ring_size;
			result = MIC_RESULT_READY;
		}
		else
			result = MIC_RESULT_NOCARD;
			
		IRQ_Restore(level);
	}
	
	return result;
}


s32 MICSetStatusAsync(s32 chan, u32 status, MICCallback setCallback)
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init &&
		chan >= 0 && chan <= 1)
	{
		u32 level = IRQ_Disable();
		
		struct MICControlBlock *cb = NULL;
		if ((result = __MICGetControlBlock(chan, FALSE, &cb)) >= MIC_RESULT_READY)
		{
			cb->status = status;
			cb->attach_callback = setCallback;
			cb->set_callback = __MICSetCallback;
			result = MIC_RESULT_READY;
		}
		
		IRQ_Restore(level);
	}
	
	return result;
}

s32 MICSetStatus(s32 chan, u32 status)
{
	s32 result = MICSetStatusAsync(chan, status, __MICSyncCallback);
	if (result >= MIC_RESULT_READY)
		return __MICSync(chan);
	else
		return result;
}

s32 MICGetStatus(s32 chan, u32* status)
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init &&
		chan >= 0 && chan <= 1 &&
		status != NULL)
	{
		struct MICControlBlock *cb = &__MICBlock[chan];
		u32 level = IRQ_Disable();
		if (cb->is_attached)
		{
			*status = cb->last_status;
			result = MIC_RESULT_READY;
		}
		else
			result = MIC_RESULT_NOCARD;
		IRQ_Restore(level);
	}
	
	return result;
}


s32 MICSetParamsAsync(s32 chan, s32 size, s32 rate, s32 gain, MICCallback setCallback)
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init &&
		chan >= 0 && chan <= 1 &&
		(size == 32 || size == 64 || size == 128) &&
		(rate == 11025 || rate == 22050 || rate == 44100) &&
		(gain == 0 || gain == 15))
	{
		u32 level = IRQ_Disable();
		
		struct MICControlBlock *cb = NULL;
		if ((result = __MICGetControlBlock(chan, FALSE, &cb)) >= MIC_RESULT_READY)
		{
			u32 status = cb->last_status;
			
			status &= ~(MIC_STATUS_64BYTES | MIC_STATUS_128BYTES);
			if (size == 64)
				status |= MIC_STATUS_64BYTES;
			else if (size == 128)
				status |= MIC_STATUS_128BYTES;
			
			status &= ~(MIC_STATUS_22050Hz | MIC_STATUS_44100Hz);
			if (rate == 22050)
				status |= MIC_STATUS_22050Hz;
			else if (rate == 44100)
				status |= MIC_STATUS_44100Hz;
			
			status &= ~MIC_STATUS_GAIN15;
			if (gain == 15)
				status |= MIC_STATUS_GAIN15;
			
			cb->status = status;
			
			cb->attach_callback = setCallback;
			cb->set_callback = __MICSetCallback;
			result = MIC_RESULT_READY;
		}
		
		IRQ_Restore(level);
	}
	
	return result;
}

s32 MICSetParams(s32 chan, s32 size, s32 rate, s32 gain)
{
	s32 result = MICSetParamsAsync(chan, size, rate, gain, __MICSyncCallback);
	if (result >= MIC_RESULT_READY)
		return __MICSync(chan);
	else
		return result;
}

s32 MICGetParams(s32 chan, s32* size, s32* rate, s32* gain)
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init &&
		chan >= 0 && chan <= 1 &&
		size != NULL &&
		rate != NULL &&
		gain != NULL)
	{
		struct MICControlBlock *cb = &__MICBlock[chan];
		u32 level = IRQ_Disable();
		if (cb->is_attached)
		{
			*size = cb->hw_buff_size;
			*rate = cb->sample_rate;
			*gain = cb->gain;
			result = MIC_RESULT_READY;
		}
		else
			result = MIC_RESULT_NOCARD;
		IRQ_Restore(level);
	}
	
	return result;
}


s32 MICSetBuffsizeAsync(s32 chan, s32 size, MICCallback setCallback)
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init &&
		chan >= 0 && chan <= 1 &&
		(size == 32 || size == 64 || size == 128))
	{
		u32 level = IRQ_Disable();
		
		struct MICControlBlock *cb = NULL;
		if ((result = __MICGetControlBlock(chan, FALSE, &cb)) >= MIC_RESULT_READY)
		{
			cb->status = cb->last_status & ~(MIC_STATUS_64BYTES | MIC_STATUS_128BYTES);
			if (size == 64)
				cb->status |= MIC_STATUS_64BYTES;
			else if (size == 128)
				cb->status |= MIC_STATUS_128BYTES;
			cb->attach_callback = setCallback;
			cb->set_callback = __MICSetCallback;
			result = MIC_RESULT_READY;
		}
		
		IRQ_Restore(level);
	}
	
	return result;
}

s32 MICSetBuffsize(s32 chan, s32 size)
{
	s32 result = MICSetBuffsizeAsync(chan, size, __MICSyncCallback);
	if (result >= MIC_RESULT_READY)
		return __MICSync(chan);
	else
		return result;
}

s32 MICGetBuffsize(s32 chan, s32* size)
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init &&
		chan >= 0 && chan <= 1 &&
		size != NULL)
	{
		struct MICControlBlock *cb = &__MICBlock[chan];
		u32 level = IRQ_Disable();
		if (cb->is_attached)
		{
			*size = cb->hw_buff_size;
			result = MIC_RESULT_READY;
		}
		else
			result = MIC_RESULT_NOCARD;
		IRQ_Restore(level);
	}
	
	return result;
}


s32 MICSetRateAsync(s32 chan, s32 rate, MICCallback setCallback)
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init &&
		chan >= 0 && chan <= 1 &&
		(rate == 11025 || rate == 22050 || rate == 44100))
	{
		u32 level = IRQ_Disable();
		
		struct MICControlBlock *cb = NULL;
		if ((result = __MICGetControlBlock(chan, FALSE, &cb)) >= MIC_RESULT_READY)
		{
			cb->status = cb->last_status & ~(MIC_STATUS_22050Hz | MIC_STATUS_44100Hz);
			if (rate == 22050)
				cb->status |= MIC_STATUS_22050Hz;
			else if (rate == 44100)
				cb->status |= MIC_STATUS_44100Hz;
			cb->attach_callback = setCallback;
			cb->set_callback = __MICSetCallback;
			result = MIC_RESULT_READY;
		}
		
		IRQ_Restore(level);
	}
	
	return result;
}

s32 MICSetRate(s32 chan, s32 rate)
{
	s32 result = MICSetRateAsync(chan, rate, __MICSyncCallback);
	if (result >= MIC_RESULT_READY)
		return __MICSync(chan);
	else
		return result;
}

s32 MICGetRate(s32 chan, s32* rate)
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init &&
		chan >= 0 && chan <= 1 &&
		rate != NULL)
	{
		struct MICControlBlock *cb = &__MICBlock[chan];
		u32 level = IRQ_Disable();
		if (cb->is_attached)
		{
			*rate = cb->sample_rate;
			result = MIC_RESULT_READY;
		}
		else
			result = MIC_RESULT_NOCARD;
		IRQ_Restore(level);
	}
	
	return result;
}


s32 MICSetGainAsync(s32 chan, s32 gain, MICCallback setCallback)
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init &&
		chan >= 0 && chan <= 1 &&
		(gain == 0 || gain == 15))
	{
		u32 level = IRQ_Disable();
		
		struct MICControlBlock *cb = NULL;
		if ((result = __MICGetControlBlock(chan, FALSE, &cb)) >= MIC_RESULT_READY)
		{
			cb->status = (cb->last_status & ~MIC_STATUS_GAIN15) | ((gain == 15) ? MIC_STATUS_GAIN15 : MIC_STATUS_GAIN0);
			cb->attach_callback = setCallback;
			cb->set_callback = __MICSetCallback;
			result = MIC_RESULT_READY;
		}
		
		IRQ_Restore(level);
	}
	
	return result;
}

s32 MICSetGain(s32 chan, s32 gain)
{
	s32 result = MICSetGainAsync(chan, gain, __MICSyncCallback);
	if (result >= MIC_RESULT_READY)
		return __MICSync(chan);
	else
		return result;
}

s32 MICGetGain(s32 chan, s32* gain)
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init &&
		chan >= 0 && chan <= 1 &&
		gain != NULL)
	{
		struct MICControlBlock *cb = &__MICBlock[chan];
		u32 level = IRQ_Disable();
		if (cb->is_attached)
		{
			*gain = cb->gain;
			result = MIC_RESULT_READY;
		}
		else
			result = MIC_RESULT_NOCARD;
		IRQ_Restore(level);
	}
	
	return result;
}


s32 MICGetButton(s32 chan, u32* button)
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init &&
		chan >= 0 && chan <= 1 &&
		button != NULL)
	{
		struct MICControlBlock *cb = &__MICBlock[chan];
		u32 level = IRQ_Disable();
		if (cb->is_attached)
		{
			*button = cb->button;
			result = MIC_RESULT_READY;
		}
		else
			result = MIC_RESULT_NOCARD;
		IRQ_Restore(level);
	}
	
	return result;
}

s32 MICGetDeviceID(s32 chan, u32* id)
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init &&
		chan >= 0 && chan <= 1 &&
		id != NULL)
	{
		struct MICControlBlock *cb = &__MICBlock[chan];
		u32 level = IRQ_Disable();
		if (cb->is_attached)
		{
			*id = (cb->last_status & 0x10) >> 4;
			result = MIC_RESULT_READY;
		}
		else
			result = MIC_RESULT_NOCARD;
		IRQ_Restore(level);
	}
	
	return result;
}


s32 MICSetOutAsync(s32 chan, u32 pattern, MICCallback setCallback)
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init &&
		chan >= 0 && chan <= 1)
	{
		u32 level = IRQ_Disable();
		
		struct MICControlBlock *cb = NULL;
		if ((result = __MICGetControlBlock(chan, TRUE, &cb)) >= MIC_RESULT_READY)
		{
			cb->status = (cb->last_status & ~0xf) | (pattern & 0xf);
			cb->attach_callback = setCallback;
			cb->set_callback = __MICSetCallback;
			result = MIC_RESULT_READY;
		}
		
		IRQ_Restore(level);
	}
	
	return result;
}

s32 MICSetOut(s32 chan, u32 pattern)
{
	s32 result = MICSetOutAsync(chan, pattern, __MICSyncCallback);
	if (result >= MIC_RESULT_READY)
		return __MICSync(chan);
	else
		return result;
}

s32 MICGetOut(s32 chan, u32* pattern)
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init &&
		chan >= 0 && chan <= 1 &&
		pattern != NULL)
	{
		struct MICControlBlock *cb = &__MICBlock[chan];
		u32 level = IRQ_Disable();
		if (cb->is_attached)
		{
			*pattern = cb->last_status & 0xf;
			result = MIC_RESULT_READY;
		}
		else
			result = MIC_RESULT_NOCARD;
		IRQ_Restore(level);
	}
	
	return result;
}


s32 MICStartAsync(s32 chan, MICCallback startCallback)
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init &&
		chan >= 0 && chan <= 1)
	{
		u32 level = IRQ_Disable();
		
		struct MICControlBlock *cb = NULL;
		if ((result = __MICGetControlBlock(chan, FALSE, &cb)) >= MIC_RESULT_READY)
		{
			cb->status = cb->last_status | MIC_STATUS_ACTIVE;
			cb->attach_callback = startCallback;
			cb->set_callback = __MICSetCallback;
			cb->error_count = 0;
			cb->buff_ring_cur = 0;
			
			int rate = (cb->last_status >> 11) & 3;
			int size = (cb->last_status >> 13) & 3;
			cb->timeout.tv_sec = 0;
			cb->timeout.tv_nsec = __MICTimeoutTable[rate * 4 + size] * ticks_to_nanosecs(92);
			
			memset(cb->buff_ring_base, 0, cb->buff_ring_size);
			
			result = MIC_RESULT_READY;
		}
		
		IRQ_Restore(level);
	}
	
	return result;
}

s32 MICStart(s32 chan)
{
	s32 result = MICStartAsync(chan, __MICSyncCallback);
	if (result >= MIC_RESULT_READY)
		return __MICSync(chan);
	else
		return result;
}


s32 MICStopAsync(s32 chan, MICCallback stopCallback )
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init &&
		chan >= 0 && chan <= 1)
	{
		u32 level = IRQ_Disable();
		
		struct MICControlBlock *cb = NULL;
		if ((result = __MICGetControlBlock(chan, TRUE, &cb)) >= MIC_RESULT_READY)
		{
			cb->status = cb->last_status & ~MIC_STATUS_ACTIVE;
			cb->attach_callback = stopCallback;
			cb->set_callback = __MICSetCallback;
			result = MIC_RESULT_READY;
		}
		
		IRQ_Restore(level);
	}
	
	return result;
}

s32 MICStop(s32 chan)
{
	s32 result = MICStopAsync(chan, __MICSyncCallback);
	if (result >= MIC_RESULT_READY)
		return __MICSync(chan);
	else
		return result;
}


s32 MICResetAsync(s32 chan, MICCallback stopCallback )
{
	s32 result = MIC_RESULT_FATAL_ERROR;
	
	if (__init &&
		chan >= 0 && chan <= 1)
	{
		u32 level = IRQ_Disable();
		
		struct MICControlBlock *cb = NULL;
		if ((result = __MICGetControlBlock(chan, TRUE, &cb)) >= MIC_RESULT_READY)
		{
			cb->attach_callback = stopCallback;
			cb->set_callback = __MICSetCallback;
			result = MIC_RESULT_READY;
		}
		
		IRQ_Restore(level);
	}
	
	return result;
}

s32 MICReset(s32 chan)
{
	s32 result = MICResetAsync(chan, __MICSyncCallback);
	if (result >= MIC_RESULT_READY)
		return __MICSync(chan);
	else
		return result;
}


BOOL MICIsActive(s32 chan)
{
	if (__init &&
		chan >= 0 && chan <= 1)
		return __MICBlock[chan].is_active;
	else
		return FALSE;
}

BOOL MICIsAttached(s32 chan)
{
	if (__init &&
		chan >= 0 && chan <= 1)
		return __MICBlock[chan].is_attached;
	else
		return FALSE;
}


s32 MICGetCurrentTop(s32 chan)
{
	s32 result = MIC_RESULT_BUSY;
	
	if (__init &&
		chan >= 0 && chan <= 1)
	{
		struct MICControlBlock *cb = &__MICBlock[chan];
		u32 level = IRQ_Disable();
		if (cb->is_attached)
			result = cb->buff_ring_cur / sizeof(s16);
		IRQ_Restore(level);
	}
	
	return result;
}

s32 MICUpdateIndex(s32 chan, s32 index, s32 samples)
{
	s32 result = MIC_RESULT_BUSY;
	
	if (__init &&
		chan >= 0 && chan <= 1 &&
		index >= 0 &&
		samples >= 0)
	{
		struct MICControlBlock *cb = &__MICBlock[chan];
		u32 level = IRQ_Disable();
		if (cb->is_attached)
		{
			// This doesn't actually update anything
			// Weird, but that's how it is
			int samples_in_ring = cb->buff_ring_size / sizeof(s16);
			int index_req = index + samples;
			result = (index_req < samples_in_ring) ? index_req : index_req - samples_in_ring;
		}
		IRQ_Restore(level);
	}
	
	return result;
}

s32 MICGetSamplesLeft(s32 chan, s32 index)
{
	s32 result = MIC_RESULT_BUSY;
	
	if (__init &&
		chan >= 0 && chan <= 1 &&
		index >= 0)
	{
		struct MICControlBlock *cb = &__MICBlock[chan];
		u32 level = IRQ_Disable();
		if (cb->is_attached)
		{
			int samples_in_ring = cb->buff_ring_size / sizeof(s16);
			int index_cur = cb->buff_ring_cur / sizeof(s16);
			if (index_cur < index) // wrap
				result = samples_in_ring - (index - index_cur);
			else
				result = index_cur - index;
		}
		IRQ_Restore(level);
	}
	
	return result;
}

s32 MICGetSamples(s32 chan, s16* buffer, s32 index, s32 samples)
{
	s32 result = MIC_RESULT_BUSY;
	
	if (__init &&
		chan >= 0 && chan <= 1 &&
		buffer != NULL &&
		index >= 0 &&
		samples >= 0)
	{
		struct MICControlBlock *cb = &__MICBlock[chan];
		u32 level = IRQ_Disable();
		
		if (cb->is_attached)
		{
			const int samples_in_ring = cb->buff_ring_size / sizeof(s16);
			const int top = cb->buff_ring_cur / sizeof(s16);
			int s = index;
			s16 *src = &cb->buff_ring_base[s];
			
			for (; s < index + samples; s++)
			{
				if (s >= samples_in_ring)
					src = cb->buff_ring_base;
				
				if (s == top)
					break;
				
				*buffer++ = *src++;
			}
			
			result = s;
		}
		
		IRQ_Restore(level);
	}
	
	return result;
}


MICCallback MICSetExiCallback(s32 chan, MICCallback exiCallback)
{
	MICCallback result = NULL;
	
	if (__init &&
		chan >= 0 && chan <= 1)
	{
		struct MICControlBlock *cb = &__MICBlock[chan];
		u32 level = IRQ_Disable();
		if (cb->is_attached)
		{
			result = cb->exi_callback;
			cb->exi_callback = exiCallback;
		}
		IRQ_Restore(level);
	}
	
	return result;
}

MICCallback MICSetTxCallback(s32 chan, MICCallback txCallback)
{
	MICCallback result = NULL;
	
	if (__init &&
		chan >= 0 && chan <= 1)
	{
		struct MICControlBlock *cb = &__MICBlock[chan];
		u32 level = IRQ_Disable();
		if (cb->is_attached)
		{
			result = cb->tx_callback;
			cb->tx_callback = txCallback;
		}
		IRQ_Restore(level);
	}
	
	return result;
}
