// Copyright 2011-2026 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
#include "xua.h"

#define XASSERT_UNIT DECOUPLE
#include "xassert.h"

#include "debug_print.h"

#if XUA_USB_EN
#include <xs1.h>
#include "xc_ptr.h"
#include "interrupt.h"
#include "xua_commands.h"
#include "xud.h"
#include "xua_usb_params_funcs.h"

#ifdef NATIVE_DSD
#include "usbaudio20.h"             /* Defines from the USB Audio 2.0 Specifications */
#endif

#if (HID_CONTROLS)
#include "user_hid.h"
#endif

/* Volume and mute tables */
#if (OUT_VOLUME_IN_MIXER == 0) && (OUTPUT_VOLUME_CONTROL == 1)
unsigned int multOut[NUM_USB_CHAN_OUT + 1];
unsafe
{
    unsigned int volatile * unsafe multOutPtr = multOut;
}
#endif
#if (IN_VOLUME_IN_MIXER == 0) && (INPUT_VOLUME_CONTROL == 1)
unsigned int multIn[NUM_USB_CHAN_IN + 1];
unsafe
{
    unsigned int volatile * unsafe multInPtr = multIn;
}
#endif

/* Default to something sensible but the following are setup at stream start (unless UAC1 only..) */
#if (XUA_USB_BUS_SPEED == 2)
int g_numUsbChan_In = NUM_USB_CHAN_IN; /* Number of channels to/from the USB bus - initialised to HS for UAC2.0 */
int g_numUsbChan_Out = NUM_USB_CHAN_OUT;
int g_curSubSlot_Out = HS_STREAM_FORMAT_OUTPUT_1_SUBSLOT_BYTES;
int g_curSubSlot_In  = HS_STREAM_FORMAT_INPUT_1_SUBSLOT_BYTES;
int sampsToWrite = DEFAULT_FREQ/8000;  /* HS assumed here. Expect to be junked during a overflow before stream start */
int totalSampsToWrite = DEFAULT_FREQ/8000;
int g_maxPacketSize = MAX_DEVICE_AUD_PACKET_SIZE_IN_HS; /* IN packet size. Init to something sensible, but expect to be re-set before stream start */
#else
int g_numUsbChan_In = NUM_USB_CHAN_IN_FS; /* Number of channels to/from the USB bus - initialised to FS for UAC1.0 */
int g_numUsbChan_Out = NUM_USB_CHAN_OUT_FS;
int g_curSubSlot_Out = FS_STREAM_FORMAT_OUTPUT_1_SUBSLOT_BYTES;
int g_curSubSlot_In  = FS_STREAM_FORMAT_INPUT_1_SUBSLOT_BYTES;
int sampsToWrite = DEFAULT_FREQ/1000;  /* FS assumed here. Expect to be junked during a overflow before stream start */
int totalSampsToWrite = DEFAULT_FREQ/1000;
int g_maxPacketSize = MAX_DEVICE_AUD_PACKET_SIZE_IN_FS;  /* IN packet size. Init to something sensible, but expect to be re-set before stream start */
#endif

/* Circular audio buffers */
#if (XUD_USB_ISO_MAX_TXNS_PER_MICROFRAME > 1)
    #define REQD_BUF_SIZE_ERR_HANDLING_HIBW_OUT (3*(XUD_USB_ISO_EP_MAX_TXN_SIZE))
    #define OUT_BUF_EXTRA_SIZE  (XUA_MAX(MAX_DEVICE_AUD_PACKET_SIZE_OUT, REQD_BUF_SIZE_ERR_HANDLING_HIBW_OUT))
    unsigned outAudioBuff[(BUFF_SIZE_OUT >> 2)+ (OUT_BUF_EXTRA_SIZE >> 2)];
#else
    unsigned outAudioBuff[(BUFF_SIZE_OUT >> 2)+ (MAX_DEVICE_AUD_PACKET_SIZE_OUT >> 2)];
#endif

unsigned audioBuffIn[(BUFF_SIZE_IN >> 2)+ (MAX_DEVICE_AUD_PACKET_SIZE_IN >> 2)];

/* Shift down accounts for bytes -> words */
unsigned inZeroBuff[(MAX_DEVICE_AUD_PACKET_SIZE_IN >> 2)];

void GetADCCounts(unsigned samFreq, int &min, int &mid, int &max);

/* Globals for EP types */
XUD_ep aud_from_host_usb_ep = 0;
XUD_ep aud_to_host_usb_ep = 0;

/* Shared global audio buffering variables */
unsigned g_aud_from_host_buffer;
unsigned g_aud_to_host_flag = 0;
int buffer_aud_ctl_chan = 0;
unsigned g_aud_from_host_flag = 0;
unsigned g_aud_from_host_info;
unsigned g_streamChange_flag = XUA_AUDCTL_NO_COMMAND; /* Combined flag and command var used for signalling from host. Valid values are defined in commands.h */
unsigned g_streamChange_sampFreq = DEFAULT_FREQ;

/* Global vars for sharing stream format change between buffer and decouple (save a channel) */
unsigned g_formatChange_SubSlot;
unsigned g_formatChange_DataFormat;
unsigned g_formatChange_NumChans;
unsigned g_formatChange_SampRes;

int speedRem = 0;

xc_ptr aud_from_host_fifo_start;
xc_ptr aud_from_host_fifo_end;
xc_ptr g_aud_from_host_wrptr;
xc_ptr g_aud_from_host_rdptr;

xc_ptr aud_to_host_fifo_start;
xc_ptr aud_to_host_fifo_end;
xc_ptr g_aud_to_host_wrptr;
xc_ptr g_aud_to_host_dptr;
xc_ptr g_aud_to_host_rdptr;
int g_aud_to_host_fill_level;

int aud_data_remaining_to_device = 0;

#ifdef XSCOPE
#include <xscope.h>
#define XUA_PROBE_UNDERFLOW()  xscope_int(0, ++g_outUnderflowCount)
#define XUA_PROBE_OVERFLOW()   xscope_int(1, ++g_outOverflowCount)
#define XUA_PROBE_FILL(v)      xscope_int(2, (unsigned)(v))
#define XUA_PROBE_DRYOUT()     xscope_int(3, g_outDryoutCount)
#else
#define XUA_PROBE_UNDERFLOW()  (g_outUnderflowCount++)
#define XUA_PROBE_OVERFLOW()   (g_outOverflowCount++)
#define XUA_PROBE_FILL(v)      ((void)(v))
#define XUA_PROBE_DRYOUT()     ((void)0)
#endif

/* Audio over/under flow flags */
unsigned outUnderflow = 1;
unsigned outOverflow = 0;

/* Instrumentation: how often the OUT buffer has actually run dry or filled up.
 * The flags above are state, not events, so they cannot answer "how often".
 * xscope_int() writes to a hardware-buffered probe rather than doing I/O, so
 * it is safe in this real-time path in a way debug_printf would not be --
 * printing from here would perturb the very timing being measured. */
unsigned g_outUnderflowCount = 0;
unsigned g_outOverflowCount = 0;
unsigned g_fillProbeDiv = 0;

/* Runtime dry-outs of the OUT FIFO, distinct from g_outUnderflowCount: the
 * literal `outUnderflow = 1` sites are stream-reset points, but the FIFO can
 * also run dry mid-stream via the computed assignment in
 * handle_audio_request(). That path executes at interrupt level feeding I2S,
 * so it must not call xscope itself; it increments this counter and the
 * decoupler emits the value from its own context at the fill-probe cadence.
 * Single writer (the interrupt side), single reader -- no atomicity needed. */
unsigned g_outDryoutCount = 0;

/* UAC host-validator reductions. The device reduces; it never judges -- there
 * are no thresholds, no verdicts and no spec clauses anywhere in this file.
 * Field meanings are normative in docs/uac-validator-wire-format.md in the
 * evkb repo; the word indices below refer to that table.
 *
 * Single writer (this core) except where noted, single reader (the emission
 * at the fill cadence). */
unsigned g_uacvPktCount = 0;            /* word 2  */
unsigned g_uacvPktShortDiscarded = 0;   /* word 3  */
unsigned g_uacvPktNotMultiple = 0;      /* word 4  */
unsigned g_uacvHistSize[8] = {0,0,0,0,0,0,0,0};   /* words 20..27 */
unsigned g_uacvHistCount[8] = {0,0,0,0,0,0,0,0};  /* words 28..35 */
unsigned g_uacvHistOverflow = 0;        /* word 19 */

/* Byte-lane occupancy. or_acc says which bit positions were EVER set, and_acc
 * which were ALWAYS set. Together they identify subslot justification without
 * knowing anything about the audio content:
 *   (or_acc & 0xFF) == 0              -> low byte never used -> left-justified
 *   (or_acc >> 24) == (and_acc >> 24) -> top byte constant   -> right-justified
 *
 * nonsilent_frames is the vacuity witness. On a silent stream or_acc stays 0
 * and the justification test would pass for the WRONG REASON, declaring a
 * right-justifying host conformant. The judge refuses to rule without enough
 * non-silent frames. */
unsigned g_uacvOrAcc = 0;               /* word 5  */
unsigned g_uacvAndAcc = 0xFFFFFFFF;     /* word 6  */
unsigned g_uacvNonsilentFrames = 0;     /* word 7  */
unsigned g_uacvFrameNonzero = 0;        /* per-frame latch, not emitted */

unsigned inUnderflow = 1;

int aud_req_in_count = 0;
int aud_req_out_count = 0;

unsigned unpackState = 0;
unsigned unpackData = 0;

unsigned packState = 0;
unsigned packData = 0;

/* These flags are booleans which log whether or not an input or output stream is active
 * They can be used for passing to audio to allow shutdown when not streaming any audio */
unsigned g_output_stream_active = 0;
unsigned g_input_stream_active = 0;
unsigned g_any_stream_active_old = 0;
unsigned g_any_stream_active_current = 0;

/* Record one observed packet size. Eight slots is generous: a conformant host
 * uses at most two distinct sizes, the fractional-sample floor and ceiling. A
 * host needing more than eight is itself a finding, which is why the overflow
 * is counted rather than silently folded into a neighbouring slot -- the judge
 * refuses to rule on packet size at all while any packet went unaccounted. */
static inline void uacvRecordSize(unsigned size)
{
    for(int i = 0; i < 8; i++)
    {
        if(g_uacvHistSize[i] == size)
        {
            g_uacvHistCount[i]++;
            return;
        }
        if(g_uacvHistSize[i] == 0)
        {
            g_uacvHistSize[i] = size;
            g_uacvHistCount[i] = 1;
            return;
        }
    }
    g_uacvHistOverflow++;
}

static inline void _send_sample_4(chanend c_mix_out, int ch)
{
    int sample;
    read_via_xc_ptr(sample, g_aud_from_host_rdptr);
    g_aud_from_host_rdptr+=4;

    /* Validator: accumulate byte-lane occupancy before anything downstream can
     * normalise it away. This is the ONLY place the raw subslot bytes exist --
     * the 3-byte path masks with 0xffffff00, and UserBufferManagement runs on
     * the other tile, after unpacking. Two ALU ops per sample. */
    g_uacvOrAcc |= (unsigned)sample;
    g_uacvAndAcc &= (unsigned)sample;
    if(sample) g_uacvFrameNonzero = 1;

#if (OUTPUT_VOLUME_CONTROL == 1) && (!OUT_VOLUME_IN_MIXER)
    int mult;
    int h;
    unsigned l;
    unsafe
    {
        mult = multOutPtr[ch];
    }
    {h, l} = macs(mult, sample, 0, 0);
    h <<= 3;
#if (STREAM_FORMAT_OUTPUT_RESOLUTION_32BIT_USED == 1)
    h |= (l >>29) & 0x7; // Note: This step is not required if we assume sample depth is 24bit (rather than 32bit)
                            // Note: We need all 32bits for Native DSD
#endif
    outuint(c_mix_out, h);
#else
    outuint(c_mix_out, sample);
#endif
}

static inline void SendSamples4(chanend c_mix_out)
{
    /* Doing this allows us to unroll */
    if(g_numUsbChan_Out == HS_STREAM_FORMAT_OUTPUT_1_CHAN_COUNT)
    {
        #pragma loop unroll
        for(int i = 0; i < HS_STREAM_FORMAT_OUTPUT_1_CHAN_COUNT; i++)
        {
            _send_sample_4(c_mix_out, i);
        }
    }
    else if(g_numUsbChan_Out == HS_STREAM_FORMAT_OUTPUT_2_CHAN_COUNT)
    {
        #pragma loop unroll
        for(int i = 0; i < HS_STREAM_FORMAT_OUTPUT_2_CHAN_COUNT; i++)
        {
            _send_sample_4(c_mix_out, i);
        }
    }
    else if(g_numUsbChan_Out == HS_STREAM_FORMAT_OUTPUT_3_CHAN_COUNT)
    {
        #pragma loop unroll
        for(int i = 0; i < HS_STREAM_FORMAT_OUTPUT_3_CHAN_COUNT; i++)
        {
            _send_sample_4(c_mix_out, i);
        }
    }
    else
    {
        #pragma loop unroll
        for(int i = 0; i < NUM_USB_CHAN_OUT_FS; i++)
        {
            _send_sample_4(c_mix_out, i);
        }
    }
}


#pragma select handler
#pragma unsafe arrays
void handle_audio_request(chanend c_mix_out)
{
#if(defined XUA_USB_DESCRIPTOR_OVERWRITE_RATE_RES)
    g_curSubSlot_Out = get_usb_to_device_bit_res() >> 3;
    g_curSubSlot_In = get_device_to_usb_bit_res() >> 3;
#endif

    /* Input word that triggered interrupt and handshake back */
    unsigned underflowSample = inuint(c_mix_out);

    /* Validator: this function emits exactly one audio frame per call, so
     * latching at entry counts the frame the PREVIOUS call assembled. Counting
     * frames rather than samples is what makes the witness comparable across
     * channel counts -- and it is a witness, not a metric: the judge refuses to
     * rule on subslot justification until enough non-silent frames have gone
     * by, because a silent stream would pass that test for the wrong reason. */
    if(g_uacvFrameNonzero)
    {
        g_uacvNonsilentFrames++;
        g_uacvFrameNonzero = 0;
    }

    // OUT
#if (NUM_USB_CHAN_OUT == 0)
    outuint(c_mix_out, underflowSample);
#else
    int outSamps;
    if(outUnderflow)
    {
#pragma xta endpoint "out_underflow"
        /* We're still pre-buffering, send out 0 samps */
        for(int i = 0; i < NUM_USB_CHAN_OUT; i++)
        {
            outuint(c_mix_out, underflowSample);
        }

        /* Calc how many samples left in buffer */
        outSamps = g_aud_from_host_wrptr - g_aud_from_host_rdptr;
        if (outSamps < 0)
        {
            outSamps += BUFF_SIZE_OUT;
        }

        /* If we have a decent number of samples, come out of underflow cond */
        if(outSamps >= (OUT_BUFFER_PREFILL))
        {
            outUnderflow = 0;
            outSamps++;
        }
    }
    else
    {
        switch(g_curSubSlot_Out)
        {

            case 2:
#if (STREAM_FORMAT_OUTPUT_SUBSLOT_2_USED == 0)
__builtin_unreachable();
#endif
                /* Buffering not underflow condition send out some samples...*/
                for(int i = 0; i < g_numUsbChan_Out; i++)
                {
#pragma xta endpoint "mixer_request"
                    int sample;
                    int mult;
                    int h;
                    unsigned l;

                    read_short_via_xc_ptr(sample, g_aud_from_host_rdptr);
                    g_aud_from_host_rdptr+=2;
                    sample <<= 16;

                    /* Validator: same byte-lane accumulation as the 24-in-4
                     * path. A 2-byte subslot has no spare lane, so the judge
                     * SKIPs the justification rule here -- but the vacuity
                     * witness still has to be counted, or a UAC1 capture could
                     * not tell silence from signal either. */
                    g_uacvOrAcc |= (unsigned)sample;
                    g_uacvAndAcc &= (unsigned)sample;
                    if(sample) g_uacvFrameNonzero = 1;

#if (OUTPUT_VOLUME_CONTROL == 1) && (!OUT_VOLUME_IN_MIXER)
                    unsafe
                    {
                        mult = multOutPtr[i];
                    }
                    {h, l} = macs(mult, sample, 0, 0);
                    /* Note, in 2 byte subslot mode - ignore lower result of macs */
                    h <<= 3;
                    outuint(c_mix_out, h);
#else
                    outuint(c_mix_out, sample);
#endif
                }
                break;

            case 4:
#if (STREAM_FORMAT_OUTPUT_SUBSLOT_4_USED == 0)
__builtin_unreachable();
#endif
                /* Buffering not underflow condition send out some samples...*/
                SendSamples4(c_mix_out);
                break;

            case 3:
#if (STREAM_FORMAT_OUTPUT_SUBSLOT_3_USED == 0)
__builtin_unreachable();
#endif
                /* Note, in this case the unpacking of data is more of an overhead than the loop overhead
                 * so we do not currently make attempts to unroll */
                for(int i = 0; i < g_numUsbChan_Out; i++)
                {
                    int sample;
                    int mult;
                    int h;
                    unsigned l;

                    /* Unpack 3 byte samples */
                    switch (unpackState&0x3)
                    {
                        case 0:
                            read_via_xc_ptr(unpackData, g_aud_from_host_rdptr);
                            g_aud_from_host_rdptr+=4;
                            sample = unpackData << 8;
                            break;
                        case 1:
                            sample = (unpackData >> 16);
                            read_via_xc_ptr(unpackData, g_aud_from_host_rdptr);
                            g_aud_from_host_rdptr+=4;
                            sample = sample | (unpackData << 16);
                            break;
                        case 2:
                            sample = (unpackData >> 8);
                            read_via_xc_ptr(unpackData, g_aud_from_host_rdptr);
                            g_aud_from_host_rdptr+=4;
                            sample = sample | (unpackData<< 24);
                            break;
                        case 3:
                            sample = unpackData & 0xffffff00;
                            break;
                    }
                    unpackState++;

#if (OUTPUT_VOLUME_CONTROL == 1) && (!OUT_VOLUME_IN_MIXER)
                    unsafe
                    {
                        mult = multOutPtr[i];
                    }
                    {h, l} = macs(mult, sample, 0, 0);
                    h <<= 3;
                    outuint(c_mix_out, h);
#else
                    outuint(c_mix_out, sample);
#endif
                }
                break;

            default:
                __builtin_unreachable();
                break;

        } /* switch(g_curSubSlot_Out) */

        for(int i = 0; i < NUM_USB_CHAN_OUT - g_numUsbChan_Out; i++)
        {
            outuint(c_mix_out, 0);
        }

        /* 3/4 bytes per sample */
        aud_data_remaining_to_device -= (g_numUsbChan_Out * g_curSubSlot_Out);
    }

#endif

    // IN
    {
        int dPtr;
        GET_SHARED_GLOBAL(dPtr, g_aud_to_host_dptr);

        /* Store samples from mixer into sample buffer */
        switch(g_curSubSlot_In)
        {
            case 2:
#if (STREAM_FORMAT_INPUT_SUBSLOT_2_USED == 0)
__builtin_unreachable();
#endif
                for(int i = 0; i < g_numUsbChan_In; i++)
                {
                    /* Receive sample */
                    int sample = inuint(c_mix_out);
#if (INPUT_VOLUME_CONTROL == 1)
#if (!IN_VOLUME_IN_MIXER)
                    /* Apply volume */
                    int mult;
                    int h;
                    unsigned l;
                    unsafe
                    {
                        mult = multInPtr[i];
                    }
                    {h, l} = macs(mult, sample, 0, 0);
                    sample = h << 3;

                    /* Note, in 2 byte sub slot - ignore lower bits of macs */
#elif (IN_VOLUME_IN_MIXER) && defined(IN_VOLUME_AFTER_MIX)
                    sample = sample << 3;
#endif
#endif
                    write_short_via_xc_ptr(dPtr, sample>>16);
                    dPtr+=2;
                }
                break;

            case 4:
            {
#if (STREAM_FORMAT_INPUT_SUBSLOT_4_USED == 0)
__builtin_unreachable();
#endif

                for(int i = 0; i < g_numUsbChan_In; i++)
                {
                    /* Receive sample */
                    int sample = inuint(c_mix_out);
#if(INPUT_VOLUME_CONTROL == 1)
#if (!IN_VOLUME_IN_MIXER)
                    /* Apply volume */
                    int mult;
                    int h;
                    unsigned l;
                    unsafe
                    {
                        mult = multInPtr[i];
                    }
                    {h, l} = macs(mult, sample, 0, 0);
                    sample = h << 3;
#if (STREAM_FORMAT_INPUT_RESOLUTION_32BIT_USED == 1)
                    sample |= (l >> 29) & 0x7; // Note, this step is not required if we assume sample depth is 24 (rather than 32)
#endif
#elif (IN_VOLUME_IN_MIXER) && (IN_VOLUME_AFTER_MIX)
                    sample = sample << 3;
#endif
#endif
                    /* Write into fifo */
                    write_via_xc_ptr(dPtr, sample);
                    dPtr+=4;
                }

                break;
            }

            case 3:
#if (STREAM_FORMAT_INPUT_SUBSLOT_3_USED == 0)
__builtin_unreachable();
#endif
                for(int i = 0; i < g_numUsbChan_In; i++)
                {
                    /* Receive sample */
                    int sample = inuint(c_mix_out);
#if (INPUT_VOLUME_CONTROL) && (!IN_VOLUME_IN_MIXER)
                    /* Apply volume */
                    int mult;
                    int h;
                    unsigned l;
                    unsafe
                    {
                        mult = multInPtr[i];
                    }
                    {h, l} = macs(mult, sample, 0, 0);
                    sample = h << 3;
#endif
                    /* Pack 3 byte samples */
                    switch (packState&0x3)
                    {
                        case 0:
                            packData = sample;
                            break;
                        case 1:
                            packData = (packData >> 8) | ((sample & 0xff00)<<16);
                            write_via_xc_ptr(dPtr, packData);
                            dPtr+=4;
                            write_via_xc_ptr(dPtr, sample>>16);
                            packData = sample;
                            break;
                        case 2:
                            packData = (packData>>16) | ((sample & 0xffff00) << 8);
                            write_via_xc_ptr(dPtr, packData);
                            dPtr+=4;
                            packData = sample;
                            break;
                        case 3:
                            packData = (packData >> 24) | (sample & 0xffffff00);
                            write_via_xc_ptr(dPtr, packData);
                            dPtr+=4;
                            break;
                    }
                    packState++;
                }
                break;

            default:
                __builtin_unreachable();
               break;
        }

        SET_SHARED_GLOBAL(g_aud_to_host_dptr, dPtr);

        /* Input any remaining channels - past this thread we always operate on max channel count */
        for(int i = 0; i < NUM_USB_CHAN_IN - g_numUsbChan_In; i++)
        {
            inuint(c_mix_out);
        }

        sampsToWrite--;
    }

    {
        /* Finished creating packet - commit it to the FIFO */
        /* Total samps to write could start at 0 (i.e. no MCLK) so need to check for < 0) */
        if(sampsToWrite <= 0)
        {
            int speed, wrPtr;
            packState = 0;

            /* Write last packet length into FIFO */
            int datasize = totalSampsToWrite * g_curSubSlot_In * g_numUsbChan_In;

            GET_SHARED_GLOBAL(wrPtr, g_aud_to_host_wrptr);
            write_via_xc_ptr(wrPtr, datasize);

            /* Round up to nearest word - note, not needed for slotsize == 4! */
            datasize = (datasize+3) & (~0x3);
            assert(datasize >= 0);
            assert(datasize <= g_maxPacketSize);

            /* Move wr ptr on by old packet length */
            wrPtr += 4+datasize;
            int fillLevel;
            GET_SHARED_GLOBAL(fillLevel, g_aud_to_host_fill_level);
            fillLevel += 4+datasize;
            assert(fillLevel <= BUFF_SIZE_IN);

            /* Do wrap */
            if (wrPtr >= aud_to_host_fifo_end)
            {
                wrPtr = aud_to_host_fifo_start;
            }

            SET_SHARED_GLOBAL(g_aud_to_host_wrptr, wrPtr);
            SET_SHARED_GLOBAL(g_aud_to_host_dptr, wrPtr + 4);

            /* Now calculate new packet length...
             * First get feedback val (ideally this would be syncronised)
             * Note, if customer hasn't applied a valid MCLK this could go to 0
             * we need to handle this gracefully */
            asm volatile("ldw   %0, dp[g_speed]" : "=r" (speed) :);

            /* Calc packet size to send back based on our fb */
            speedRem += speed;
            totalSampsToWrite = speedRem >> 16;
            speedRem &= 0xffff;

            /* This patches up the case where the FB is well off, leading to totalSampsToWrite to also be off */
            /* This can be startup case, bad mclk input etc */
            if (totalSampsToWrite < 0 || totalSampsToWrite * g_curSubSlot_In * g_numUsbChan_In > g_maxPacketSize)
            {
                totalSampsToWrite = 0;
            }

            /* Must allow space for at least one sample per channel, as these are written at the beginning of
             * the interrupt handler even if totalSampsToWrite is zero (will be overwritten by a later packet). */
            int spaceRequired = XUA_MAX(totalSampsToWrite, 1) * g_numUsbChan_In * g_curSubSlot_In + 4;

            if (spaceRequired > BUFF_SIZE_IN - fillLevel)
            {
                /* In pipe has filled its buffer - we need to overflow
                 * Accept the packet, and throw away the oldest in the buffer */

                unsigned sampFreq;
                GET_SHARED_GLOBAL(sampFreq, g_streamChange_sampFreq);
                int min, mid, max;
                GetADCCounts(sampFreq, min, mid, max);
                const int max_pkt_size = ((max * g_curSubSlot_In * g_numUsbChan_In + 3) & ~0x3) + 4;
                int rdPtr;
                GET_SHARED_GLOBAL(rdPtr, g_aud_to_host_rdptr);

                /* Keep throwing away packets until buffer contains two packets */
                do
                {
                    int wrPtr;
                    GET_SHARED_GLOBAL(wrPtr, g_aud_to_host_wrptr);

                    /* Read length of packet in buffer at read pointer */
                    int datalength;
                    asm volatile("ldw %0, %1[0]":"=r"(datalength):"r"(rdPtr));

                    /* Round up datalength */
                    datalength = ((datalength+3) & ~0x3) + 4;
                    assert(datalength >= 4);
                    assert(fillLevel >= datalength);

                    /* Move read pointer on by length */
                    fillLevel -= datalength;
                    rdPtr += datalength;
                    if (rdPtr >= aud_to_host_fifo_end)
                    {
                        rdPtr = aud_to_host_fifo_start;
                    }

                    assert(rdPtr < aud_to_host_fifo_end && msg("rdPtr must be within buffer"));

                } while (fillLevel > 2 * max_pkt_size);

                SET_SHARED_GLOBAL(g_aud_to_host_rdptr, rdPtr);
            }

            SET_SHARED_GLOBAL(g_aud_to_host_fill_level, fillLevel);
            sampsToWrite = totalSampsToWrite;
        }
    }

    if (!outUnderflow && (aud_data_remaining_to_device<(g_curSubSlot_Out * g_numUsbChan_Out)))
    {
        /* Handle any tail - incase a bad driver sent us a datalength not a multiple of chan count */
        if (aud_data_remaining_to_device)
        {
            /* Round up to nearest word */
            aud_data_remaining_to_device +=3 - (unpackState&0x3);
            aud_data_remaining_to_device &= (~3);

            /* Skip the rest of this malformed packet */
            g_aud_from_host_rdptr += aud_data_remaining_to_device;

            aud_data_remaining_to_device = 0;
        }

        /* Wrap read pointer */
        if (g_aud_from_host_rdptr >= aud_from_host_fifo_end)
        {
            g_aud_from_host_rdptr = aud_from_host_fifo_start;
        }

        outUnderflow = (g_aud_from_host_rdptr == g_aud_from_host_wrptr);

        if (!outUnderflow)
        {
            read_via_xc_ptr(aud_data_remaining_to_device, g_aud_from_host_rdptr);

            unpackState = 0;

            g_aud_from_host_rdptr+=4;
        }
        else
        {
            /* Instrumentation: the FIFO ran dry mid-stream. From here the
             * device plays OUT_BUFFER_PREFILL bytes of silence before
             * resuming -- an audible gap. Counter only: this runs at
             * interrupt level, so the xscope emission happens in the
             * decoupler loop, not here. */
            g_outDryoutCount++;
        }
    }
}

#if (NUM_USB_CHAN_IN > 0)
/* Mark Endpoint (IN) ready with an appropriately sized zero buffer */
/* TODO We should properly size zeros packet rather than using "mid" */
static inline void SetupZerosSendBuffer(XUD_ep aud_to_host_usb_ep, unsigned sampFreq, unsigned slotSize,
                                        xc_ptr aud_to_host_zeros)
{
    int min, mid, max;
    GetADCCounts(sampFreq, min, mid, max);

    /* Set IN stream packet size to something sensible. We expect the buffer to
     * over flow and this to be reset */
    SET_SHARED_GLOBAL(sampsToWrite, mid);
    SET_SHARED_GLOBAL(totalSampsToWrite, mid);

    mid *= g_numUsbChan_In * slotSize;

    asm volatile("stw %0, %1[0]"::"r"(mid),"r"(aud_to_host_zeros));

#if XUA_DEBUG_BUFFER
    printstr("SetupZerosSendBuffer\n");
    printstr("slotSize: ");
    printintln(slotSize);
    printstr("g_numUsbChan_In: ");
    printintln(g_numUsbChan_In);
    printstr("mid: ");
    printintln(mid);
#endif

    /* Mark EP ready with the zero buffer. Note this will simply update the packet size
    * if it is already ready */
    XUD_SetReady_InPtr(aud_to_host_usb_ep, aud_to_host_zeros+4, mid);
}
#endif


static void check_and_signal_stream_event_to_audio(chanend c_mix_out, unsigned dsdMode, unsigned sampResOut)
{
    /* We do OR logic so audio hub is sent info about whether *ANY* stream is active or not */
    g_any_stream_active_current = g_input_stream_active || g_output_stream_active;
    if(g_any_stream_active_current != g_any_stream_active_old)
    {
        /* Forward stream active command to audio if needed - this will cause the audio loop to break */
        inuint(c_mix_out);
        if(g_any_stream_active_current)
        {
            outct(c_mix_out, XUA_AUD_SET_AUDIO_START);
            outuint(c_mix_out, dsdMode);
            outuint(c_mix_out, sampResOut);
        }
        else
        {
            outct(c_mix_out, XUA_AUD_SET_AUDIO_STOP);
        }
        chkct(c_mix_out, XS1_CT_END);
    }
    g_any_stream_active_old = g_any_stream_active_current;
}

#pragma unsafe arrays
void XUA_Buffer_Decouple(chanend c_mix_out
#ifdef XUA_CHAN_BUFF_CTRL
    , chanend c_buf_ctrl
#endif
)
{
    unsigned sampFreq = DEFAULT_FREQ;
#if (NUM_USB_CHAN_OUT > 0)
    int aud_from_host_flag=0;
    xc_ptr released_buffer;
#endif
#if (NUM_USB_CHAN_IN > 0)
    int aud_to_host_flag = 0;
#endif

    int t = array_to_xc_ptr(outAudioBuff);

    aud_from_host_fifo_start = t;
    aud_from_host_fifo_end = aud_from_host_fifo_start + BUFF_SIZE_OUT;
    g_aud_from_host_wrptr = aud_from_host_fifo_start;
    g_aud_from_host_rdptr = aud_from_host_fifo_start;

    t = array_to_xc_ptr(audioBuffIn);

    int aud_to_host_buffer;
    aud_to_host_fifo_start = t;
    aud_to_host_fifo_end = aud_to_host_fifo_start + BUFF_SIZE_IN;
    SET_SHARED_GLOBAL(g_aud_to_host_wrptr, aud_to_host_fifo_start);
    SET_SHARED_GLOBAL(g_aud_to_host_rdptr, aud_to_host_fifo_start);
    SET_SHARED_GLOBAL(g_aud_to_host_dptr, aud_to_host_fifo_start + 4);
    SET_SHARED_GLOBAL(g_aud_to_host_fill_level, 0);

    /* Setup pointer to In stream 0 buffer. Note, length will be innited to 0
     * However, this should be over-written on first stream start (assuming host
       properly sends a SetInterface() before streaming. In any case we will send
       0 length packets, which is reasonable behaviour */
    t = array_to_xc_ptr(inZeroBuff);
    xc_ptr aud_to_host_zeros = t;

    /* Stream format vars */
    int dataFormatOut, sampResOut;
    int dsdMode = DSD_MODE_OFF;
    int dataFormatIn, usbSpeed;

    /* Init vol mult tables */
#if (OUT_VOLUME_IN_MIXER == 0) && (OUTPUT_VOLUME_CONTROL == 1)
    for (int i = 0; i < NUM_USB_CHAN_OUT + 1; i++)
    unsafe{
        multOutPtr[i] = MAX_VOLUME_MULT;
    }
#endif

#if (IN_VOLUME_IN_MIXER == 0) && (INPUT_VOLUME_CONTROL == 1)
    for (int i = 0; i < NUM_USB_CHAN_IN + 1; i++)
    unsafe{
        multInPtr[i] = MAX_VOLUME_MULT;
    }
#endif

    set_interrupt_handler(handle_audio_request, 1, c_mix_out, 0);

    /* Wait for usb_buffer() to set up globals for us to use
     * Note: assumed that buffer_aud_ctl_chan is also setup before these globals are !0 */
#if (NUM_USB_CHAN_OUT > 0)
    while(!aud_from_host_flag)
    {
        GET_SHARED_GLOBAL(aud_from_host_flag, g_aud_from_host_flag);
    }

    aud_from_host_flag = 0;
    SET_SHARED_GLOBAL(g_aud_from_host_flag, aud_from_host_flag);

    // send the current host -> device buffer out of the fifo
    SET_SHARED_GLOBAL(g_aud_from_host_buffer, g_aud_from_host_wrptr);
    XUD_SetReady_OutPtr(aud_from_host_usb_ep, g_aud_from_host_wrptr+4);
#endif

#if (NUM_USB_CHAN_IN > 0)
    /* Wait for usb_buffer to set up */
    while(!aud_to_host_flag)
    {
        GET_SHARED_GLOBAL(aud_to_host_flag, g_aud_to_host_flag);
    }

    aud_to_host_flag = 0;
    SET_SHARED_GLOBAL(g_aud_to_host_flag, aud_to_host_flag);

    /* NOTE: For UAC2 IN EP not marked ready at this point - Initial size of zero buffer not known
     * since we don't know the USB bus-speed yet.
     * The host will send a SetAltInterface before streaming which will lead to this core
     * getting a SET_STREAM_FORMAT_IN. This will setup the EP for the first packet */
#if (AUDIO_CLASS == 1)
    /* For UAC1 we know we only run at FS */
    /* Set buffer back to zeros buffer */
    SetupZerosSendBuffer(aud_to_host_usb_ep, sampFreq, g_curSubSlot_In, aud_to_host_zeros);
#endif
#endif

    while(1)
    {
        int cmd;

#ifdef XUA_CHAN_BUFF_CTRL
        if(!outOverflow)
        {
            /* Need to keep polling in overflow case */
            inuchar(c_buf_ctrl);
        }
#endif
        {
            asm("#decouple-default");

            /* Check for freq change or other update */

            GET_SHARED_GLOBAL(cmd, g_streamChange_flag);
            if (cmd == XUA_AUDCTL_SET_SAMPLE_FREQ)
            {
                SET_SHARED_GLOBAL(g_streamChange_flag, XUA_AUDCTL_NO_COMMAND);
                GET_SHARED_GLOBAL(sampFreq, g_streamChange_sampFreq);

                /* Pass on to mixer */
                DISABLE_INTERRUPTS();
                inuint(c_mix_out);
                outct(c_mix_out, XUA_AUDCTL_SET_SAMPLE_FREQ);
                outuint(c_mix_out, sampFreq);

                if(sampFreq != AUDIO_STOP_FOR_DFU)
                {
                    inUnderflow = 1;
                    SET_SHARED_GLOBAL(g_aud_to_host_rdptr, aud_to_host_fifo_start);
                    SET_SHARED_GLOBAL(g_aud_to_host_wrptr, aud_to_host_fifo_start);
                    SET_SHARED_GLOBAL(g_aud_to_host_dptr,aud_to_host_fifo_start+4);
                    SET_SHARED_GLOBAL(g_aud_to_host_fill_level, 0);

                    /* Set buffer to send back to zeros buffer */
                    aud_to_host_buffer = aud_to_host_zeros;

#if (NUM_USB_CHAN_IN > 0)
                    /* Update size of zeros buffer (and sampsToWrite) */
                    SetupZerosSendBuffer(aud_to_host_usb_ep, sampFreq, g_curSubSlot_In, aud_to_host_zeros);
#endif

#if (NUM_USB_CHAN_OUT > 0)
                    /* Reset OUT buffer state */
                    outUnderflow = 1;
                    XUA_PROBE_UNDERFLOW();
                    SET_SHARED_GLOBAL(g_aud_from_host_rdptr, aud_from_host_fifo_start);
                    SET_SHARED_GLOBAL(g_aud_from_host_wrptr, aud_from_host_fifo_start);
                    SET_SHARED_GLOBAL(aud_data_remaining_to_device, 0);

                    if(outOverflow)
                    {
                        /* If we were previously in overflow we wont have marked as ready */
                        XUD_SetReady_OutPtr(aud_from_host_usb_ep, aud_from_host_fifo_start + 4);
                        outOverflow = 0;
                    }
#endif
                }

                /* Wait for handshake back and pass back up */
                chkct(c_mix_out, XS1_CT_END);

                asm volatile("outct res[%0],%1"::"r"(buffer_aud_ctl_chan),"r"(XS1_CT_END));
                SET_SHARED_GLOBAL(g_streamChangeOngoing, XUA_AUDCTL_NO_COMMAND);
                ENABLE_INTERRUPTS();

                if(sampFreq != AUDIO_STOP_FOR_DFU)
                    speedRem = 0;
                continue;
            }
#if (MIN_FREQ != MAX_FREQ)
            else
#endif
            if(cmd == XUA_AUDCTL_SET_STREAM_INPUT_START)
            {
                /* Change in IN channel count */
                DISABLE_INTERRUPTS();
                SET_SHARED_GLOBAL(g_streamChange_flag, XUA_AUDCTL_NO_COMMAND);

                GET_SHARED_GLOBAL(g_numUsbChan_In, g_formatChange_NumChans);
                GET_SHARED_GLOBAL(g_curSubSlot_In, g_formatChange_SubSlot);
                GET_SHARED_GLOBAL(dataFormatIn, g_formatChange_DataFormat); /* Not currently used for input stream */

                /* Reset IN buffer state */
                inUnderflow = 1;
                SET_SHARED_GLOBAL(g_aud_to_host_rdptr, aud_to_host_fifo_start);
                SET_SHARED_GLOBAL(g_aud_to_host_wrptr,aud_to_host_fifo_start);
                SET_SHARED_GLOBAL(g_aud_to_host_dptr,aud_to_host_fifo_start+4);
                SET_SHARED_GLOBAL(g_aud_to_host_fill_level, 0);

                /* Set buffer back to zeros buffer */
                aud_to_host_buffer = aud_to_host_zeros;

#if (NUM_USB_CHAN_IN > 0)
                /* Update size of zeros buffer (and sampsToWrite) */
                SetupZerosSendBuffer(aud_to_host_usb_ep, sampFreq, g_curSubSlot_In, aud_to_host_zeros);
#endif

                GET_SHARED_GLOBAL(usbSpeed, g_curUsbSpeed);
                if (usbSpeed == XUD_SPEED_HS)
                {
                    g_maxPacketSize = (MAX_DEVICE_AUD_PACKET_SIZE_MULT_HS * g_numUsbChan_In);
                }
                else
                {
                    g_maxPacketSize = (MAX_DEVICE_AUD_PACKET_SIZE_MULT_FS * g_numUsbChan_In);
                }

                g_input_stream_active = 1;
                check_and_signal_stream_event_to_audio(c_mix_out, dsdMode, sampResOut);

                /* ACK back to EP0 */
                asm volatile("outct res[%0],%1"::"r"(buffer_aud_ctl_chan),"r"(XS1_CT_END));
                SET_SHARED_GLOBAL(g_streamChangeOngoing, XUA_AUDCTL_NO_COMMAND);
                ENABLE_INTERRUPTS();
            }
            else if(cmd == XUA_AUDCTL_SET_STREAM_OUTPUT_START)
            {
                /* Change in OUT channel count - note we expect this on every stream start event */
                DISABLE_INTERRUPTS();
                SET_SHARED_GLOBAL(g_streamChange_flag, XUA_AUDCTL_NO_COMMAND);
                GET_SHARED_GLOBAL(g_numUsbChan_Out, g_formatChange_NumChans);
                GET_SHARED_GLOBAL(g_curSubSlot_Out, g_formatChange_SubSlot);
                GET_SHARED_GLOBAL(dataFormatOut, g_formatChange_DataFormat);
                GET_SHARED_GLOBAL(sampResOut, g_formatChange_SampRes);

#if (NUM_USB_CHAN_OUT > 0)
                /* Reset OUT buffer state */
                SET_SHARED_GLOBAL(g_aud_from_host_rdptr, aud_from_host_fifo_start);
                SET_SHARED_GLOBAL(g_aud_from_host_wrptr, aud_from_host_fifo_start);
                SET_SHARED_GLOBAL(aud_data_remaining_to_device, 0);

                /* NOTE, this is potentially usefull for UAC1 */
                unpackState = 0;

                outUnderflow = 1;
                XUA_PROBE_UNDERFLOW();
                if(outOverflow)
                {
                    /* If we were previously in overflow we wont have marked as ready */
                    XUD_SetReady_OutPtr(aud_from_host_usb_ep, aud_from_host_fifo_start+4);
                    outOverflow = 0;
                }
#endif

#ifdef NATIVE_DSD
                if(dataFormatOut == UAC_FORMAT_TYPEI_RAW_DATA)
                {
                    dsdMode = DSD_MODE_NATIVE;
                }
                else
                {
                    dsdMode = DSD_MODE_OFF;
                }
#endif
                /* Wait for the audio code to request samples and respond with command */
                g_output_stream_active = 1;
                check_and_signal_stream_event_to_audio(c_mix_out, dsdMode, sampResOut);

                asm volatile("outct res[%0],%1"::"r"(buffer_aud_ctl_chan),"r"(XS1_CT_END));
                SET_SHARED_GLOBAL(g_streamChangeOngoing, XUA_AUDCTL_NO_COMMAND);
                ENABLE_INTERRUPTS();
            }
            else if(cmd == XUA_AUDCTL_SET_STREAM_INPUT_STOP || cmd == XUA_AUDCTL_SET_STREAM_OUTPUT_STOP)
            {
                DISABLE_INTERRUPTS();
                SET_SHARED_GLOBAL(g_streamChange_flag, XUA_AUDCTL_NO_COMMAND);

                /* clear stream active if needed */
                g_input_stream_active = (cmd == XUA_AUDCTL_SET_STREAM_INPUT_STOP) ? 0 : g_input_stream_active;
                g_output_stream_active = (cmd == XUA_AUDCTL_SET_STREAM_OUTPUT_STOP) ? 0 : g_output_stream_active;
                check_and_signal_stream_event_to_audio(c_mix_out, dsdMode, sampResOut);

                /* ACK back to EP0 */
                asm volatile("outct res[%0],%1"::"r"(buffer_aud_ctl_chan),"r"(XS1_CT_END));
                SET_SHARED_GLOBAL(g_streamChangeOngoing, XUA_AUDCTL_NO_COMMAND);
                ENABLE_INTERRUPTS();
            }
            else if(cmd == XUA_EXIT)
            {
                DISABLE_INTERRUPTS();
                inct(c_mix_out);
                outct(c_mix_out, XS1_CT_END);
                SET_SHARED_GLOBAL(g_streamChangeOngoing, XUA_AUDCTL_NO_COMMAND);
                SET_SHARED_GLOBAL(g_streamChange_flag, XUA_AUDCTL_NO_COMMAND);
                return;
            }
        }

#if (NUM_USB_CHAN_OUT > 0)
        /* Check for OUT data flag from host - set by buffer() */
        GET_SHARED_GLOBAL(aud_from_host_flag, g_aud_from_host_flag);
        if (aud_from_host_flag)
        {
            /* The buffer thread has filled up a buffer */
            int datalength;
            int space_left;
            int aud_from_host_wrptr;
            int aud_from_host_rdptr;
            GET_SHARED_GLOBAL(aud_from_host_wrptr, g_aud_from_host_wrptr);
            GET_SHARED_GLOBAL(aud_from_host_rdptr, g_aud_from_host_rdptr);

            SET_SHARED_GLOBAL(g_aud_from_host_flag, 0);
            GET_SHARED_GLOBAL(released_buffer, g_aud_from_host_buffer);

            /* Read datalength from buffer */
            read_via_xc_ptr(datalength, released_buffer);

            /* Ignore bad small packets */
            if((datalength >= (g_numUsbChan_Out * g_curSubSlot_Out)) && (released_buffer == aud_from_host_wrptr))
            {
                /* Validator: this packet was accepted. Record its size, and
                 * whether it is a whole number of audio frames -- lib_xua
                 * tolerates a non-multiple silently (see the tail handling in
                 * handle_audio_request, whose comment names a bad driver),
                 * which is exactly the kind of quiet tolerance this tool
                 * exists to make audible. */
                g_uacvPktCount++;
                uacvRecordSize((unsigned)datalength);
                if(datalength % (g_numUsbChan_Out * g_curSubSlot_Out))
                    g_uacvPktNotMultiple++;

                /* Move the write pointer of the fifo on - round up to nearest word */
                aud_from_host_wrptr = aud_from_host_wrptr + ((datalength+3)&~0x3) + 4;

                /* Wrap pointer */
                if (aud_from_host_wrptr >= aud_from_host_fifo_end)
                {
                    aud_from_host_wrptr = aud_from_host_fifo_start;
                }
                SET_SHARED_GLOBAL(g_aud_from_host_wrptr, aud_from_host_wrptr);
            }
            else if(datalength < (g_numUsbChan_Out * g_curSubSlot_Out))
            {
                /* Validator: shorter than one audio frame. lib_xua discards
                 * these silently -- count them so the judge can say so. */
                g_uacvPktShortDiscarded++;
            }

            /* if we have enough space left then send a new buffer pointer
             * back to the buffer thread */
            space_left = aud_from_host_rdptr - aud_from_host_wrptr;

            /* Mod and special case */
            if(g_aud_from_host_rdptr == aud_from_host_fifo_start)
            {
                space_left = aud_from_host_fifo_end - g_aud_from_host_wrptr;
            }

            /* Instrumentation: OUT FIFO fill in bytes, every 10th packet
             * (100 Hz). This is the device's own view of whether the host is
             * over- or under-feeding, and unlike the over/underflow flags it
             * is continuous -- drift shows up long before it forces a
             * correction.
             *
             * Deliberately NOT space_left: that is a pointer difference
             * carrying a special case that flips its meaning as the ring
             * wraps, and a drift fit over it produced a slope of 0.222 where
             * calibrated arithmetic demands 1.000. Fill is the plain
             * occupancy (wrptr - rdptr) wrapped into the buffer size. rdptr
             * was sampled at entry, so the value is noisy by up to one packet
             * of concurrent consumption -- harmless for a fit over seconds.
             *
             * The dry-out count rides along at the same cadence: it is
             * incremented at interrupt level in handle_audio_request(), where
             * an xscope call would not be safe, and only emitted here. */
            if(++g_fillProbeDiv >= 10)
            {
                g_fillProbeDiv = 0;
                int fill_bytes = aud_from_host_wrptr - aud_from_host_rdptr;
                if (fill_bytes < 0)
                    fill_bytes += BUFF_SIZE_OUT;
                XUA_PROBE_FILL(fill_bytes);
                XUA_PROBE_DRYOUT();
            }

#if (XUD_USB_ISO_MAX_TXNS_PER_MICROFRAME > 1)
            if (space_left <= 0 || space_left >= REQD_BUF_SIZE_ERR_HANDLING_HIBW_OUT)
#else
            if (space_left <= 0 || space_left >= MAX_DEVICE_AUD_PACKET_SIZE_OUT)
#endif
            {
                SET_SHARED_GLOBAL(g_aud_from_host_buffer, aud_from_host_wrptr);
                XUD_SetReady_OutPtr(aud_from_host_usb_ep, aud_from_host_wrptr+4);
            }
            else
            {
                /* Enter OUT over flow state */
                outOverflow = 1;
                XUA_PROBE_OVERFLOW();

#ifdef DEBUG_LEDS
                led(c_led);
#endif
            }
            continue;
        }
        else if (outOverflow)
        {
            int space_left;
            int aud_from_host_wrptr;
            int aud_from_host_rdptr;
            GET_SHARED_GLOBAL(aud_from_host_wrptr, g_aud_from_host_wrptr);
            GET_SHARED_GLOBAL(aud_from_host_rdptr, g_aud_from_host_rdptr);
            space_left = aud_from_host_rdptr - aud_from_host_wrptr;
            if (space_left <= 0)
                space_left += BUFF_SIZE_OUT;
            if (space_left >= (BUFF_SIZE_OUT/2))
            {
                /* Come out of OUT overflow state */
                outOverflow = 0;
                SET_SHARED_GLOBAL(g_aud_from_host_buffer, aud_from_host_wrptr);
                XUD_SetReady_OutPtr(aud_from_host_usb_ep, aud_from_host_wrptr+4);
#ifdef DEBUG_LEDS
                  led(c_led);
#endif
            }
        }
#endif

#if (NUM_USB_CHAN_IN > 0)
        {
            /* Check if buffer() has sent a packet to host - uses shared mem flag to save chanends */
            int sentPkt;
            GET_SHARED_GLOBAL(sentPkt, g_aud_to_host_flag);
            //case inuint_byref(c_buf_in, cmd):
            if (sentPkt)
            {
                /* Signals that the IN endpoint has sent data from the passed buffer */
                /* Reset flag */
                SET_SHARED_GLOBAL(g_aud_to_host_flag, 0);

                DISABLE_INTERRUPTS();

                if(inUnderflow)
                {
                    int fillLevel;
                    GET_SHARED_GLOBAL(fillLevel, g_aud_to_host_fill_level);
                    assert(fillLevel >= 0);
                    assert(fillLevel <= BUFF_SIZE_IN);

                    /* Check if we have come out of underflow */
                    unsigned sampFreq;
                    GET_SHARED_GLOBAL(sampFreq, g_streamChange_sampFreq);
                    int min, mid, max;
                    GetADCCounts(sampFreq, min, mid, max);
                    const int min_pkt_size = ((min * g_curSubSlot_In * g_numUsbChan_In + 3) & ~0x3) + 4;

                    /*
                        Come out of underflow if there are exactly 2 packets in the buffer.
                        This ensures that handle_audio_request() does not drop packets when writing packets into the aud_to_host buffer
                        when aud_to_host buffer is not in underflow.
                        For example, coming out of underflow with 3 packets in the buffer would mean handle_audio_request()
                        drops packets if 2 pkts are received from audio hub in 1 SOF period. Coming out of underflow with 4
                        packets would mean handle_audio_request would drop packets after writing 1 packet to the aud_to_host buffer.
                    */
                    if ((fillLevel >= (min_pkt_size*2)) && (fillLevel < (min_pkt_size*3)))
                    {
                        int aud_to_host_rdptr;
                        GET_SHARED_GLOBAL(aud_to_host_rdptr, g_aud_to_host_rdptr);
                        inUnderflow = 0;
                        aud_to_host_buffer = aud_to_host_rdptr;
                    }
                    else
                    {
                        aud_to_host_buffer = aud_to_host_zeros;
                    }
                }
                else
                {
                    /* Not in IN underflow state */
                    int datalength;
                    int aud_to_host_wrptr;
                    int aud_to_host_rdptr;
                    int fillLevel;
                    GET_SHARED_GLOBAL(aud_to_host_wrptr, g_aud_to_host_wrptr);
                    GET_SHARED_GLOBAL(aud_to_host_rdptr, g_aud_to_host_rdptr);
                    GET_SHARED_GLOBAL(fillLevel, g_aud_to_host_fill_level);

                    /* Read datalength and round to nearest word */
                    read_via_xc_ptr(datalength, aud_to_host_rdptr);
                    datalength = ((datalength + 3) & ~0x3) + 4;
                    assert(datalength >= 4);
                    assert(fillLevel >= datalength);

                    aud_to_host_rdptr += datalength;
                    fillLevel -= datalength;

                    if (aud_to_host_rdptr >= aud_to_host_fifo_end)
                    {
                        aud_to_host_rdptr = aud_to_host_fifo_start;
                    }
                    SET_SHARED_GLOBAL(g_aud_to_host_rdptr, aud_to_host_rdptr);
                    SET_SHARED_GLOBAL(g_aud_to_host_fill_level, fillLevel);

                    /* Check for read pointer hitting write pointer - underflow */
                    if (fillLevel != 0)
                    {
                        aud_to_host_buffer = aud_to_host_rdptr;
                    }
                    else
                    {
                        assert(aud_to_host_rdptr == aud_to_host_wrptr);
                        inUnderflow = 1;
                        aud_to_host_buffer = aud_to_host_zeros;
                    }
                }

                /* Request to send packet */
                {
                    int len;
                    asm volatile("ldw %0, %1[0]":"=r"(len):"r"(aud_to_host_buffer));
                    XUD_SetReady_InPtr(aud_to_host_usb_ep, aud_to_host_buffer+4, len);
                }

                ENABLE_INTERRUPTS();

                continue;
            }
        }
#endif /* NUM_USB_CHAN_IN > 0 */
    }
}
#endif /* XUA_USB_EN */
