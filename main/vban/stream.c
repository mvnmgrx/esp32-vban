#include <string.h>

#include "stream.h"

STREAM_RC_T Stream_AddFrame(STREAM_T* ptStream, VBAN_FRAME_T* ptFrame)
{
    VBAN_FRAME_T* ptNextFrame = NULL;
    if(!ptStream) {
        return STREAM_NOT_OKAY;
    }

    /* Check if packet length meets VBAN specification */
    VBAN_ERR_T tRetval = VBAN_Frame_Validate(ptFrame, VBAN_PROTOCOL_AUDIO, VBAN_CODEC_PCM);
    if(VBAN_OK != tRetval) {
        switch(tRetval) {
            case VBAN_ERR_PACKET_TOO_LONG: ptStream->tCounters.uiFrameTooLong++; break;
            case VBAN_ERR_PACKET_TOO_SHORT: ptStream->tCounters.uiFrameTooShort++; break;
            case VBAN_ERR_INVALID_PREAMBLE: ptStream->tCounters.uiInvalidPreamble++; break;
            case VBAN_ERR_UNEXPECTED_SUBPROTO: ptStream->tCounters.uiUnexpectedSubproto++; break;
            case VBAN_ERR_UNEXPECTED_CODEC: ptStream->tCounters.uiUnexpectedCodec++; break;
            case VBAN_ERR_UNSUPPORTED_BIT_RES: ptStream->tCounters.uiUnsupportedBitRes++; break;
            case VBAN_ERR_INVALID_NUM_SAMPLES: ptStream->tCounters.uiInvalidNumSamples++; break;
            case VBAN_ERR_INVALID_NUM_CHANNELS: ptStream->tCounters.uiInvalidNumChannels++; break;
            case VBAN_ERR_BR_BIT3_NOT_ZERO: ptStream->tCounters.uiBrBit3NotZero++; break;
            default:
                break;
        }
        return STREAM_NOT_OKAY;
    }

    if(VBAN_RB_OKAY != VBAN_RingBuf_GetNextFreeFrame(&ptStream->tRingBuf, &ptNextFrame)) {
        ptStream->tCounters.uiBufferOverflow++;
        return STREAM_NOT_OKAY;
    }

    /* Copy frame into ringbuffer slot */
    memcpy(ptNextFrame, ptFrame, sizeof(VBAN_FRAME_T));

    if(VBAN_RB_OKAY != VBAN_RingBuf_Push(&ptStream->tRingBuf)) {
        return STREAM_NOT_OKAY;
    }

    ptStream->tCounters.uiFrameOk++;
    return STREAM_OKAY;
}

STREAM_RC_T Stream_GetNextDataFrame(STREAM_T* ptStream, VBAN_FRAME_T** pptFrame)
{
    if(!ptStream) {
        return STREAM_NOT_OKAY;
    }

    /* Get next data frame from ring buffer */
    if(VBAN_RingBuf_GetNextDataFrame(&ptStream->tRingBuf, pptFrame) != VBAN_RB_OKAY) {
        ptStream->tCounters.uiBufferUnderflow++;
        return STREAM_NOT_OKAY;
    }

    return STREAM_OKAY;
}

STREAM_RC_T Stream_ConfirmDataFrame(STREAM_T* ptStream)
{
    return VBAN_RingBuf_Pop(&ptStream->tRingBuf) ? STREAM_NOT_OKAY : STREAM_OKAY;
}