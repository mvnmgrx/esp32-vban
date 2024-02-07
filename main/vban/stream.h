#ifndef STREAM_H
#define STREAM_H

#include "vban_frame.h"
#include "vban_ringbuf.h"

typedef struct STREAM_COUNTERS {
    unsigned int uiFrameOk;
    unsigned int uiFrameTooLong;
    unsigned int uiFrameTooShort;
    unsigned int uiInvalidPreamble;
    unsigned int uiUnexpectedSubproto;
    unsigned int uiUnexpectedCodec;
    unsigned int uiUnsupportedBitRes;
    unsigned int uiInvalidNumSamples;
    unsigned int uiInvalidNumChannels;
    unsigned int uiBrBit3NotZero;
    unsigned int uiBufferOverflow;
    unsigned int uiBufferUnderflow;
} STREAM_COUNTERS_T;

typedef struct STREAM {
    VBAN_RB_T tRingBuf;
    char szName[VBAN_FRAME_MAX_STREAM_NAME_LENGTH];
    bool fActive;
    unsigned int uiLastFrameRecv;
    STREAM_COUNTERS_T tCounters;
} STREAM_T;

typedef enum STREAM_RC {
    STREAM_OKAY,
    STREAM_NOT_OKAY
} STREAM_RC_T;

STREAM_RC_T Stream_AddFrame(STREAM_T* ptStream, VBAN_FRAME_T* ptFrame);
STREAM_RC_T Stream_GetNextDataFrame(STREAM_T* ptStream, VBAN_FRAME_T** pptFrame);
STREAM_RC_T Stream_ConfirmDataFrame(STREAM_T* ptStream);
STREAM_RC_T Stream_Deinit(STREAM_T* ptStream);

#endif /* STREAM_H */