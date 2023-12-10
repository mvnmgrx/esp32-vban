#ifndef STREAM_CTRL_H
#define STREAM_CTRL_H

#include "stream.h"

#define STREAM_CTRL_MAX_STREAMS 5

typedef struct STREAM_CTRL_COUNTERS {
    unsigned int uiBitrateMissmatch;
    unsigned int uiI2SBytesWritten;
} STREAM_CTRL_COUNTERS_T;

typedef struct STREAM_CTRL {
    STREAM_T atStreams[STREAM_CTRL_MAX_STREAMS];
    STREAM_CTRL_COUNTERS_T tCounters;
} STREAM_CTRL_T;

typedef enum STREAM_CTRL_RC {
    STREAM_CTRL_OKAY,
    STREAM_CTRL_INVALID_PARAM,
    STREAM_CTRL_NO_DATA
} STREAM_CTRL_RC_T;

STREAM_CTRL_RC_T StreamCtrl_Update(STREAM_CTRL_T* ptStreamCtrl);
STREAM_CTRL_RC_T StreamCtrl_ComputeNextFrame(STREAM_CTRL_T* ptStreamCtrl, VBAN_FRAME_T* ptFrame);
STREAM_CTRL_RC_T StreamCtrl_GetStreamByName(STREAM_CTRL_T* ptStreamCtrl, const char* pszStreamName, STREAM_T** pptStream);
STREAM_CTRL_RC_T StreamCtrl_PrintStats(STREAM_CTRL_T* ptStreamCtrl, const char* szTag);

#endif /* STREAM_CTRL_H */