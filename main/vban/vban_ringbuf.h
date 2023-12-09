#ifndef VBAN_RINGBUF_H
#define VBAN_RINGBUF_H

#include <stdint.h>
#include <stdbool.h>

#include "vban_frame.h"

#define VBAN_RINGBUF_MAX_DEPTH 20

typedef struct VBAN_RINGBUF {
    VBAN_FRAME_T atFrames[VBAN_RINGBUF_MAX_DEPTH];
    unsigned int uiHead;
    unsigned int uiTail;
    bool fFull;
} VBAN_RB_T;

typedef enum VBAN_RB_ERR {
    VBAN_RB_OKAY,
    VBAN_RB_INVALID_PARAM,
    VBAN_RB_UNDERFLOW,
    VBAN_RB_OVERFLOW,
    VBAN_RB_INVALID_OPERATION
} VBAN_RB_ERR_T;

VBAN_RB_ERR_T VBAN_RingBuf_Init(VBAN_RB_T* const ptBuf);

/*************************************************************************************************/ 

inline bool VBAN_RingBuf_IsFull(const VBAN_RB_T* const ptBuf)
{
    return ptBuf->fFull;
}

inline bool VBAN_RingBuf_IsEmpty(const VBAN_RB_T* const ptBuf)
{
    return (!ptBuf->fFull && (ptBuf->uiHead == ptBuf->uiTail));
}

inline void VBAN_RingBuf_Advance(VBAN_RB_T* const ptBuf)
{
    if(!VBAN_RingBuf_IsFull(ptBuf)) {
        ptBuf->uiHead = (ptBuf->uiHead + 1) % VBAN_RINGBUF_MAX_DEPTH;
        ptBuf->fFull = (ptBuf->uiHead == ptBuf->uiTail);
    }
}

inline void VBAN_RingBuf_Retreat(VBAN_RB_T* const ptBuf)
{
    if(!VBAN_RingBuf_IsEmpty(ptBuf)) {
        ptBuf->uiTail = (ptBuf->uiTail + 1) % VBAN_RINGBUF_MAX_DEPTH;
        ptBuf->fFull = false;
    }
}

/*************************************************************************************************/

inline VBAN_RB_ERR_T VBAN_RingBuf_GetNextFreeFrame(VBAN_RB_T* const ptBuf, VBAN_FRAME_T** ptFrame)
{
    if(!ptBuf) {
        return VBAN_RB_INVALID_PARAM;
    }
    if(VBAN_RingBuf_IsFull(ptBuf)) {
        return VBAN_RB_OVERFLOW;
    }
    *ptFrame = &(ptBuf->atFrames[ptBuf->uiHead]);
    return VBAN_RB_OKAY;
}

inline VBAN_RB_ERR_T VBAN_RingBuf_Push(VBAN_RB_T* const ptBuf)
{
    if(!ptBuf) {
        return VBAN_RB_INVALID_PARAM;
    }
    if(VBAN_RingBuf_IsFull(ptBuf)) {
        return VBAN_RB_INVALID_OPERATION;
    }
    VBAN_RingBuf_Advance(ptBuf);
    return VBAN_RB_OKAY;
}

inline VBAN_RB_ERR_T VBAN_RingBuf_GetNextDataFrame(VBAN_RB_T* const ptBuf, VBAN_FRAME_T** ptFrame)
{
    if(!ptBuf) {
        return VBAN_RB_INVALID_PARAM;
    }
    if(VBAN_RingBuf_IsEmpty(ptBuf)) {
        return VBAN_RB_UNDERFLOW;
    }
    *ptFrame = &(ptBuf->atFrames[ptBuf->uiTail]);
    return VBAN_RB_OKAY;
}

inline VBAN_RB_ERR_T VBAN_RingBuf_Pop(VBAN_RB_T* const ptBuf)
{
    if(!ptBuf) {
        return VBAN_RB_INVALID_PARAM;
    }
    if(VBAN_RingBuf_IsEmpty(ptBuf)) {
        return VBAN_RB_INVALID_OPERATION;
    } 
    VBAN_RingBuf_Retreat(ptBuf);
    return VBAN_RB_OKAY;
}

inline unsigned int VBAN_RingBuf_GetFillLevel(const VBAN_RB_T* const ptBuf)
{
    if(!ptBuf) {
        return 0;
    }
    int uiTemp = ptBuf->uiHead - ptBuf->uiTail;
    return (unsigned int)(uiTemp < 0 ? uiTemp + VBAN_RINGBUF_MAX_DEPTH : uiTemp);
}

// extern inline VBAN_RB_ERR_T VBAN_RingBuf_GetNextFreeFrame(const VBAN_RB_T* const ptBuf, VBAN_FRAME_T** ptFrame);
// extern inline VBAN_RB_ERR_T VBAN_RingBuf_Push(VBAN_RB_T* const ptBuf);
// extern inline VBAN_RB_ERR_T VBAN_RingBuf_GetNextDataFrame(const VBAN_RB_T* const ptBuf, VBAN_FRAME_T** ptFrame);
// extern inline VBAN_RB_ERR_T VBAN_RingBuf_Pop(VBAN_RB_T* const ptBuf);

#endif /* VBAN_RINGBUF_H */