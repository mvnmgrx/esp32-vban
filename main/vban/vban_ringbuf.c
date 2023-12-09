#include "vban_ringbuf.h"

VBAN_RB_ERR_T VBAN_RingBuf_Init(VBAN_RB_T* const ptBuf)
{
    if(!ptBuf) {
        return VBAN_RB_INVALID_PARAM;
    }
    ptBuf->uiHead = 0;
    ptBuf->uiTail = 0;
    ptBuf->fFull = false;
    return VBAN_RB_OKAY;
}
