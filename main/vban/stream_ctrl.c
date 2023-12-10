#include <string.h>
#include <stddef.h>

#include "esp_log.h"

#include "stream.h"
#include "stream_ctrl.h"

STREAM_CTRL_RC_T StreamCtrl_Update(STREAM_CTRL_T* ptStreamCtrl)
{
    /* to be implemented */
    return STREAM_CTRL_OKAY;
}

STREAM_CTRL_RC_T StreamCtrl_ComputeNextFrame(STREAM_CTRL_T* ptStreamCtrl, VBAN_FRAME_T* ptFrame)
{
    uint16_t* pusVbanData = NULL;
    bool fSomeStreamActive = false;

    if(!ptStreamCtrl || !ptFrame) {
        return STREAM_CTRL_INVALID_PARAM;
    }

    pusVbanData = (uint16_t*)VBAN_Frame_GetData(ptFrame);

    for(unsigned int i = 0; i < STREAM_CTRL_MAX_STREAMS; i++)
    {
        STREAM_T* ptStream = &ptStreamCtrl->atStreams[i];
        uint16_t* pusNextVbanData = NULL;
        VBAN_FRAME_T* ptNextFrame = NULL;
        unsigned int uiNumSamples = 0;

        /* Skip inactive streams */
        if(!ptStream->fActive) {
            continue;
        }

        /* Get next frame from stream */
        if(STREAM_OKAY != Stream_GetNextDataFrame(ptStream, &ptNextFrame)) {
            continue;
        }

        /* Only supports 16 bit frames for now */
        if(VBAN_BIT_RES_INT16 != VBAN_Frame_GetBitResolution(ptNextFrame)) {
            ptStreamCtrl->tCounters.uiBitrateMissmatch++;
            continue;
        }

        /* First stream dictates overall sample rate and codec */
        if(i == 0) {
            ptFrame->tPacket.tHeader.bSrSubProto = ptNextFrame->tPacket.tHeader.bSrSubProto;
            ptFrame->tPacket.tHeader.bNumChannels = ptNextFrame->tPacket.tHeader.bNumChannels;
            ptFrame->tPacket.tHeader.bDataFmtCodec = ptNextFrame->tPacket.tHeader.bDataFmtCodec;
        }

        /* Update number of samples if needed */
        uiNumSamples = VBAN_Frame_GetNumSamples(ptNextFrame);
        if(uiNumSamples > VBAN_Frame_GetNumSamples(ptFrame)) {
            ptFrame->tPacket.tHeader.bNumSamples = uiNumSamples;
            ptFrame->uiTotalLen = ptNextFrame->uiTotalLen;
        }

        /* Copy each sample onto the samples in ptFrame */
        memcpy(VBAN_Frame_GetData(ptFrame), VBAN_Frame_GetData(ptNextFrame));

        // pusNextVbanData = (uint16_t*)VBAN_Frame_GetData(ptNextFrame);
        // for(unsigned int uiSample = 0; uiSample < uiNumSamples; uiSample++)
        // {
        //     for(unsigned int y = 0; y < 2; y++) {
        //         *pusVbanData += *pusNextVbanData;
        //         pusVbanData++;
        //         pusNextVbanData++;
        //     }
        // }

        /* Confirm that the next frame is handled */
        Stream_ConfirmDataFrame(ptStream);
        fSomeStreamActive = true;
    }

    return fSomeStreamActive ? STREAM_CTRL_OKAY : STREAM_CTRL_NO_DATA;
}

STREAM_CTRL_RC_T StreamCtrl_GetStreamByName(STREAM_CTRL_T* ptStreamCtrl, const char* pszStreamName, STREAM_T** pptStream)
{
    bool fSomeStreamActive = false;

    if(!ptStreamCtrl || !pszStreamName || !pptStream) {
        return STREAM_CTRL_INVALID_PARAM;
    }

    for(unsigned int i = 0; i < STREAM_CTRL_MAX_STREAMS; i++)
    {
        STREAM_T* ptStream = &ptStreamCtrl->atStreams[i];

        /* Skip inactive streams */
        if(!ptStream->fActive) {
            continue;
        }

        /* Skip streams that have the wrong name */
        if(strcmp(ptStream->szName, pszStreamName)) {
            continue;
        }

        *pptStream = ptStream;
        fSomeStreamActive = true;
        break;
    }

    /* No active stream found yet, use next free stream */
    if(!fSomeStreamActive)
    {
        for(unsigned int i = 0; i < STREAM_CTRL_MAX_STREAMS; i++)
        {
            STREAM_T* ptStream = &ptStreamCtrl->atStreams[i];
            /* Skip active streams */
            if(ptStream->fActive) {
                continue;
            }

            strcpy(ptStream->szName, pszStreamName);
            ptStream->fActive = true;
            *pptStream = ptStream;
            fSomeStreamActive = true;
            break;
        }
    }

    return fSomeStreamActive ? STREAM_CTRL_OKAY : STREAM_CTRL_NO_DATA;
}

STREAM_CTRL_RC_T StreamCtrl_PrintStats(STREAM_CTRL_T* ptStreamCtrl, const char* szTag)
{
    if(!ptStreamCtrl || !szTag) {
        return STREAM_CTRL_INVALID_PARAM;
    }

    ESP_LOGI(szTag, "");
    ESP_LOGI(szTag, "");
    ESP_LOGI(szTag, "BITRATE_MISSMATCH : %d", ptStreamCtrl->tCounters.uiBitrateMissmatch);
    ESP_LOGI(szTag, "I2S_BYTES_WRITTEN : %d", ptStreamCtrl->tCounters.uiI2SBytesWritten);
    ESP_LOGI(szTag, "");

    for(unsigned int i = 0; i < STREAM_CTRL_MAX_STREAMS; i++)
    {
        STREAM_T* ptStream = &ptStreamCtrl->atStreams[i];

        if(ptStream->fActive) {
            ESP_LOGI(szTag, "");
            ESP_LOGI(szTag, "");
            ESP_LOGI(szTag, "#### STREAM %d (%s)", i, ptStream->szName);
            ESP_LOGI(szTag, "  FRAME_OK             : %d", ptStream->tCounters.uiFrameOk);
            ESP_LOGI(szTag, "  FRAME_TOO_LONG       : %d", ptStream->tCounters.uiFrameTooLong);
            ESP_LOGI(szTag, "  FRAME_TOO_SHORT      : %d", ptStream->tCounters.uiFrameTooShort);
            ESP_LOGI(szTag, "  INVALID_PREAMBLE     : %d", ptStream->tCounters.uiInvalidPreamble);
            ESP_LOGI(szTag, "  UNEXPECTED_SUBPROTO  : %d", ptStream->tCounters.uiUnexpectedSubproto);
            ESP_LOGI(szTag, "  UNEXPECTED_CODEC     : %d", ptStream->tCounters.uiUnexpectedCodec);
            ESP_LOGI(szTag, "  UNSUPPORTED_BIT_RES  : %d", ptStream->tCounters.uiUnsupportedBitRes);
            ESP_LOGI(szTag, "  INVALID_NUM_SAMPLES  : %d", ptStream->tCounters.uiInvalidNumSamples);
            ESP_LOGI(szTag, "  INVALID_NUM_CHANNELS : %d", ptStream->tCounters.uiInvalidNumChannels);
            ESP_LOGI(szTag, "  BR_BIT_3_NOT_ZERO    : %d", ptStream->tCounters.uiBrBit3NotZero);
            ESP_LOGI(szTag, "  BUFFER_OVERFLOW      : %d", ptStream->tCounters.uiBufferOverflow);
            ESP_LOGI(szTag, "  BUFFER_UNDERFLOW     : %d", ptStream->tCounters.uiBufferUnderflow);
        }
        else {
            ESP_LOGI(szTag, "#### STREAM %d (inactive)", i);
        }
    }

    return STREAM_CTRL_OKAY;
}