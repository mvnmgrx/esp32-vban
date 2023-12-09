#include "vban_frame.h"

const unsigned int s_uiSampleRateList[VBAN_FRAME_MAX_SR_NUM] = {
    6000,  12000, 24000, 48000, 96000,  192000, 384000,
    8000,  16000, 32000, 64000, 128000, 256000, 512000,
    11025, 22050, 44100, 88200, 176400, 352800, 705600
};

const unsigned int s_uiSampleLengthList[VBAN_FRAME_BR_MSK + 1] = {
    2, 4, 6, 8, 8, 16, 4, 4
};

VBAN_ERR_T VBAN_Frame_Validate(const VBAN_FRAME_T* const ptFrame,
                            VBAN_SP_T tSubProto,
                            VBAN_CODEC_T tCodec)
{
    if(!ptFrame) {
        return VBAN_ERR_INVALID_PARAM;
    }

    if(ptFrame->uiTotalLen > VBAN_FRAME_MAX_LENGTH) {
        return VBAN_ERR_PACKET_TOO_LONG;
    }

    if(ptFrame->uiTotalLen < VBAN_FRAME_HEADER_LENGTH) {
        return VBAN_ERR_PACKET_TOO_SHORT;
    }

    if(ptFrame->tPacket.tHeader.ulPreamble != VBAN_FRAME_PREAMBLE) {
        return VBAN_ERR_INVALID_PREAMBLE;
    }

    if(VBAN_Frame_GetSubProtocol(ptFrame) != tSubProto) {
        return VBAN_ERR_UNEXPECTED_SUBPROTO;
    }

    if(VBAN_Frame_GetCodec(ptFrame) != tCodec) {
        return VBAN_ERR_UNEXPECTED_CODEC;
    }

    /* Check sample count depending on context */
    unsigned int uiSamplesInPacket = 0;
    switch(VBAN_Frame_GetBitResolution(ptFrame)) {
        case VBAN_BIT_RES_INT16:
        case VBAN_BIT_RES_INT24:
            uiSamplesInPacket = VBAN_Frame_GetDataLen(ptFrame) / s_uiSampleLengthList[VBAN_Frame_GetBitResolution(ptFrame)];
            break;
        default: 
            return VBAN_ERR_UNSUPPORTED_BIT_RES;
    }

    if(   uiSamplesInPacket > VBAN_FRAME_MAX_SAMPLES 
       || uiSamplesInPacket == 0 
       || (ptFrame->tPacket.tHeader.bNumSamples+1) != uiSamplesInPacket) {
        return VBAN_ERR_INVALID_NUM_SAMPLES;
    }

    if(VBAN_Frame_GetNumChannels(ptFrame) >= VBAN_FRAME_MAX_CHANNELS) {
        return VBAN_ERR_INVALID_NUM_CHANNELS;
    }

    /* Bit 3 in bit res must be zero */
    if(ptFrame->tPacket.tHeader.bDataFmtCodec & VBAN_FRAME_BR_MSK_ZERO_BIT) {
        return VBAN_ERR_BR_BIT3_NOT_ZERO;
    }

    return VBAN_OK;
}

const char* VBAN_Frame_GetErrorMessage(VBAN_ERR_T tError)
{
    switch(tError) 
    {
        case VBAN_OK: return "VBAN_OK";
        case VBAN_ERR_INVALID_PARAM: return "VBAN_ERR_INVALID_PARAM";
        case VBAN_ERR_PACKET_TOO_LONG: return "VBAN_ERR_PACKET_TOO_LONG";
        case VBAN_ERR_PACKET_TOO_SHORT: return "VBAN_ERR_PACKET_TOO_SHORT";
        case VBAN_ERR_INVALID_PREAMBLE: return "VBAN_ERR_INVALID_PREAMBLE";
        case VBAN_ERR_UNEXPECTED_SUBPROTO: return "VBAN_ERR_UNEXPECTED_SUBPROTO";
        case VBAN_ERR_UNEXPECTED_CODEC: return "VBAN_ERR_UNEXPECTED_CODEC";
        case VBAN_ERR_UNSUPPORTED_BIT_RES: return "VBAN_ERR_UNSUPPORTED_BIT_RES";
        case VBAN_ERR_INVALID_NUM_SAMPLES: return "VBAN_ERR_INVALID_NUM_SAMPLES";
        case VBAN_ERR_INVALID_NUM_CHANNELS: return "VBAN_ERR_INVALID_NUM_CHANNELS";
        case VBAN_ERR_BR_BIT3_NOT_ZERO: return "VBAN_ERR_BR_BIT3_NOT_ZERO";
        default: return "INVALID_ERROR";
    }
}
