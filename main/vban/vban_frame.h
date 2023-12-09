#ifndef VBAN_FRAME_H
#define VBAN_FRAME_H

#include <stdint.h>

#define VBAN_FRAME_HEADER_LENGTH 28
#define VBAN_FRAME_MAX_DATA_LENGTH 1436
#define VBAN_FRAME_MAX_LENGTH (VBAN_FRAME_HEADER_LENGTH + VBAN_FRAME_MAX_DATA_LENGTH)

#define VBAN_FRAME_PREAMBLE 0x4E414256UL // 'NABV'
#define VBAN_FRAME_MAX_SAMPLES 256
#define VBAN_FRAME_MAX_CHANNELS 8
#define VBAN_FRAME_MAX_SR_NUM 21
#define VBAN_FRAME_MAX_STREAM_NAME_LENGTH 16

/* Masks */
#define VBAN_FRAME_SR_MSK 0x1F
#define VBAN_FRAME_PROTO_MSK 0xE0
#define VBAN_FRAME_BR_MSK 0x07
#define VBAN_FRAME_CODEC_MSK 0xF0
#define VBAN_FRAME_BR_MSK_ZERO_BIT 0x08

typedef struct VBAN_HEADER {
    uint32_t ulPreamble;
    uint8_t bSrSubProto;
    uint8_t bNumSamples;
    uint8_t bNumChannels;
    uint8_t bDataFmtCodec;
    char cStreamName[16];
    uint32_t ulFrameCnt;
} __attribute__((packed)) VBAN_HEADER_T;

typedef struct VBAN_PACKET {
    VBAN_HEADER_T tHeader;
    uint8_t bData[VBAN_FRAME_MAX_DATA_LENGTH];
} __attribute__((packed)) VBAN_PACKET_T;

typedef struct VBAN_FRAME {
    VBAN_PACKET_T tPacket;
    unsigned int uiTotalLen;
} VBAN_FRAME_T;

typedef enum VBAN_SP {
    VBAN_PROTOCOL_AUDIO = 0x00,
    VBAN_PROTOCOL_SERIAL = 0x20,
    VBAN_PROTOCOL_TXT = 0x40,
    VBAN_PROTOCOL_SERVICE = 0x60,
    VBAN_PROTOCOL_UNDEFINED_1 = 0x80,
    VBAN_PROTOCOL_UNDEFINED_2 = 0xA0,
    VBAN_PROTOCOL_UNDEFINED_3 = 0xC0,
    VBAN_PROTOCOL_USER = 0xE0
} VBAN_SP_T;

typedef enum VBAN_BR {
    VBAN_BIT_RES_BYTE8,
    VBAN_BIT_RES_INT16,
    VBAN_BIT_RES_INT24,
    VBAN_BIT_RES_INT32,
    VBAN_BIT_RES_FLOAT32,
    VBAN_BIT_RES_FLOAT64,
    VBAN_BIT_RES_12BITS,
    VBAN_BIT_RES_10BITS
} VBAN_BR_T;

typedef enum VBAN_CODEC {
    VBAN_CODEC_PCM = 0x0,
    VBAN_CODEC_VBCA = 0x10,
    VBAN_CODEC_CBCV = 0x20,
    VBAN_CODEC_UNDEFINED_1 = 0x30,
    VBAN_CODEC_UNDEFINED_2 = 0x40,
    VBAN_CODEC_UNDEFINED_3 = 0x50,
    VBAN_CODEC_UNDEFINED_4 = 0x60,
    VBAN_CODEC_UNDEFINED_5 = 0x70,
    VBAN_CODEC_UNDEFINED_6 = 0x80,
    VBAN_CODEC_UNDEFINED_7 = 0x90,
    VBAN_CODEC_UNDEFINED_8 = 0xA0,
    VBAN_CODEC_UNDEFINED_9 = 0xB0,
    VBAN_CODEC_UNDEFINED_10 = 0xC0,
    VBAN_CODEC_UNDEFINED_11 = 0xD0,
    VBAN_CODEC_UNDEFINED_12 = 0xE0,
    VBAN_CODEC_USER = 0xF0
} VBAN_CODEC_T;

typedef enum VBAN_ERR {
    VBAN_OK,
    VBAN_ERR_INVALID_PARAM,
    VBAN_ERR_PACKET_TOO_LONG,
    VBAN_ERR_PACKET_TOO_SHORT,
    VBAN_ERR_INVALID_PREAMBLE,
    VBAN_ERR_UNEXPECTED_SUBPROTO,
    VBAN_ERR_UNEXPECTED_CODEC,
    VBAN_ERR_UNSUPPORTED_BIT_RES,
    VBAN_ERR_INVALID_NUM_SAMPLES,
    VBAN_ERR_INVALID_NUM_CHANNELS,
    VBAN_ERR_BR_BIT3_NOT_ZERO
} VBAN_ERR_T;

extern const unsigned int s_uiSampleRateList[VBAN_FRAME_MAX_SR_NUM];

VBAN_ERR_T VBAN_Frame_Validate(const VBAN_FRAME_T* const ptFrame, VBAN_SP_T tSubProto, VBAN_CODEC_T tCodec);

inline unsigned int VBAN_Frame_GetSampleRate(const VBAN_FRAME_T* const ptFrame)
{
    return s_uiSampleRateList[ptFrame->tPacket.tHeader.bSrSubProto & VBAN_FRAME_SR_MSK];
}

inline VBAN_SP_T VBAN_Frame_GetSubProtocol(const VBAN_FRAME_T* const ptFrame)
{
    return (VBAN_SP_T)(ptFrame->tPacket.tHeader.bSrSubProto & VBAN_FRAME_PROTO_MSK);
}

inline unsigned int VBAN_Frame_GetNumSamples(const VBAN_FRAME_T* const ptFrame)
{
    return ptFrame->tPacket.tHeader.bNumSamples+1;
}

inline unsigned int VBAN_Frame_GetNumChannels(const VBAN_FRAME_T* const ptFrame)
{
    return ptFrame->tPacket.tHeader.bNumChannels;
}

inline VBAN_BR_T VBAN_Frame_GetBitResolution(const VBAN_FRAME_T* const ptFrame)
{
    return (VBAN_BR_T)(ptFrame->tPacket.tHeader.bDataFmtCodec & VBAN_FRAME_BR_MSK);
}

inline VBAN_CODEC_T VBAN_Frame_GetCodec(const VBAN_FRAME_T* const ptFrame)
{
    return (VBAN_CODEC_T)(ptFrame->tPacket.tHeader.bDataFmtCodec & VBAN_FRAME_CODEC_MSK);
}

inline const char* VBAN_Frame_GetStreamName(VBAN_FRAME_T* const ptFrame)
{
    ptFrame->tPacket.tHeader.cStreamName[VBAN_FRAME_MAX_STREAM_NAME_LENGTH - 1] = 0;
    return ptFrame->tPacket.tHeader.cStreamName;
}

inline uint32_t VBAN_Frame_GetPacketCounter(const VBAN_FRAME_T* const ptFrame)
{
    return ptFrame->tPacket.tHeader.ulFrameCnt;
}

inline uint8_t* VBAN_Frame_GetData(VBAN_FRAME_T* const ptFrame) 
{
    return ptFrame->tPacket.bData;
}

inline unsigned int VBAN_Frame_GetDataLen(const VBAN_FRAME_T* const ptFrame)
{
    return ptFrame->uiTotalLen - VBAN_FRAME_HEADER_LENGTH;
}

inline void VBAN_Frame_SetTotalLength(VBAN_FRAME_T* const ptFrame, unsigned int uiLength)
{
    ptFrame->uiTotalLen = uiLength;
}

// extern inline unsigned int VBAN_Frame_GetSampleRate(const VBAN_FRAME_T* const ptFrame);
// extern inline VBAN_SP_T VBAN_Frame_GetSubProtocol(const VBAN_FRAME_T* const ptFrame);
// extern inline unsigned int VBAN_Frame_GetNumSamples(const VBAN_FRAME_T* const ptFrame);
// extern inline unsigned int VBAN_Frame_GetNumChannels(const VBAN_FRAME_T* const ptFrame);
// extern inline VBAN_BR_T VBAN_Frame_GetBitResolution(const VBAN_FRAME_T* const ptFrame);
// extern inline VBAN_CODEC_T VBAN_Frame_GetCodec(const VBAN_FRAME_T* const ptFrame);
// extern inline const char* VBAN_Frame_GetStreamName(VBAN_FRAME_T* const ptFrame);
// extern inline uint32_t VBAN_Frame_GetPacketCounter(const VBAN_FRAME_T* const ptFrame);
// extern inline uint8_t* VBAN_Frame_GetData(VBAN_FRAME_T* const ptFrame);
// extern inline unsigned int VBAN_Frame_GetDataLen(const VBAN_FRAME_T* const ptFrame);
// extern inline void VBAN_Frame_SetTotalLength(VBAN_FRAME_T* const ptFrame, unsigned int uiLength);
const char* VBAN_Frame_GetErrorMessage(VBAN_ERR_T tError);

#endif /* VBAN_FRAME_H */