
/*
 * mmWave.h
 *
 * This file contains various defines used within this package.
 *
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/


#ifndef _TI_IWR14XX_
#define _TI_IWR14XX_

#include <iostream>
#include <iostream>
#include <cstdio>
#include "serial/serial.h"
#include "ros/ros.h"
#include <boost/thread.hpp>
#include <cstdint>

enum MmwDemo_Output_TLV_Types
{
    MMWDEMO_OUTPUT_MSG_NULL = 0,
    /*! @brief   检测到的点数 */
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS,

    /*! @brief   数据 */
    MMWDEMO_OUTPUT_MSG_RANGE_PROFILE,

    /*! @brief   噪声 */
    MMWDEMO_OUTPUT_MSG_NOISE_PROFILE,

    /*! @brief  计算 azimuth  heatmap */
    MMWDEMO_OUTPUT_MSG_AZIMUTH_STATIC_HEAT_MAP,

    /*! @brief   速度、多普勒 */
    MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP,

    /*! @brief   状态信息 */
    MMWDEMO_OUTPUT_MSG_STATS,

    /*! @brief   检测到的信息 */
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO,

    MMWDEMO_OUTPUT_MSG_MAX
};

enum SorterState{ READ_HEADER, 
    CHECK_TLV_TYPE,
    READ_OBJ_STRUCT, 
    READ_LOG_MAG_RANGE, 
    READ_NOISE, 
    READ_AZIMUTH, 
    READ_DOPPLER, 
    READ_STATS,
    SWAP_BUFFERS,
    READ_SIDE_INFO};

struct MmwDemo_output_message_header_t
    {
        /*! brief   Version: : MajorNum * 2^24 + MinorNum * 2^16 + BugfixNum * 2^8 + BuildNum   */
        uint32_t    version;

        /*! @brief   总包长度 */
        uint32_t    totalPacketLen;

        /*! @brief   平台类型 */
        uint32_t    platform;
        
        /*! @brief   帧数 */
        uint32_t    frameNumber;

        /*! @brief   信息在时间戳中创建的时间 */
        uint32_t    timeCpuCycles;
        
        /*! @brief   检测到的物体数 */
        uint32_t    numDetectedObj;

        /*! @brief   TLV数 */
        uint32_t    numTLVs;

        /*! @brief   主帧数 */
        uint32_t    subFrameNumber;
    };


struct MmwDemo_DetectedObj
    {
        uint16_t   rangeIdx;     /*!< @brief 速度索引 */
        uint16_t   dopplerIdx;   /*!< @brief 多普勒索引 */
        uint16_t   peakVal;      /*!< @brief 峰值 */
        int16_t  x;             /*!< @brief ｘ坐标 */
        int16_t  y;             /*!< @brief y坐标 */
        int16_t  z;             /*!< @brief z 坐标 */
    };
    


/**
 * @brief
 *  笛卡尔坐标系中定义的点信息数据结构
 *
 */
typedef struct DPIF_PointCloudCartesian_t
{
    /*! @brief  x  */
    float  x;

    /*! @brief  y  */
    float  y;

    /*! @brief  z  */
    float  z;

    /*! @brief  多普勒速度　*/
    float    velocity;
}DPIF_PointCloudCartesian;

/**
 * @brief
 *  SNR信息以及噪声阈值
 *
 */
typedef struct DPIF_PointCloudSideInfo_t
{
    
    int16_t  snr;

    
    int16_t  noise;
}DPIF_PointCloudSideInfo;


struct mmwDataPacket{
        
    MmwDemo_output_message_header_t header;
    
    uint16_t numObjOut;
    
    uint16_t xyzQFormat;  // only used for SDK 1.x and 2.x
    
    MmwDemo_DetectedObj objOut;   // only used for SDK 1.x and 2.x

    DPIF_PointCloudCartesian_t newObjOut;  // used for SDK 3.x
    DPIF_PointCloudSideInfo_t  sideInfo;   // used for SDK 3.x
};

const uint8_t magicWord[8] = {2, 1, 4, 3, 6, 5, 8, 7};

#endif










