/*
 *  RPLIDAR SDK
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2019 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "rplidar_driver_impl.h"

#include <algorithm>

#if !defined(_countof)
#if !defined(__cplusplus)
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#else
extern "C++"
{
template <typename _CountofType, size_t _SizeOfArray>
char (*__countof_helper( _CountofType (&_Array)[_SizeOfArray]))[_SizeOfArray];
#define _countof(_Array) sizeof(*__countof_helper(_Array))
}
#endif
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

#define DEPRECATED_WARN(fn, replacement) do { \
        static bool __shown__ = false; \
        if (!__shown__) { \
            printDeprecationWarn(fn, replacement); \
            __shown__ = true; \
        } \
    } while (0)

    static void printDeprecationWarn(const char* fn, const char* replacement)
    {
        fprintf(stderr, "*WARN* YOU ARE USING DEPRECATED API: %s, PLEASE MOVE TO %s\n", fn, replacement);
    }

static void convert(const rplidar_response_measurement_node_t& from, rplidar_response_measurement_node_hq_t& to)
{
    to.angle_z_q14 = (((from.angle_q6_checkbit) >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) << 8) / 90;  //transfer to q14 Z-angle
    to.dist_mm_q2 = from.distance_q2;
    to.flag = (from.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);  // trasfer syncbit to HQ flag field
    to.quality = (from.sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;  //remove the last two bits and then make quality from 0-63 to 0-255
}

static void convert(const rplidar_response_measurement_node_hq_t& from, rplidar_response_measurement_node_t& to)
{
    to.sync_quality = (from.flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) | ((from.quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
    to.angle_q6_checkbit = 1 | (((from.angle_z_q14 * 90) >> 8) << RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT);
    to.distance_q2 = from.dist_mm_q2 > _u16(-1) ? _u16(0) : _u16(from.dist_mm_q2);
}

const int lidarRxPin = 34;
const int lidarTxPin = 33;
#define lidarSerial Serial5

bool RPLidar::begin()
{
    pinMode(lidarRxPin, INPUT);
    pinMode(lidarTxPin, OUTPUT);
    lidarSerial.begin(115200);

    _cached_scan_node_hq_count = 0;
    _isConnected = true;

    return true;
}

bool RPLidar::isConnected()
{
    return _isConnected;
}

bool RPLidar::isScanning()
{
    return _isScanning;
}

u_result RPLidar::reset(_u32 timeout)
{
    u_result ans;

    {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_RESET))) {
            return ans;
        }
    }
    return RESULT_OK;
}

u_result RPLidar::_waitResponseHeader(rplidar_ans_header_t * header, _u32 timeout)
{
    _u8  recvPos = 0;
    _u32 currentTs = millis();
    _u32 remainingtime;
    _u8 *headerbuf = (_u8*)header;
    while ((remainingtime=millis() - currentTs) <= timeout) {
        
        int currentbyte = lidarSerial.read();
        if (currentbyte<0) continue;
        
        switch (recvPos) {
        case 0:
            if (currentbyte != RPLIDAR_ANS_SYNC_BYTE1) {
                continue;
            }
            break;
        case 1:
            if (currentbyte != RPLIDAR_ANS_SYNC_BYTE2) {
                recvPos = 0;
                continue;
            }
            break;
        }
        headerbuf[recvPos++] = currentbyte;

        if (recvPos == sizeof(rplidar_ans_header_t)) {
            return RESULT_OK;
        }
    }

    Serial.println("waitReponseHeader timeout");

    return RESULT_OPERATION_TIMEOUT;
}

u_result RPLidar::getHealth(rplidar_response_device_health_t & healthinfo, _u32 timeout)
{
    u_result  ans;
    
    if (!isConnected()) return RESULT_OPERATION_FAIL;
    
    _disableDataGrabbing();

    {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_HEALTH))) {
            return ans;
        }

        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_DEVHEALTH) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if ( header_size < sizeof(rplidar_response_device_health_t)) {
            return RESULT_INVALID_DATA;
        }

        //  if (!_chanDev->waitfordata(header_size, timeout)) {
        //     return RESULT_OPERATION_TIMEOUT;
        // }
        // _chanDev->recvdata(reinterpret_cast<_u8 *>(&healthinfo), sizeof(healthinfo));
        _u8 *infobuf = (_u8 *)&healthinfo;
        _u8  recvPos = 0;
        _u32 currentTs = millis();
        _u32 remainingtime;
        while ((remainingtime=millis() - currentTs) <= timeout) {
            int currentbyte = lidarSerial.read();
            if (currentbyte < 0) continue;
            
            infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(rplidar_response_device_health_t)) {
                return RESULT_OK;
            }
        }
    }
    return RESULT_OK;
}

u_result RPLidar::getDeviceInfo(rplidar_response_device_info_t & info, _u32 timeout)
{
    u_result  ans;
    
    if (!isConnected()) return RESULT_OPERATION_FAIL;

    _disableDataGrabbing();

    {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_INFO))) {
            return ans;
        }

        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_DEVINFO) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if (header_size < sizeof(rplidar_response_device_info_t)) {
            return RESULT_INVALID_DATA;
        }

        // if (!_chanDev->waitfordata(header_size, timeout)) {
        //     return RESULT_OPERATION_TIMEOUT;
        // }
        // _chanDev->recvdata(reinterpret_cast<_u8 *>(&info), sizeof(info));
        _u8 *infobuf = (_u8*)&info;
        _u32 remainingtime;
        _u32 currentTs = millis();
        _u8  recvPos = 0;
        while ((remainingtime=millis() - currentTs) <= timeout) {
            int currentbyte = lidarSerial.read();
            if (currentbyte<0) continue;    
            infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(rplidar_response_device_info_t)) {
                return RESULT_OK;
            }
        }
        if ((info.model >> 4) > RPLIDAR_TOF_MINUM_MAJOR_ID){
            _isTofLidar = true;
        }else {
            _isTofLidar = false;
        }
    }
    return RESULT_OK;
}

u_result RPLidar::_waitNode(rplidar_response_measurement_node_t * node, _u32 timeout)
{
   _u32 currentTs = millis();
   _u32 remainingtime;
   _u8 *nodebuf = (_u8*)node;
   _u8 recvPos = 0;

   while ((remainingtime=millis() - currentTs) <= timeout) {
        int currentbyte = lidarSerial.read();
        if (currentbyte<0) continue;

        switch (recvPos) {
            case 0: // expect the sync bit and its reverse in this byte          {
                {
                    _u8 tmp = (currentbyte>>1);
                    if ( (tmp ^ currentbyte) & 0x1 ) {
                        // pass
                    } else {
                        continue;
                    }

                }
                break;
            case 1: // expect the highest bit to be 1
                {
                    if (currentbyte & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) {
                        // pass
                    } else {
                        recvPos = 0;
                        continue;
                    }
                }
                break;
          }
          nodebuf[recvPos++] = currentbyte;

          if (recvPos == sizeof(rplidar_response_measurement_node_t)) {
            return RESULT_OK;
          }
   }

   return RESULT_OPERATION_TIMEOUT;
}

u_result RPLidar::_waitCapsuledNode(rplidar_response_capsule_measurement_nodes_t & node, _u32 timeout)
{
    int  recvPos = 0;
    _u32 startTs = millis();
    _u8  recvBuffer[sizeof(rplidar_response_capsule_measurement_nodes_t)];
    _u8 *nodeBuffer = (_u8*)&node;
    _u32 waitTime;

   while ((waitTime=millis() - startTs) <= timeout) {
        size_t remainSize = sizeof(rplidar_response_capsule_measurement_nodes_t) - recvPos;

        int currentbyte = lidarSerial.read();
        if (currentbyte<0) continue;
        recvBuffer[0] = currentbyte;
        size_t recvSize=1;
        
        for (size_t pos = 0; pos < recvSize; ++pos) {
            _u8 currentByte = recvBuffer[pos];

            switch (recvPos) {
            case 0: // expect the sync bit 1
                {
                    _u8 tmp = (currentByte>>4);
                    if ( tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1 ) {
                        // pass
                    } else {
                        _is_previous_capsuledataRdy = false;
                        continue;
                    }

                }
                break;
            case 1: // expect the sync bit 2
                {
                    _u8 tmp = (currentByte>>4);
                    if (tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
                        // pass
                    } else {
                        recvPos = 0;
                        _is_previous_capsuledataRdy = false;
                        continue;
                    }
                }
                break;
            }
            nodeBuffer[recvPos++] = currentByte;
            if (recvPos == sizeof(rplidar_response_capsule_measurement_nodes_t)) {
                // calc the checksum ...
                _u8 checksum = 0;
                _u8 recvChecksum = ((node.s_checksum_1 & 0xF) | (node.s_checksum_2<<4));
                for (size_t cpos = offsetof(rplidar_response_capsule_measurement_nodes_t, start_angle_sync_q6);
                    cpos < sizeof(rplidar_response_capsule_measurement_nodes_t); ++cpos)
                {
                    checksum ^= nodeBuffer[cpos];
                }
                if (recvChecksum == checksum)
                {
                    // only consider vaild if the checksum matches...
                    if (node.start_angle_sync_q6 & RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT) 
                    {
                        // this is the first capsule frame in logic, discard the previous cached data...
                        _is_previous_capsuledataRdy = false;
                        return RESULT_OK;
                    }
                    return RESULT_OK;
                }
                _is_previous_capsuledataRdy = false;
                return RESULT_INVALID_DATA;
            }
        }
    }
    _is_previous_capsuledataRdy = false;
    return RESULT_OPERATION_TIMEOUT;
}

u_result RPLidar::loopScanData()
{
    static uint16_t recvNodeCount = 0;
    u_result ans;
    rplidar_response_measurement_node_t node;
    rplidar_response_measurement_node_hq_t nodeHq;

    if (IS_FAIL(ans = _waitNode(&node, DEFAULT_TIMEOUT))) {
        _isScanning = false;
        return RESULT_OPERATION_FAIL;
    }

    convert(node, nodeHq);
    _cached_scan_node_hq_buf[_cached_scan_node_hq_count++] = nodeHq;
    if (_cached_scan_node_hq_count >= _countof(_cached_scan_node_hq_buf)){
        _cached_scan_node_hq_count = 0;
    }

    return RESULT_OK;
}

u_result RPLidar::loopScanExpressData()
{
    static uint16_t recvNodeCount = 0;
    u_result ans;
    rplidar_response_capsule_measurement_nodes_t capsule_node;
    rplidar_response_measurement_node_hq_t nodesHq[512];

    if (IS_FAIL(ans = _waitCapsuledNode(capsule_node, DEFAULT_TIMEOUT))) {
        _isScanning = false;
        return RESULT_OPERATION_FAIL;
    }

    size_t count = 512;
    _capsuleToNormal(capsule_node, nodesHq, count);

    for (size_t pos=0; pos < count; ++pos){
        _cached_scan_node_hq_buf[_cached_scan_node_hq_count++] = nodesHq[pos];
        if (_cached_scan_node_hq_count >= _countof(_cached_scan_node_hq_buf)){
            _cached_scan_node_hq_count = 0;
        }
    }
    return RESULT_OK;
}

u_result RPLidar::grabScanData(rplidar_response_measurement_node_hq_t * nodebuffer, size_t & count, _u32 timeout)
{
    if (_cached_scan_node_hq_count == 0) return RESULT_OPERATION_TIMEOUT; //consider as timeout
    size_t size_to_copy = min(count, _cached_scan_node_hq_count);
    memcpy(nodebuffer, _cached_scan_node_hq_buf, _cached_scan_node_hq_count * sizeof(rplidar_response_measurement_node_hq_t));
    count = size_to_copy;
    _cached_scan_node_hq_count = 0;
    return RESULT_OK;
}
u_result RPLidar::grabScanExpressData(rplidar_response_measurement_node_hq_t * nodebuffer, size_t & count, _u32 timeout)
{
    return grabScanData(nodebuffer, count, timeout);
}

u_result RPLidar::startScanNormal(bool force, _u32 timeout)
{
    u_result ans;
    if (!isConnected()) return RESULT_OPERATION_FAIL;
    if (_isScanning) return RESULT_ALREADY_DONE;

    stop(); //force the previous operation to stop

    {
        if (IS_FAIL(ans = _sendCommand(force?RPLIDAR_CMD_FORCE_SCAN:RPLIDAR_CMD_SCAN))) {
            return ans;
        }

        // waiting for confirmation
        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_MEASUREMENT) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if (header_size < sizeof(rplidar_response_measurement_node_t)) {
            return RESULT_INVALID_DATA;
        }

        _isScanning = true;
        // _cachethread = CLASS_THREAD(RPLidar, _cacheScanData);
        // if (_cachethread.getHandle() == 0) {
        //     return RESULT_OPERATION_FAIL;
        // }
    }
    return RESULT_OK;
}

// TODO not implemented
int RPLidar::_getSyncBitByAngle(const int current_angle_q16, const int angleInc_q16)
{
    static int last_angleInc_q16 = 0;
    int current_angleInc_q16 = angleInc_q16;
    int syncBit_check_threshold = (int)((5 << 16) / angleInc_q16) + 1;//find syncBit in 0~3 degree
    int syncBit = 0;
    int predict_angle_q16 = (current_angle_q16 + angleInc_q16) % (360 << 16);

    if (predict_angle_q16 < 0) {
        predict_angle_q16 += (360 << 16);
    }
    if (!_syncBit_is_finded)
    {
        if (0 < predict_angle_q16 && predict_angle_q16 < (90 << 16))
            syncBit = 1;
        if (syncBit)
            _syncBit_is_finded = true;
    }
    else
    {
        if(predict_angle_q16 > (270<<16))
            _syncBit_is_finded = false;
        //if (predict_angle_q16 > (syncBit_check_threshold * angleInc_q16)) {
        //    _is_previous_syncBit = false;
        //}
    }
    last_angleInc_q16 = current_angleInc_q16;
    return syncBit;
}

void RPLidar::_capsuleToNormal(const rplidar_response_capsule_measurement_nodes_t & capsule, rplidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount)
{
    nodeCount = 0;
    if (_is_previous_capsuledataRdy) {
        int diffAngle_q8;
        int currentStartAngle_q8 = ((capsule.start_angle_sync_q6 & 0x7FFF)<< 2);
        int prevStartAngle_q8 = ((_cached_previous_capsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

        diffAngle_q8 = (currentStartAngle_q8) - (prevStartAngle_q8);
        if (prevStartAngle_q8 >  currentStartAngle_q8) {
            diffAngle_q8 += (360<<8);
        }

        int angleInc_q16 = (diffAngle_q8 << 3);
        int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
        for (size_t pos = 0; pos < _countof(_cached_previous_capsuledata.cabins); ++pos)
        {
            int dist_q2[2];
            int angle_q16[2];
            int syncBit[2] = { 0,0 };

            dist_q2[0] = (_cached_previous_capsuledata.cabins[pos].distance_angle_1 & 0xFFFC);
            dist_q2[1] = (_cached_previous_capsuledata.cabins[pos].distance_angle_2 & 0xFFFC);

            int angle_offset1_q3 = ( (_cached_previous_capsuledata.cabins[pos].offset_angles_q3 & 0xF) | ((_cached_previous_capsuledata.cabins[pos].distance_angle_1 & 0x3)<<4));
            int angle_offset2_q3 = ( (_cached_previous_capsuledata.cabins[pos].offset_angles_q3 >> 4) | ((_cached_previous_capsuledata.cabins[pos].distance_angle_2 & 0x3)<<4));

            int syncBit_check_threshold = (int)((2 << 16) / angleInc_q16) + 1;//find syncBit in 0~1 degree

            angle_q16[0] = (currentAngle_raw_q16 - (angle_offset1_q3<<13));
            syncBit[0] = _getSyncBitByAngle(currentAngle_raw_q16, angleInc_q16);
            currentAngle_raw_q16 += angleInc_q16;

            angle_q16[1] = (currentAngle_raw_q16 - (angle_offset2_q3<<13));
            syncBit[1] = _getSyncBitByAngle(currentAngle_raw_q16, angleInc_q16);
            currentAngle_raw_q16 += angleInc_q16;

            for (int cpos = 0; cpos < 2; ++cpos) {

                if (angle_q16[cpos] < 0) angle_q16[cpos] += (360<<16);
                if (angle_q16[cpos] >= (360<<16)) angle_q16[cpos] -= (360<<16);

                rplidar_response_measurement_node_hq_t node;

                node.angle_z_q14 = _u16((angle_q16[cpos] >> 2) / 90);
                node.flag = (syncBit[cpos] | ((!syncBit[cpos]) << 1));
                node.quality = dist_q2[cpos] ? (0x2f << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
                node.dist_mm_q2 = dist_q2[cpos];

                nodebuffer[nodeCount++] = node;
             }

        }
    }

    _cached_previous_capsuledata = capsule;
    _is_previous_capsuledataRdy = true;
}

void RPLidar::_dense_capsuleToNormal(const rplidar_response_capsule_measurement_nodes_t & capsule, rplidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount)
{
    const rplidar_response_dense_capsule_measurement_nodes_t *dense_capsule = reinterpret_cast<const rplidar_response_dense_capsule_measurement_nodes_t*>(&capsule);
    nodeCount = 0;
    if (_is_previous_capsuledataRdy) {
        int diffAngle_q8;
        int currentStartAngle_q8 = ((dense_capsule->start_angle_sync_q6 & 0x7FFF) << 2);
        int prevStartAngle_q8 = ((_cached_previous_dense_capsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

        diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8);
        if (prevStartAngle_q8 >  currentStartAngle_q8) {
            diffAngle_q8 += (360 << 8);
        }

        int angleInc_q16 = (diffAngle_q8 << 8)/40;
        int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
        for (size_t pos = 0; pos < _countof(_cached_previous_dense_capsuledata.cabins); ++pos)
        {
            int dist_q2;
            int angle_q6;
            int syncBit;
            const int dist = static_cast<const int>(_cached_previous_dense_capsuledata.cabins[pos].distance);
            dist_q2 = dist << 2;
            angle_q6 = (currentAngle_raw_q16 >> 10);
            syncBit = _getSyncBitByAngle(currentAngle_raw_q16, angleInc_q16);
            currentAngle_raw_q16 += angleInc_q16;

            if (angle_q6 < 0) angle_q6 += (360 << 6);
            if (angle_q6 >= (360 << 6)) angle_q6 -= (360 << 6);

            

            rplidar_response_measurement_node_hq_t node;

            node.angle_z_q14 = _u16((angle_q6 << 8) / 90);
            node.flag = (syncBit | ((!syncBit) << 1));
            node.quality = dist_q2 ? (0x2f << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) : 0;
            node.dist_mm_q2 = dist_q2;

            nodebuffer[nodeCount++] = node;
            

        }
    }

    _cached_previous_dense_capsuledata = *dense_capsule;
    _is_previous_capsuledataRdy = true;
}

//CRC calculate
static _u32 table[256];//crc32_table

//reflect
static _u32 _bitrev(_u32 input, _u16 bw)
{
    _u16 i;
    _u32 var;
    var = 0;
    for (i = 0; i<bw; i++){
        if (input & 0x01)
        {
            var |= 1 << (bw - 1 - i);
        }
        input >>= 1;
    }
    return var;
}

// crc32_table init
static void _crc32_init(_u32 poly)
{
    _u16 i;
    _u16 j;
    _u32 c;
    
    poly = _bitrev(poly, 32);
    for (i = 0; i<256; i++){
        c = i;
        for (j = 0; j<8; j++){
            if (c & 1)
                c = poly ^ (c >> 1);
            else
                c = c >> 1;
        }
        table[i] = c;
    }
}

static _u32 _crc32cal(_u32 crc, void* input, _u16 len)
{
    _u16 i;
    _u8 index;
    _u8* pch;
    pch = (unsigned char*)input;
    _u8 leftBytes = 4 - len & 0x3;

    for (i = 0; i<len; i++){
        index = (unsigned char)(crc^*pch);
        crc = (crc >> 8) ^ table[index];
        pch++;
    }

    for (i = 0; i < leftBytes; i++) {//zero padding
        index = (unsigned char)(crc^0);
        crc = (crc >> 8) ^ table[index];
    }
    return crc^0xffffffff;
}

//crc32cal
static u_result _crc32(_u8 *ptr, _u32 len) {
	static _u8 tmp;
	if (tmp != 1) {
		_crc32_init(0x4C11DB7);
		tmp = 1;
	}
	
	return _crc32cal(0xFFFFFFFF, ptr,len);
}

// TODO not implemented
u_result RPLidar::_waitHqNode(rplidar_response_hq_capsule_measurement_nodes_t & node, _u32 timeout)
{
    if (!_isConnected) {
        return RESULT_OPERATION_FAIL;
    }

    int  recvPos = 0;
    _u32 startTs = millis();
    _u8  recvBuffer[sizeof(rplidar_response_hq_capsule_measurement_nodes_t)];
    _u8 *nodeBuffer = (_u8*)&node;
    _u32 waitTime;
    
    while ((waitTime=millis() - startTs) <= timeout) {
        size_t remainSize = sizeof(rplidar_response_hq_capsule_measurement_nodes_t) - recvPos;
        size_t recvSize;
        
        // TODO
        // bool ans = _chanDev->waitfordata(remainSize, timeout-waitTime, &recvSize);
        // if(!ans)
        // {
        //     return RESULT_OPERATION_TIMEOUT;
        // }
        // if (recvSize > remainSize) recvSize = remainSize;
        
        // recvSize = _chanDev->recvdata(recvBuffer, recvSize);
    
        for (size_t pos = 0; pos < recvSize; ++pos) {
            _u8 currentByte = recvBuffer[pos];
            switch (recvPos) {
            case 0: // expect the sync byte
                {
                    _u8 tmp = (currentByte);
                    if ( tmp == RPLIDAR_RESP_MEASUREMENT_HQ_SYNC ) {
                    // pass
                    }
                    else {
                        recvPos = 0;
                        _is_previous_HqdataRdy = false;
                        continue;
                    }
                }
           break;
           case sizeof(rplidar_response_hq_capsule_measurement_nodes_t) - 1 - 4: 
            {

            }
           break;
           case sizeof(rplidar_response_hq_capsule_measurement_nodes_t) - 1: 
            {				

            }
           break;
           }
           nodeBuffer[recvPos++] = currentByte;
           if (recvPos == sizeof(rplidar_response_hq_capsule_measurement_nodes_t)) {
                _u32 crcCalc2 = _crc32(nodeBuffer, sizeof(rplidar_response_hq_capsule_measurement_nodes_t) - 4);

                if(crcCalc2 == node.crc32){
                    _is_previous_HqdataRdy = true;
                    return RESULT_OK;
                }
                else {
                    _is_previous_HqdataRdy = false;
                    return RESULT_INVALID_DATA;
                }

            }
        }
    }
    _is_previous_HqdataRdy = false;
    return RESULT_OPERATION_TIMEOUT;
}

void RPLidar::_HqToNormal(const rplidar_response_hq_capsule_measurement_nodes_t & node_hq, rplidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount) 
{
    nodeCount = 0;
    if (_is_previous_HqdataRdy) {
        for (size_t pos = 0; pos < _countof(_cached_previous_Hqdata.node_hq); ++pos)
        {
            nodebuffer[nodeCount++] = node_hq.node_hq[pos];
        }	
    }
    _cached_previous_Hqdata = node_hq;
    _is_previous_HqdataRdy = true;

}
//*******************************************HQ support********************************//

static _u32 _varbitscale_decode(_u32 scaled, _u32 & scaleLevel)
{
    static const _u32 VBS_SCALED_BASE[] = {
        RPLIDAR_VARBITSCALE_X16_DEST_VAL,
        RPLIDAR_VARBITSCALE_X8_DEST_VAL,
        RPLIDAR_VARBITSCALE_X4_DEST_VAL,
        RPLIDAR_VARBITSCALE_X2_DEST_VAL,
        0,
    };

    static const _u32 VBS_SCALED_LVL[] = {
        4,
        3,
        2,
        1,
        0,
    };

    static const _u32 VBS_TARGET_BASE[] = {
        (0x1 << RPLIDAR_VARBITSCALE_X16_SRC_BIT),
        (0x1 << RPLIDAR_VARBITSCALE_X8_SRC_BIT),
        (0x1 << RPLIDAR_VARBITSCALE_X4_SRC_BIT),
        (0x1 << RPLIDAR_VARBITSCALE_X2_SRC_BIT),
        0,
    };

    for (size_t i = 0; i < _countof(VBS_SCALED_BASE); ++i)
    {
        int remain = ((int)scaled - (int)VBS_SCALED_BASE[i]);
        if (remain >= 0) {
            scaleLevel = VBS_SCALED_LVL[i];
            return VBS_TARGET_BASE[i] + (remain << scaleLevel);
        }
    }
    return 0;
}

u_result RPLidar::checkSupportConfigCommands(bool& outSupport, _u32 timeoutInMs)
{
    u_result ans;

    rplidar_response_device_info_t devinfo;
    ans = getDeviceInfo(devinfo, timeoutInMs);
    if (IS_FAIL(ans)) return ans;

    // if lidar firmware >= 1.24
    if (devinfo.firmware_version >= ((0x1 << 8) | 24)) {
        outSupport = true;
    }
    return ans;
}

#if 0
// TODO not implemented
u_result RPLidar::getLidarConf(_u32 type, std::vector<_u8> &outputBuf, const std::vector<_u8> &reserve, _u32 timeout)
{
    rplidar_payload_get_scan_conf_t query;
    memset(&query, 0, sizeof(query));
    query.type = type;
    int sizeVec = reserve.size();

    int maxLen = sizeof(query.reserved) / sizeof(query.reserved[0]);
    if (sizeVec > maxLen) sizeVec = maxLen;

    if (sizeVec > 0)
        memcpy(query.reserved, &reserve[0], reserve.size());

    u_result ans;
    {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_LIDAR_CONF, &query, sizeof(query)))) {
            return ans;
        }

        // waiting for confirmation
        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_GET_LIDAR_CONF) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if (header_size < sizeof(type)) {
            return RESULT_INVALID_DATA;
        }

        // TODO
        // if (!_chanDev->waitfordata(header_size, timeout)) {
        //     return RESULT_OPERATION_TIMEOUT;
        // }

        std::vector<_u8> dataBuf;
        // TODO
        dataBuf.resize(header_size);
        // _chanDev->recvdata(reinterpret_cast<_u8 *>(&dataBuf[0]), header_size);

        _u8 *infobuf = (_u8*)&dataBuf;
        _u32 remainingtime;
        _u32 currentTs = millis();
        _u8  recvPos = 0;
        while ((remainingtime=millis() - currentTs) <= timeout) {
            int currentbyte = lidarSerial.read();
            if (currentbyte<0) continue;    
            infobuf[recvPos++] = currentbyte;

            if (recvPos == header_size) {
                break;
            }
        }
        Serial.println("z");

        //check if returned type is same as asked type
        // _u32 replyType = -1;
        // memcpy(&replyType, &dataBuf[0], sizeof(type));
        // if (replyType != type) {
        //     return RESULT_INVALID_DATA;
        // }
        Serial.println("x");

        //copy all the payload into &outputBuf
        int payLoadLen = header_size - sizeof(type);

        //do consistency check
        if (payLoadLen <= 0) {
            return RESULT_INVALID_DATA;
        }
        //copy all payLoadLen bytes to outputBuf
        outputBuf.resize(payLoadLen);
        memcpy(&outputBuf[0], &dataBuf[0] + sizeof(type), payLoadLen);
        Serial.println("y");
    }
    return ans;
}

// TODO not implemented
u_result RPLidar::getTypicalScanMode(_u16& outMode, _u32 timeoutInMs)
{
    u_result ans;
    std::vector<_u8> answer;
    bool lidarSupportConfigCmds = false;
    ans = checkSupportConfigCommands(lidarSupportConfigCmds);
    if (IS_FAIL(ans)) return RESULT_INVALID_DATA;

    if (lidarSupportConfigCmds)
    {
        ans = getLidarConf(RPLIDAR_CONF_SCAN_MODE_TYPICAL, answer, std::vector<_u8>(), timeoutInMs);
        if (IS_FAIL(ans)) {
            return ans;
        }
        if (answer.size() < sizeof(_u16)) {
            return RESULT_INVALID_DATA;
        }

        const _u16 *p_answer = reinterpret_cast<const _u16*>(&answer[0]);
        outMode = *p_answer;
        return ans;
    }
    //old version of triangle lidar
    else
    {
        outMode = RPLIDAR_CONF_SCAN_COMMAND_EXPRESS;
        return ans;
    }
    return ans;
}

// TODO not implemented
u_result RPLidar::getLidarSampleDuration(float& sampleDurationRes, _u16 scanModeID, _u32 timeoutInMs)
{
    u_result ans;
    std::vector<_u8> reserve(2);
    memcpy(&reserve[0], &scanModeID, sizeof(scanModeID));

    std::vector<_u8> answer;
    ans = getLidarConf(RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE, answer, reserve, timeoutInMs);
    if (IS_FAIL(ans))
    {
        return ans;
    }
    if (answer.size() < sizeof(_u32))
    {
        return RESULT_INVALID_DATA;
    }
    const _u32 *result = reinterpret_cast<const _u32*>(&answer[0]);
    sampleDurationRes = (float)(*result >> 8);
    _cached_current_us_per_sample = sampleDurationRes;
    return ans;
}

// TODO not implemented
u_result RPLidar::getMaxDistance(float &maxDistance, _u16 scanModeID, _u32 timeoutInMs)
{
    u_result ans;
    std::vector<_u8> reserve(2);
    memcpy(&reserve[0], &scanModeID, sizeof(scanModeID));

    std::vector<_u8> answer;
    ans = getLidarConf(RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE, answer, reserve, timeoutInMs);
    if (IS_FAIL(ans))
    {
        return ans;
    }
    if (answer.size() < sizeof(_u32))
    {
        return RESULT_INVALID_DATA;
    }
    const _u32 *result = reinterpret_cast<const _u32*>(&answer[0]);
    maxDistance = (float)(*result >> 8);
    return ans;
}

// TODO not implemented
u_result RPLidar::getScanModeAnsType(_u8 &ansType, _u16 scanModeID, _u32 timeoutInMs)
{
    u_result ans;
    std::vector<_u8> reserve(2);
    memcpy(&reserve[0], &scanModeID, sizeof(scanModeID));

    std::vector<_u8> answer;
    ans = getLidarConf(RPLIDAR_CONF_SCAN_MODE_ANS_TYPE, answer, reserve, timeoutInMs);
    if (IS_FAIL(ans))
    {
        return ans;
    }
    if (answer.size() < sizeof(_u8))
    {
        return RESULT_INVALID_DATA;
    }
    const _u8 *result = reinterpret_cast<const _u8*>(&answer[0]);
    ansType = *result;
    return ans;
}

// TODO not implemented
u_result RPLidar::getScanModeName(char* modeName, _u16 scanModeID, _u32 timeoutInMs)
{
    u_result ans;
    std::vector<_u8> reserve(2);
    memcpy(&reserve[0], &scanModeID, sizeof(scanModeID));

    std::vector<_u8> answer;
    ans = getLidarConf(RPLIDAR_CONF_SCAN_MODE_NAME, answer, reserve, timeoutInMs);
    if (IS_FAIL(ans))
    {
        return ans;
    }
    int len = answer.size();
    if (0 == len) return RESULT_INVALID_DATA;
    memcpy(modeName, &answer[0], len);
    return ans;
}

// TODO not implemented
u_result RPLidar::getAllSupportedScanModes(std::vector<RplidarScanMode>& outModes, _u32 timeoutInMs)
{
    u_result ans;
    bool confProtocolSupported = false;
    ans = checkSupportConfigCommands(confProtocolSupported);
    if (IS_FAIL(ans)) return RESULT_INVALID_DATA;

    if (confProtocolSupported)
    {
        // 1. get scan mode count
        _u16 modeCount;
        ans = getScanModeCount(modeCount);
        if (IS_FAIL(ans))
        {
            return RESULT_INVALID_DATA;
        }
        // 2. for loop to get all fields of each scan mode
        for (_u16 i = 0; i < modeCount; i++)
        {
            Serial.println("_");
            RplidarScanMode scanModeInfoTmp;
            memset(&scanModeInfoTmp, 0, sizeof(scanModeInfoTmp));
            scanModeInfoTmp.id = i;
            ans = getLidarSampleDuration(scanModeInfoTmp.us_per_sample, i);
            if (IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }
            ans = getMaxDistance(scanModeInfoTmp.max_distance, i);
            if (IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }
            ans = getScanModeAnsType(scanModeInfoTmp.ans_type, i);
            if (IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }
            ans = getScanModeName(scanModeInfoTmp.scan_mode, i);
            if (IS_FAIL(ans))
            {
                return RESULT_INVALID_DATA;
            }
            outModes.push_back(scanModeInfoTmp);
        }
        return ans;
    }
    else
    {
        rplidar_response_sample_rate_t sampleRateTmp;
        ans = getSampleDuration_uS(sampleRateTmp);
        if (IS_FAIL(ans)) return RESULT_INVALID_DATA;
        //judge if support express scan
        bool ifSupportExpScan = false;
        ans = checkExpressScanSupported(ifSupportExpScan);
        if (IS_FAIL(ans)) return RESULT_INVALID_DATA;

        RplidarScanMode stdScanModeInfo;
        stdScanModeInfo.id = RPLIDAR_CONF_SCAN_COMMAND_STD;
        stdScanModeInfo.us_per_sample = sampleRateTmp.std_sample_duration_us;
        stdScanModeInfo.max_distance = 16;
        stdScanModeInfo.ans_type = RPLIDAR_ANS_TYPE_MEASUREMENT;
        strcpy(stdScanModeInfo.scan_mode, "Standard");
        outModes.push_back(stdScanModeInfo);
        if (ifSupportExpScan)
        {
            RplidarScanMode expScanModeInfo;
            expScanModeInfo.id = RPLIDAR_CONF_SCAN_COMMAND_EXPRESS;
            expScanModeInfo.us_per_sample = sampleRateTmp.express_sample_duration_us;
            expScanModeInfo.max_distance = 16;
            expScanModeInfo.ans_type = RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED;
            strcpy(expScanModeInfo.scan_mode, "Express");
            outModes.push_back(expScanModeInfo);
        }
        return ans;
    }
    return ans;
}

// TODO not implemented
u_result RPLidar::getScanModeCount(_u16& modeCount, _u32 timeoutInMs)
{
    u_result ans;
    std::vector<_u8> answer;
    ans = getLidarConf(RPLIDAR_CONF_SCAN_MODE_COUNT, answer, std::vector<_u8>(), timeoutInMs);

    if (IS_FAIL(ans)) {
        return ans;
    }
    if (answer.size() < sizeof(_u16)) {
        return RESULT_INVALID_DATA;
    }
    const _u16 *p_answer = reinterpret_cast<const _u16*>(&answer[0]);
    modeCount = *p_answer;

    return ans;
}
#endif

u_result RPLidar::startScanExpress(bool force, _u16 scanMode, _u32 options, RplidarScanMode* outUsedScanMode, _u32 timeout)
{
    u_result ans;
    if (!isConnected()) return RESULT_OPERATION_FAIL;
    if (_isScanning) return RESULT_ALREADY_DONE;

    stop(); //force the previous operation to stop

    if (scanMode == RPLIDAR_CONF_SCAN_COMMAND_STD)
    {
        return startScanNormal(force);
    }

    
    //get scan answer type to specify how to wait data
    _u8 scanAnsType;
    scanAnsType = RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED;

    {
        rplidar_payload_express_scan_t scanReq;
        memset(&scanReq, 0, sizeof(scanReq));
        if (scanMode != RPLIDAR_CONF_SCAN_COMMAND_STD && scanMode != RPLIDAR_CONF_SCAN_COMMAND_EXPRESS)
            scanReq.working_mode = _u8(scanMode);
        scanReq.working_flags = options;

        char report[30];
        snprintf(report, sizeof(report), "%d", scanMode);
        Serial.println(report);

        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_EXPRESS_SCAN, &scanReq, sizeof(scanReq)))) {
            return ans;
        }

        // waiting for confirmation
        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != scanAnsType) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);

        if (scanAnsType == RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED)
        {
            if (header_size < sizeof(rplidar_response_capsule_measurement_nodes_t)) {
                return RESULT_INVALID_DATA;
            }
            _cached_express_flag = 0;
            _isScanning = true;
            // _cachethread = CLASS_THREAD(RPLidar, _cacheCapsuledScanData);
        }
        else if (scanAnsType == RPLIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED)
        {
            if (header_size < sizeof(rplidar_response_capsule_measurement_nodes_t)) {
                return RESULT_INVALID_DATA;
            }
            _cached_express_flag = 1;
            _isScanning = true;
            // _cachethread = CLASS_THREAD(RPLidar, _cacheCapsuledScanData);
        }
        else if (scanAnsType == RPLIDAR_ANS_TYPE_MEASUREMENT_HQ) {
            if (header_size < sizeof(rplidar_response_hq_capsule_measurement_nodes_t)) {
                return RESULT_INVALID_DATA;
            }
            _isScanning = true;
            // _cachethread = CLASS_THREAD(RPLidar, _cacheHqScanData);
        }
        else
        {
            if (header_size < sizeof(rplidar_response_ultra_capsule_measurement_nodes_t)) {
                return RESULT_INVALID_DATA;
            }
            _isScanning = true;
            return RESULT_INVALID_DATA;
            Serial.println("Not implemented");
        }
    }
    return RESULT_OK;
}

u_result RPLidar::stop(_u32 timeout)
{
    u_result ans;
    _disableDataGrabbing();

    {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_STOP))) {
            return ans;
        }
    }
    return RESULT_OK;
}

#if 0
u_result RPLidar::grabScanData(rplidar_response_measurement_node_t * nodebuffer, size_t & count, _u32 timeout)
{
    DEPRECATED_WARN("grabScanData()", "grabScanDataHq()");

    switch (_dataEvt.wait(timeout))
    {
    case rp::hal::Event::EVENT_TIMEOUT:
        count = 0;
        return RESULT_OPERATION_TIMEOUT;
    case rp::hal::Event::EVENT_OK:
        {
            if(_cached_scan_node_hq_count == 0) return RESULT_OPERATION_TIMEOUT; //consider as timeout

            rp::hal::AutoLocker l(_lock);

            size_t size_to_copy = min(count, _cached_scan_node_hq_count);

            for (size_t i = 0; i < size_to_copy; i++)
                convert(_cached_scan_node_hq_buf[i], nodebuffer[i]);

            count = size_to_copy;
            _cached_scan_node_hq_count = 0;
        }
        return RESULT_OK;

    default:
        count = 0;
        return RESULT_OPERATION_FAIL;
    }
}

u_result RPLidar::grabScanDataHq(rplidar_response_measurement_node_hq_t * nodebuffer, size_t & count, _u32 timeout)
{
    switch (_dataEvt.wait(timeout))
    {
    case rp::hal::Event::EVENT_TIMEOUT:
        count = 0;
        return RESULT_OPERATION_TIMEOUT;
    case rp::hal::Event::EVENT_OK:
    {
        if (_cached_scan_node_hq_count == 0) return RESULT_OPERATION_TIMEOUT; //consider as timeout

        rp::hal::AutoLocker l(_lock);

        size_t size_to_copy = min(count, _cached_scan_node_hq_count);
        memcpy(nodebuffer, _cached_scan_node_hq_buf, size_to_copy * sizeof(rplidar_response_measurement_node_hq_t));

        count = size_to_copy;
        _cached_scan_node_hq_count = 0;
    }
    return RESULT_OK;

    default:
        count = 0;
        return RESULT_OPERATION_FAIL;
    }
}

u_result RPLidar::getScanDataWithInterval(rplidar_response_measurement_node_t * nodebuffer, size_t & count)
{
    DEPRECATED_WARN("getScanDataWithInterval(rplidar_response_measurement_node_t*, size_t&)", "getScanDataWithInterval(rplidar_response_measurement_node_hq_t*, size_t&)");

    size_t size_to_copy = 0;
    {
        rp::hal::AutoLocker l(_lock);
        if(_cached_scan_node_hq_count_for_interval_retrieve == 0)
        {
            return RESULT_OPERATION_TIMEOUT; 
        }
        //copy all the nodes(_cached_scan_node_count_for_interval_retrieve nodes) in _cached_scan_node_buf_for_interval_retrieve
        size_to_copy = _cached_scan_node_hq_count_for_interval_retrieve;
        for (size_t i = 0; i < size_to_copy; i++)
        {
            convert(_cached_scan_node_hq_buf_for_interval_retrieve[i], nodebuffer[i]);
        }
        _cached_scan_node_hq_count_for_interval_retrieve = 0;
    }
    count = size_to_copy;

    return RESULT_OK;
}
#endif

#if 0
u_result RPLidar::getScanDataWithIntervalHq(rplidar_response_measurement_node_hq_t * nodebuffer, size_t & count)
{
    size_t size_to_copy = 0;
    // Prevent crash in case lidar is not scanning - that way this function will leave nodebuffer untouched and set
    // count to 0.
    if (_isScanning)
    {
        if (_cached_scan_node_hq_count_for_interval_retrieve == 0)
        {
            return RESULT_OPERATION_TIMEOUT;
        }
        // Copy at most count nodes from _cached_scan_node_buf_for_interval_retrieve
        size_to_copy = min(_cached_scan_node_hq_count_for_interval_retrieve, count);
        memcpy(nodebuffer, _cached_scan_node_hq_buf_for_interval_retrieve, size_to_copy * sizeof(rplidar_response_measurement_node_hq_t));
        _cached_scan_node_hq_count_for_interval_retrieve -= size_to_copy;
        // Move remaining data to the start of the array.
        memmove(&_cached_scan_node_hq_buf_for_interval_retrieve[0], &_cached_scan_node_hq_buf_for_interval_retrieve[size_to_copy], _cached_scan_node_hq_count_for_interval_retrieve *  sizeof(rplidar_response_measurement_node_hq_t));
    }
    count = size_to_copy;

	// If there is remaining data, return with a warning.
    // TODO
	// if (_cached_scan_node_hq_count_for_interval_retrieve > 0)
	// 	return RESULT_REMAINING_DATA;
    return RESULT_OK;
}
#endif

static inline float getAngle(const rplidar_response_measurement_node_t& node)
{
    return (node.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.f;
}

static inline void setAngle(rplidar_response_measurement_node_t& node, float v)
{
    _u16 checkbit = node.angle_q6_checkbit & RPLIDAR_RESP_MEASUREMENT_CHECKBIT;
    node.angle_q6_checkbit = (((_u16)(v * 64.0f)) << RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) | checkbit;
}

static inline float getAngle(const rplidar_response_measurement_node_hq_t& node)
{
    return node.angle_z_q14 * 90.f / 16384.f;
}

static inline void setAngle(rplidar_response_measurement_node_hq_t& node, float v)
{
    node.angle_z_q14 = _u32(v * 16384.f / 90.f);
}

static inline _u16 getDistanceQ2(const rplidar_response_measurement_node_t& node)
{
    return node.distance_q2;
}

static inline _u32 getDistanceQ2(const rplidar_response_measurement_node_hq_t& node)
{
    return node.dist_mm_q2;
}

u_result RPLidar::_sendCommand(_u8 cmd, const void * payload, size_t payloadsize)
{
    _u8 pkt_header[10];
    rplidar_cmd_packet_t * header = reinterpret_cast<rplidar_cmd_packet_t * >(pkt_header);
    _u8 checksum = 0;

    if (!_isConnected) return RESULT_OPERATION_FAIL;

    if (payloadsize && payload) {
        cmd |= RPLIDAR_CMDFLAG_HAS_PAYLOAD;
    }

    header->syncByte = RPLIDAR_CMD_SYNC_BYTE;
    header->cmd_flag = cmd;

    // send header first
    lidarSerial.write((uint8_t *)header, 2);

    if (cmd & RPLIDAR_CMDFLAG_HAS_PAYLOAD) {
        checksum ^= RPLIDAR_CMD_SYNC_BYTE;
        checksum ^= cmd;
        checksum ^= (payloadsize & 0xFF);

        // calc checksum
        for (size_t pos = 0; pos < payloadsize; ++pos) {
            checksum ^= ((_u8 *)payload)[pos];
        }
        // send size
        _u8 sizebyte = payloadsize;
        lidarSerial.write((uint8_t *)&sizebyte, 1);

        // send payload
        lidarSerial.write((uint8_t *)payload, sizebyte);

        // send checksum
        lidarSerial.write((uint8_t *)&checksum, 1);
    }

    return RESULT_OK;
}

u_result RPLidar::getSampleDuration_uS(rplidar_response_sample_rate_t & rateInfo, _u32 timeout)
{  
    if (!isConnected()) return RESULT_OPERATION_FAIL;
    
    _disableDataGrabbing();

    rplidar_response_device_info_t devinfo;
    // 1. fetch the device version first...
    u_result ans = getDeviceInfo(devinfo, timeout);

    rateInfo.express_sample_duration_us = _cached_sampleduration_express;
    rateInfo.std_sample_duration_us = _cached_sampleduration_std;

    if (devinfo.firmware_version < ((0x1<<8) | 17)) {
        // provide fake data...

        return RESULT_OK;
    }


    {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_SAMPLERATE))) {
            return ans;
        }

        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_SAMPLE_RATE) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if ( header_size < sizeof(rplidar_response_sample_rate_t)) {
            return RESULT_INVALID_DATA;
        }

        _u8 *infobuf = (_u8*)&rateInfo;
        _u32 remainingtime;
        _u8  recvPos = 0;
        _u32 currentTs = millis();
        while ((remainingtime=millis() - currentTs) <= timeout) {
            int currentbyte = lidarSerial.read();
            if (currentbyte<0) continue;    
            infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(rplidar_response_sample_rate_t)) {
                return RESULT_OK;
            }
        }
    }
    _cached_current_us_per_sample = rateInfo.express_sample_duration_us;
    return RESULT_OK;
}

void RPLidar::_disableDataGrabbing()
{
    _isScanning = false;
}
