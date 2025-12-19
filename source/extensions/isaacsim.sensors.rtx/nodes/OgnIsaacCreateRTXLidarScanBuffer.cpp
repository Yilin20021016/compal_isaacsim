// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include "GenericModelOutput.h"
#include "IsaacSimSensorsRTXCuda.cuh"
#include "LidarConfigHelper.h"
#include "isaacsim/core/includes/BaseResetNode.h"
#include "isaacsim/core/includes/Buffer.h"
#include "isaacsim/core/includes/ScopedCudaDevice.h"
#include "isaacsim/core/includes/UsdUtilities.h"

#include <carb/tasking/ITasking.h>

#include <OgnIsaacCreateRTXLidarScanBufferDatabase.h>
#include <cstdint>
#include <unordered_map>

namespace isaacsim
{
namespace sensors
{
namespace rtx
{

class OgnIsaacCreateRTXLidarScanBuffer : public isaacsim::core::includes::BaseResetNode
{
private:
    bool m_firstFrame{ true };
    bool m_isInitialized{ false };

    size_t m_maxPoints{ 0 };
    size_t m_currentReturnCount{ 0 };
    size_t m_currentBuffer{ 0 };
    size_t m_nextBuffer{ 1 };
    size_t m_totalElements{ 0 };

    // parallelism + inter-frame overlap
    // Basic data: current buffer streams
    static constexpr size_t STREAM_AZIMUTH_CURRENT = 0; // azimuth current buffer
    static constexpr size_t STREAM_ELEVATION_CURRENT = 1; // elevation current buffer
    static constexpr size_t STREAM_DISTANCE_CURRENT = 2; // distance current buffer
    static constexpr size_t STREAM_FLAGS_CURRENT = 3; // flags current buffer
    // Basic data: next buffer streams
    static constexpr size_t STREAM_AZIMUTH_NEXT = 4; // azimuth next buffer
    static constexpr size_t STREAM_ELEVATION_NEXT = 5; // elevation next buffer
    static constexpr size_t STREAM_DISTANCE_NEXT = 6; // distance next buffer
    static constexpr size_t STREAM_FLAGS_NEXT = 7; // flags next buffer
    // Optional data streams
    static constexpr size_t STREAM_INTENSITY = 8; // intensity data
    static constexpr size_t STREAM_TIMESTAMP = 9; // timestamp data
    static constexpr size_t STREAM_EMITTER_ID = 10; // emitter ID data
    static constexpr size_t STREAM_CHANNEL_ID = 11; // channel ID data
    static constexpr size_t STREAM_MATERIAL_ID = 12; // material ID data
    static constexpr size_t STREAM_TICK_ID = 13; // tick ID data
    static constexpr size_t STREAM_NORMAL = 14; // normal vectors
    static constexpr size_t STREAM_VELOCITY = 15; // velocity data
    static constexpr size_t STREAM_OBJECT_ID = 16; // object ID data
    static constexpr size_t STREAM_ECHO_ID = 17; // echo ID data
    static constexpr size_t STREAM_TICK_STATE = 18; // tick states data
    static constexpr size_t STREAM_RADIAL_VELOCITY_MS = 19; // radial velocity data in m/s
    // Point cloud processing stream
    static constexpr size_t STREAM_POINT_CLOUD = 20; // point cloud processing

    size_t m_numStreams{ STREAM_POINT_CLOUD + 1 }; // parallel basic data + pipeline optimization

    std::vector<cudaStream_t> m_cudaStreams; // Persistent streams
    std::vector<cudaEvent_t> m_cudaEvents; // Persistent events

    // Cached device properties
    int m_maxThreadsPerBlock{ 0 };
    int m_multiProcessorCount{ 0 };

    // CUDA graph optimization for basic data copy operations
    cudaGraph_t m_basicDataCopyGraph{};
    cudaGraphExec_t m_basicDataCopyGraphExec{};
    bool m_graphsInitialized{ false };

    // Graph node handles - runtime parameter updates (maybe not needed for all next buffers?)
    cudaGraphNode_t m_azimuthCurrentNode{};
    cudaGraphNode_t m_elevationCurrentNode{};
    cudaGraphNode_t m_distanceCurrentNode{};
    cudaGraphNode_t m_flagsCurrentNode{};
    // Separate overflow graph (next-buffer copies)
    cudaGraph_t m_overflowCopyGraph{};
    cudaGraphExec_t m_overflowCopyGraphExec{};
    cudaGraphNode_t m_azimuthNextNode{};
    cudaGraphNode_t m_elevationNextNode{};
    cudaGraphNode_t m_distanceNextNode{};
    cudaGraphNode_t m_flagsNextNode{};

    std::array<isaacsim::core::includes::DeviceBufferBase<float>, 2> azimuthBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<float>, 2> elevationBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<float>, 2> distanceBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<uint8_t>, 2> flagsBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<float>, 2> intensityBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<uint64_t>, 2> timestampBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<uint32_t>, 2> emitterIdBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<uint32_t>, 2> channelIdBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<uint32_t>, 2> materialIdBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<uint32_t>, 2> tickIdBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<float3>, 2> normalBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<float3>, 2> velocityBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<uint8_t>, 2> objectIdBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<uint8_t>, 2> echoIdBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<uint8_t>, 2> tickStateBuffers;
    std::array<isaacsim::core::includes::DeviceBufferBase<float>, 2> radialVelocityMSBuffers;

    bool m_outputAzimuth{ false };
    bool m_outputElevation{ false };
    bool m_outputDistance{ false };
    bool m_outputIntensity{ false };
    bool m_outputTimestamp{ false };
    bool m_outputEmitterId{ false };
    bool m_outputChannelId{ false };
    bool m_outputMaterialId{ false };
    bool m_outputTickId{ false };
    bool m_outputHitNormal{ false };
    bool m_outputVelocity{ false };
    bool m_outputObjectId{ false };
    bool m_outputEchoId{ false };
    bool m_outputTickState{ false };
    bool m_outputRadialVelocityMS{ false };

    isaacsim::core::includes::DeviceBufferBase<size_t> indicesBuffer;
    isaacsim::core::includes::DeviceBufferBase<size_t> indicesValidBuffer;

    // Dedicated output buffers for the valid points
    isaacsim::core::includes::DeviceBufferBase<float3> pcBufferValid;
    isaacsim::core::includes::DeviceBufferBase<float> distanceBufferValid;
    isaacsim::core::includes::DeviceBufferBase<float> intensityBufferValid;
    isaacsim::core::includes::DeviceBufferBase<float> azimuthBufferValid;
    isaacsim::core::includes::DeviceBufferBase<float> elevationBufferValid;
    isaacsim::core::includes::DeviceBufferBase<int32_t> deltaTimesBufferValid;
    isaacsim::core::includes::DeviceBufferBase<uint64_t> timestampBufferValid;
    isaacsim::core::includes::DeviceBufferBase<uint32_t> emitterIdBufferValid;
    isaacsim::core::includes::DeviceBufferBase<uint32_t> channelIdBufferValid;
    isaacsim::core::includes::DeviceBufferBase<uint32_t> materialIdBufferValid;
    isaacsim::core::includes::DeviceBufferBase<uint32_t> tickIdBufferValid;
    isaacsim::core::includes::DeviceBufferBase<float3> normalBufferValid;
    isaacsim::core::includes::DeviceBufferBase<float3> velocityBufferValid;
    isaacsim::core::includes::DeviceBufferBase<uint8_t> objectIdBufferValid;
    isaacsim::core::includes::DeviceBufferBase<uint8_t> echoIdBufferValid;
    isaacsim::core::includes::DeviceBufferBase<uint8_t> tickStateBufferValid;
    isaacsim::core::includes::DeviceBufferBase<float> radialVelocityMSBufferValid;

    // Cached temporary storage for CUB operations
    void* m_d_temp_storage{ nullptr };
    size_t m_temp_storage_bytes{ 0 };
    int m_numPoints{ 0 };

    // Cached enable masks for output selection kernels
    uint32_t m_requiredOutputsMask{ 0 };
    uint32_t m_optionalOutputsMask{ 0 };

    omni::sensors::GenericModelOutput* hostGMO{ nullptr };
    omni::sensors::LidarAuxiliaryData* hostLidarAuxPoints{ nullptr };
    omni::sensors::RadarAuxiliaryData* hostRadarAuxPoints{ nullptr };

    int* numValidPointsHost{ nullptr };
    int* numValidPointsDevice{ nullptr };

    static constexpr size_t MAX_POINTS_RADAR = 2500; // RTX Radar model limit

public:
    void reset()
    {
        m_firstFrame = true;
        m_isInitialized = false;
        m_maxPoints = 0;
        m_currentReturnCount = 0;
        m_currentBuffer = 0;
        m_nextBuffer = 1;
        m_totalElements = 0;
        m_maxThreadsPerBlock = 0;
        m_multiProcessorCount = 0;
        CUDA_CHECK(cudaFreeHost(hostGMO));
        CUDA_CHECK(cudaFreeHost(hostLidarAuxPoints));
        CUDA_CHECK(cudaFreeHost(hostRadarAuxPoints));

        // Cleanup persistent streams
        for (auto& stream : m_cudaStreams)
        {
            CUDA_CHECK(cudaStreamDestroy(stream));
        }
        m_cudaStreams.clear();

        // Cleanup persistent events
        for (auto& event : m_cudaEvents)
        {
            CUDA_CHECK(cudaEventDestroy(event));
        }
        m_cudaEvents.clear();

        // Cleanup graphs
        if (m_graphsInitialized)
        {
            CUDA_CHECK(cudaGraphExecDestroy(m_basicDataCopyGraphExec));
            CUDA_CHECK(cudaGraphDestroy(m_basicDataCopyGraph));
            if (m_overflowCopyGraphExec)
            {
                CUDA_CHECK(cudaGraphExecDestroy(m_overflowCopyGraphExec));
            }
            if (m_overflowCopyGraph)
            {
                CUDA_CHECK(cudaGraphDestroy(m_overflowCopyGraph));
            }
            m_graphsInitialized = false;
        }

        // Cleanup cached temporary storage
        if (m_d_temp_storage)
        {
            CUDA_CHECK(cudaFree(m_d_temp_storage));
            m_d_temp_storage = nullptr;
        }
        m_temp_storage_bytes = 0;
        m_numPoints = 0;

        // Reset cached enable masks
        m_requiredOutputsMask = 0;
        m_optionalOutputsMask = 0;
    }

    bool initialize(OgnIsaacCreateRTXLidarScanBufferDatabase& db)
    {
        CARB_PROFILE_ZONE(0, "[IsaacSim] IsaacCreateRTXLidarScanBuffer initialize");

        int cudaDeviceIndex = db.inputs.cudaDeviceIndex() == -1 ? 0 : db.inputs.cudaDeviceIndex();
        isaacsim::core::includes::ScopedDevice scopedDev(cudaDeviceIndex);

        // Query for device props
        try
        {
            cudaDeviceProp prop;
            CUDA_CHECK(cudaGetDeviceProperties(&prop, cudaDeviceIndex));

            // cache results
            m_maxThreadsPerBlock = prop.maxThreadsPerBlock;
            m_multiProcessorCount = prop.multiProcessorCount;
        }
        catch (const std::exception& e)
        {
            CARB_LOG_ERROR("Failed to get device properties for GPU %d: %s", cudaDeviceIndex, e.what());
            return false;
        }

        // Allocate pinned host memory
        CUDA_CHECK(cudaMallocHost(&hostGMO, sizeof(omni::sensors::GenericModelOutput)));
        CUDA_CHECK(cudaMallocHost(&hostLidarAuxPoints, sizeof(omni::sensors::LidarAuxiliaryData)));
        CUDA_CHECK(cudaMallocHost(&hostRadarAuxPoints, sizeof(omni::sensors::RadarAuxiliaryData)));
        CUDA_CHECK(cudaMallocHost(&numValidPointsHost, sizeof(int)));
        numValidPointsHost[0] = 0;

        // Get auxiliary data structure to set output flags
        CUDA_CHECK(cudaMemcpy(hostGMO, reinterpret_cast<void*>(db.inputs.dataPtr()),
                              sizeof(omni::sensors::GenericModelOutput), cudaMemcpyDeviceToHost));
        const auto auxType = hostGMO->auxType;
        const auto modality = hostGMO->modality;
        const std::string renderProductPath = std::string(db.tokenToString(db.inputs.renderProductPath()));
        const pxr::UsdPrim sensorPrim = isaacsim::core::includes::getCameraPrimFromRenderProduct(renderProductPath);
        const std::string sensorPrimPath = sensorPrim.GetPath().GetString();
        if (modality == omni::sensors::Modality::LIDAR)
        {
            if (auxType > omni::sensors::AuxType::NONE)
            {
                CUDA_CHECK(cudaMemcpy(hostLidarAuxPoints, hostGMO->auxiliaryData,
                                      sizeof(omni::sensors::LidarAuxiliaryData), cudaMemcpyDeviceToHost));
            }
            // Retrieve lidar prim from render product path, then validate its attributes
            if (renderProductPath.length() == 0)
            {
                CARB_LOG_ERROR("IsaacComputeRTXLidarFlatScan: renderProductPath input is empty. Skipping execution.");
                return false;
            }
            if (sensorPrim.IsA<pxr::UsdGeomCamera>())
            {
                CARB_LOG_WARN(
                    "RTX sensors as camera prims are deprecated as of Isaac Sim 5.0, and support will be removed in a future release. Please use an OmniLidar prim with the new OmniSensorGenericLidarCoreAPI schema.");
                LidarConfigHelper configHelper;
                configHelper.updateLidarConfig(renderProductPath.c_str());
                if (configHelper.scanType == LidarScanType::kUnknown)
                {
                    CARB_LOG_ERROR(
                        "IsaacComputeRTXLidarFlatScan: Lidar prim scanType is Unknown, and node will not execute. Stop the simulation, correct the issue, and restart.");
                    return false;
                }
                m_maxPoints = configHelper.numChannels * configHelper.maxReturns *
                              static_cast<size_t>(std::ceil(static_cast<float>(configHelper.reportRateBaseHz) /
                                                            static_cast<float>(configHelper.scanRateBaseHz)));
            }
            else
            {
                uint32_t maxReturns;
                uint32_t numChannels;
                uint32_t patternFiringRateHz;
                uint32_t scanRateBaseHz;
                sensorPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:maxReturns")).Get(&maxReturns);
                sensorPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:numberOfChannels")).Get(&numChannels);
                sensorPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:patternFiringRateHz")).Get(&patternFiringRateHz);
                sensorPrim.GetAttribute(pxr::TfToken("omni:sensor:Core:scanRateBaseHz")).Get(&scanRateBaseHz);
                m_maxPoints = numChannels * maxReturns *
                              static_cast<size_t>(std::ceil(static_cast<float>(patternFiringRateHz) /
                                                            static_cast<float>(scanRateBaseHz)));
            }
        }
        else if (modality == omni::sensors::Modality::RADAR)
        {
            if (auxType > omni::sensors::AuxType::NONE)
            {
                CUDA_CHECK(cudaMemcpy(hostRadarAuxPoints, hostGMO->auxiliaryData,
                                      sizeof(omni::sensors::RadarAuxiliaryData), cudaMemcpyDeviceToHost));
            }
            m_maxPoints = MAX_POINTS_RADAR;
        }
        else
        {
            CARB_LOG_ERROR("IsaacCreateRTXLidarScanBuffer: Invalid modality: %d, expected LIDAR or RADAR",
                           static_cast<int>(modality));
            return false;
        }

        m_outputAzimuth = db.inputs.outputAzimuth();
        m_outputElevation = db.inputs.outputElevation();
        m_outputDistance = db.inputs.outputDistance();

        // Lambda function to validate, set output flags, and allocate buffers with aux type and modality checks
        auto validateAndAllocateOutput = [&](bool inputEnabled, omni::sensors::AuxType requiredAuxType,
                                             omni::sensors::Modality requiredModality,
                                             omni::sensors::LidarAuxHas auxMember, const char* outputName,
                                             auto& deviceBufferArray, auto& validBuffer, size_t stride = 1) -> bool
        {
            if (!inputEnabled)
            {
                return false;
            }

            bool auxTypeValid = auxType >= requiredAuxType;
            bool modalityValid = modality == requiredModality;
            bool auxMemberFilled = (requiredModality == omni::sensors::Modality::LIDAR) ?
                                       ((hostLidarAuxPoints->filledAuxMembers & auxMember) == auxMember) :
                                       true;

            if (!auxTypeValid)
            {
                CARB_LOG_WARN(
                    "IsaacCreateRTXLidarScanBuffer: %s requested for sensor '%s' but auxType (%d) is insufficient (requires %d)",
                    outputName, sensorPrimPath.c_str(), static_cast<int>(auxType), static_cast<int>(requiredAuxType));
                return false;
            }

            if (!modalityValid)
            {
                CARB_LOG_WARN(
                    "IsaacCreateRTXLidarScanBuffer: %s requested for sensor '%s' but modality (%d) is incorrect (requires %d)",
                    outputName, sensorPrimPath.c_str(), static_cast<int>(modality), static_cast<int>(requiredModality));
                return false;
            }

            if (requiredModality == omni::sensors::Modality::LIDAR && !auxMemberFilled)
            {
                return false;
            }

            // Allocate device buffers (double buffering)
            for (size_t i = 0; i < 2; ++i)
            {
                deviceBufferArray[i].setDevice(cudaDeviceIndex);
                deviceBufferArray[i].resize(m_maxPoints * stride);
            }

            // Allocate valid output buffer
            validBuffer.setDevice(cudaDeviceIndex);
            validBuffer.resize(m_maxPoints * stride);

            return true;
        };

        m_outputIntensity = validateAndAllocateOutput(db.inputs.outputIntensity(), omni::sensors::AuxType::NONE,
                                                      modality, omni::sensors::LidarAuxHas::NONE, "outputIntensity",
                                                      intensityBuffers, intensityBufferValid);
        m_outputTimestamp = validateAndAllocateOutput(db.inputs.outputTimestamp(), omni::sensors::AuxType::NONE,
                                                      modality, omni::sensors::LidarAuxHas::NONE, "outputTimestamp",
                                                      timestampBuffers, timestampBufferValid);
        m_outputEmitterId = validateAndAllocateOutput(
            db.inputs.outputEmitterId(), omni::sensors::AuxType::BASIC, omni::sensors::Modality::LIDAR,
            omni::sensors::LidarAuxHas::EMITTER_ID, "outputEmitterId", emitterIdBuffers, emitterIdBufferValid);
        m_outputChannelId = validateAndAllocateOutput(
            db.inputs.outputChannelId(), omni::sensors::AuxType::BASIC, omni::sensors::Modality::LIDAR,
            omni::sensors::LidarAuxHas::CHANNEL_ID, "outputChannelId", channelIdBuffers, channelIdBufferValid);
        m_outputMaterialId = validateAndAllocateOutput(
            db.inputs.outputMaterialId(), omni::sensors::AuxType::EXTRA, omni::sensors::Modality::LIDAR,
            omni::sensors::LidarAuxHas::MAT_ID, "outputMaterialId", materialIdBuffers, materialIdBufferValid);
        m_outputTickId = validateAndAllocateOutput(db.inputs.outputTickId(), omni::sensors::AuxType::BASIC,
                                                   omni::sensors::Modality::LIDAR, omni::sensors::LidarAuxHas::TICK_ID,
                                                   "outputTickId", tickIdBuffers, tickIdBufferValid);
        m_outputHitNormal = validateAndAllocateOutput(
            db.inputs.outputHitNormal(), omni::sensors::AuxType::FULL, omni::sensors::Modality::LIDAR,
            omni::sensors::LidarAuxHas::HIT_NORMALS, "outputHitNormal", normalBuffers, normalBufferValid);
        m_outputVelocity = validateAndAllocateOutput(
            db.inputs.outputVelocity(), omni::sensors::AuxType::FULL, omni::sensors::Modality::LIDAR,
            omni::sensors::LidarAuxHas::VELOCITIES, "outputVelocity", velocityBuffers, velocityBufferValid);
        m_outputObjectId = validateAndAllocateOutput(db.inputs.outputObjectId(), omni::sensors::AuxType::EXTRA,
                                                     omni::sensors::Modality::LIDAR, omni::sensors::LidarAuxHas::OBJ_ID,
                                                     "outputObjectId", objectIdBuffers, objectIdBufferValid, 16);
        m_outputEchoId = validateAndAllocateOutput(db.inputs.outputEchoId(), omni::sensors::AuxType::BASIC,
                                                   omni::sensors::Modality::LIDAR, omni::sensors::LidarAuxHas::ECHO_ID,
                                                   "outputEchoId", echoIdBuffers, echoIdBufferValid);
        m_outputTickState = validateAndAllocateOutput(
            db.inputs.outputTickState(), omni::sensors::AuxType::BASIC, omni::sensors::Modality::LIDAR,
            omni::sensors::LidarAuxHas::TICK_STATES, "outputTickState", tickStateBuffers, tickStateBufferValid);
        m_outputRadialVelocityMS =
            validateAndAllocateOutput(db.inputs.outputRadialVelocityMS(), omni::sensors::AuxType::BASIC,
                                      omni::sensors::Modality::RADAR, omni::sensors::LidarAuxHas::NONE,
                                      "outputRadialVelocityMS", radialVelocityMSBuffers, radialVelocityMSBufferValid);

        // Initialize cached enable masks for output selection kernels
        m_requiredOutputsMask = 0;
        if (m_outputAzimuth)
            m_requiredOutputsMask |= 1; // bit 0: azimuth
        if (m_outputElevation)
            m_requiredOutputsMask |= 2; // bit 1: elevation
        if (m_outputDistance)
            m_requiredOutputsMask |= 4; // bit 2: distance
        if (m_outputIntensity)
            m_requiredOutputsMask |= 8; // bit 3: intensity

        m_optionalOutputsMask = 0;
        if (m_outputTimestamp)
            m_optionalOutputsMask |= (1 << 4); // bit 4: timestamp
        if (m_outputEmitterId)
            m_optionalOutputsMask |= (1 << 5); // bit 5: emitter ID
        if (m_outputChannelId)
            m_optionalOutputsMask |= (1 << 6); // bit 6: channel ID
        if (m_outputMaterialId)
            m_optionalOutputsMask |= (1 << 7); // bit 7: material ID
        if (m_outputTickId)
            m_optionalOutputsMask |= (1 << 8); // bit 8: tick ID
        if (m_outputHitNormal)
            m_optionalOutputsMask |= (1 << 9); // bit 9: normal
        if (m_outputVelocity)
            m_optionalOutputsMask |= (1 << 10); // bit 10: velocity
        if (m_outputObjectId)
            m_optionalOutputsMask |= (1 << 11); // bit 11: object ID
        if (m_outputEchoId)
            m_optionalOutputsMask |= (1 << 12); // bit 12: echo ID
        if (m_outputTickState)
            m_optionalOutputsMask |= (1 << 13); // bit 13: tick states
        if (m_outputRadialVelocityMS)
            m_optionalOutputsMask |= (1 << 14); // bit 14: radial velocity MS

        // Allocate additional device memory for the number of valid points
        CUDA_CHECK(cudaMalloc(&numValidPointsDevice, sizeof(int)));

        // Allocate device buffers necessary for the point cloud kernel (azimuth, elevation, distance, flags always
        // needed)
        for (size_t i = 0; i < 2; ++i)
        {
            azimuthBuffers[i].setDevice(cudaDeviceIndex);
            azimuthBuffers[i].resize(m_maxPoints);
            elevationBuffers[i].setDevice(cudaDeviceIndex);
            elevationBuffers[i].resize(m_maxPoints);
            distanceBuffers[i].setDevice(cudaDeviceIndex);
            distanceBuffers[i].resize(m_maxPoints);
            flagsBuffers[i].setDevice(cudaDeviceIndex);
            flagsBuffers[i].resize(m_maxPoints);
        }

        // Allocate any necessary output buffers (point cloud buffer always needed, others handled by lambda)
        pcBufferValid.setDevice(cudaDeviceIndex);
        pcBufferValid.resize(m_maxPoints);
        if (m_outputAzimuth)
        {
            azimuthBufferValid.setDevice(cudaDeviceIndex);
            azimuthBufferValid.resize(m_maxPoints);
        }
        if (m_outputElevation)
        {
            elevationBufferValid.setDevice(cudaDeviceIndex);
            elevationBufferValid.resize(m_maxPoints);
        }
        if (m_outputDistance)
        {
            distanceBufferValid.setDevice(cudaDeviceIndex);
            distanceBufferValid.resize(m_maxPoints);
        }

        indicesBuffer.setDevice(cudaDeviceIndex);
        indicesBuffer.resize(m_maxPoints);
        indicesValidBuffer.setDevice(cudaDeviceIndex);
        indicesValidBuffer.resize(m_maxPoints);

        // Init persistent CUDA streams
        m_cudaStreams.resize(m_numStreams);
        for (size_t i = 0; i < m_numStreams; ++i)
        {
            CUDA_CHECK(cudaStreamCreate(&m_cudaStreams[i]));
        }

        // Init persistent CUDA events
        m_cudaEvents.resize(m_numStreams);
        for (size_t i = 0; i < m_numStreams; ++i)
        {
            CUDA_CHECK(cudaEventCreate(&m_cudaEvents[i]));
        }

        fillIndices(indicesBuffer.data(), m_maxPoints, m_maxThreadsPerBlock, cudaDeviceIndex);

        // init cuda graphs for repeated ops
        if (!initializeCudaGraphs(cudaDeviceIndex))
        {
            CARB_LOG_WARN("Failed to initialize CUDA graph, falling back to individual operations");
        }

        // Pre-allocate temporary storage for CUB operations - dynamic allocation not supported on vGPU
        m_d_temp_storage = nullptr;
        m_temp_storage_bytes = getTempStorageSizeForValidIndices(m_maxPoints, cudaDeviceIndex);

        // Allocate temporary storage based on max points
        CUDA_CHECK(cudaMalloc(&m_d_temp_storage, m_temp_storage_bytes));
        m_numPoints = static_cast<int>(m_maxPoints); // Cache the max points

        return true;
    }

    bool initializeCudaGraphs(int cudaDeviceIndex)
    {
        if (m_graphsInitialized)
            return true;

        isaacsim::core::includes::ScopedDevice scopedDev(cudaDeviceIndex);

        try
        {
            // Create CUDA graph for standard data copy operations
            CUDA_CHECK(cudaGraphCreate(&m_basicDataCopyGraph, 0));

            // Create properly configured 3D memcpy nodes for 1D operations
            cudaMemcpy3DParms copyParams = {};
            copyParams.srcPos = make_cudaPos(0, 0, 0);
            copyParams.dstPos = make_cudaPos(0, 0, 0);
            copyParams.kind = cudaMemcpyDeviceToDevice;

            // Current buffer nodes - use placeholder parameters that will be updated at runtime
            copyParams.srcPtr =
                make_cudaPitchedPtr(azimuthBuffers[m_currentBuffer].data(), m_maxPoints * sizeof(float), m_maxPoints, 1);
            copyParams.dstPtr =
                make_cudaPitchedPtr(azimuthBuffers[m_currentBuffer].data(), m_maxPoints * sizeof(float), m_maxPoints, 1);
            copyParams.extent = make_cudaExtent(m_maxPoints * sizeof(float), 1, 1); // Width in bytes for proper 1D copy
            CUDA_CHECK(cudaGraphAddMemcpyNode(&m_azimuthCurrentNode, m_basicDataCopyGraph, nullptr, 0, &copyParams));

            copyParams.srcPtr = make_cudaPitchedPtr(
                elevationBuffers[m_currentBuffer].data(), m_maxPoints * sizeof(float), m_maxPoints, 1);
            copyParams.dstPtr = make_cudaPitchedPtr(
                elevationBuffers[m_currentBuffer].data(), m_maxPoints * sizeof(float), m_maxPoints, 1);
            CUDA_CHECK(cudaGraphAddMemcpyNode(&m_elevationCurrentNode, m_basicDataCopyGraph, nullptr, 0, &copyParams));

            copyParams.srcPtr = make_cudaPitchedPtr(
                distanceBuffers[m_currentBuffer].data(), m_maxPoints * sizeof(float), m_maxPoints, 1);
            copyParams.dstPtr = make_cudaPitchedPtr(
                distanceBuffers[m_currentBuffer].data(), m_maxPoints * sizeof(float), m_maxPoints, 1);
            CUDA_CHECK(cudaGraphAddMemcpyNode(&m_distanceCurrentNode, m_basicDataCopyGraph, nullptr, 0, &copyParams));

            // Flags current - uint8_t data
            copyParams.srcPtr =
                make_cudaPitchedPtr(flagsBuffers[m_currentBuffer].data(), m_maxPoints * sizeof(uint8_t), m_maxPoints, 1);
            copyParams.dstPtr =
                make_cudaPitchedPtr(flagsBuffers[m_currentBuffer].data(), m_maxPoints * sizeof(uint8_t), m_maxPoints, 1);
            copyParams.extent = make_cudaExtent(m_maxPoints * sizeof(uint8_t), 1, 1); // Width in bytes
            CUDA_CHECK(cudaGraphAddMemcpyNode(&m_flagsCurrentNode, m_basicDataCopyGraph, nullptr, 0, &copyParams));

            // Instantiate prepared graph for current copies
            CUDA_CHECK(cudaGraphInstantiate(&m_basicDataCopyGraphExec, m_basicDataCopyGraph, nullptr, nullptr, 0));

            // Create a separate CUDA graph for overflow (next-buffer) copies
            CUDA_CHECK(cudaGraphCreate(&m_overflowCopyGraph, 0));
            cudaMemcpy3DParms overflowParams = {};
            overflowParams.srcPos = make_cudaPos(0, 0, 0);
            overflowParams.dstPos = make_cudaPos(0, 0, 0);
            overflowParams.kind = cudaMemcpyDeviceToDevice;

            // Next buffer nodes in separate graph
            overflowParams.srcPtr =
                make_cudaPitchedPtr(azimuthBuffers[m_nextBuffer].data(), m_maxPoints * sizeof(float), m_maxPoints, 1);
            overflowParams.dstPtr =
                make_cudaPitchedPtr(azimuthBuffers[m_nextBuffer].data(), m_maxPoints * sizeof(float), m_maxPoints, 1);
            overflowParams.extent = make_cudaExtent(m_maxPoints * sizeof(float), 1, 1);
            CUDA_CHECK(cudaGraphAddMemcpyNode(&m_azimuthNextNode, m_overflowCopyGraph, nullptr, 0, &overflowParams));

            overflowParams.srcPtr =
                make_cudaPitchedPtr(elevationBuffers[m_nextBuffer].data(), m_maxPoints * sizeof(float), m_maxPoints, 1);
            overflowParams.dstPtr =
                make_cudaPitchedPtr(elevationBuffers[m_nextBuffer].data(), m_maxPoints * sizeof(float), m_maxPoints, 1);
            CUDA_CHECK(cudaGraphAddMemcpyNode(&m_elevationNextNode, m_overflowCopyGraph, nullptr, 0, &overflowParams));

            overflowParams.srcPtr =
                make_cudaPitchedPtr(distanceBuffers[m_nextBuffer].data(), m_maxPoints * sizeof(float), m_maxPoints, 1);
            overflowParams.dstPtr =
                make_cudaPitchedPtr(distanceBuffers[m_nextBuffer].data(), m_maxPoints * sizeof(float), m_maxPoints, 1);
            CUDA_CHECK(cudaGraphAddMemcpyNode(&m_distanceNextNode, m_overflowCopyGraph, nullptr, 0, &overflowParams));

            overflowParams.srcPtr =
                make_cudaPitchedPtr(flagsBuffers[m_nextBuffer].data(), m_maxPoints * sizeof(uint8_t), m_maxPoints, 1);
            overflowParams.dstPtr =
                make_cudaPitchedPtr(flagsBuffers[m_nextBuffer].data(), m_maxPoints * sizeof(uint8_t), m_maxPoints, 1);
            overflowParams.extent = make_cudaExtent(m_maxPoints * sizeof(uint8_t), 1, 1);
            CUDA_CHECK(cudaGraphAddMemcpyNode(&m_flagsNextNode, m_overflowCopyGraph, nullptr, 0, &overflowParams));

            CUDA_CHECK(cudaGraphInstantiate(&m_overflowCopyGraphExec, m_overflowCopyGraph, nullptr, nullptr, 0));

            m_graphsInitialized = true;
            CARB_LOG_INFO("CUDA graphs initialized successfully for lidar data copy operations");
            return true;
        }
        catch (...)
        {
            // Cleanup on failure
            if (m_basicDataCopyGraphExec)
            {
                cudaGraphExecDestroy(m_basicDataCopyGraphExec);
                m_basicDataCopyGraphExec = {};
            }
            if (m_basicDataCopyGraph)
            {
                cudaGraphDestroy(m_basicDataCopyGraph);
                m_basicDataCopyGraph = {};
            }
            if (m_overflowCopyGraphExec)
            {
                cudaGraphExecDestroy(m_overflowCopyGraphExec);
                m_overflowCopyGraphExec = {};
            }
            if (m_overflowCopyGraph)
            {
                cudaGraphDestroy(m_overflowCopyGraph);
                m_overflowCopyGraph = {};
            }
            return false;
        }
    }

    bool updateCurrentGraphParameters(const omni::sensors::GenericModelOutput* hostGMO,
                                      size_t numElementsToCopyToCurrentBuffer,
                                      size_t startIndex)
    {
        if (!m_graphsInitialized)
            return false;

        try
        {
            // Use properly configured 3D memcpy params for 1D operations
            cudaMemcpy3DParms currentParams = {};
            currentParams.srcPos = make_cudaPos(0, 0, 0);
            currentParams.dstPos = make_cudaPos(0, 0, 0);
            currentParams.kind = cudaMemcpyDeviceToDevice;

            if (numElementsToCopyToCurrentBuffer > 0)
            {
                size_t copyByteSize = numElementsToCopyToCurrentBuffer * sizeof(float);
                size_t startByteOffset = startIndex * sizeof(float);

                // Azimuth current
                currentParams.srcPtr = make_cudaPitchedPtr((void*)hostGMO->elements.x, copyByteSize, copyByteSize, 1);
                currentParams.dstPtr =
                    make_cudaPitchedPtr((void*)((char*)azimuthBuffers[m_currentBuffer].data() + startByteOffset),
                                        m_maxPoints * sizeof(float), m_maxPoints * sizeof(float), 1);
                currentParams.extent = make_cudaExtent(copyByteSize, 1, 1);
                CUDA_CHECK(
                    cudaGraphExecMemcpyNodeSetParams(m_basicDataCopyGraphExec, m_azimuthCurrentNode, &currentParams));

                // Elevation current
                currentParams.srcPtr = make_cudaPitchedPtr((void*)hostGMO->elements.y, copyByteSize, copyByteSize, 1);
                currentParams.dstPtr =
                    make_cudaPitchedPtr((void*)((char*)elevationBuffers[m_currentBuffer].data() + startByteOffset),
                                        m_maxPoints * sizeof(float), m_maxPoints * sizeof(float), 1);
                CUDA_CHECK(
                    cudaGraphExecMemcpyNodeSetParams(m_basicDataCopyGraphExec, m_elevationCurrentNode, &currentParams));

                // Distance current
                currentParams.srcPtr = make_cudaPitchedPtr((void*)hostGMO->elements.z, copyByteSize, copyByteSize, 1);
                currentParams.dstPtr =
                    make_cudaPitchedPtr((void*)((char*)distanceBuffers[m_currentBuffer].data() + startByteOffset),
                                        m_maxPoints * sizeof(float), m_maxPoints * sizeof(float), 1);
                CUDA_CHECK(
                    cudaGraphExecMemcpyNodeSetParams(m_basicDataCopyGraphExec, m_distanceCurrentNode, &currentParams));

                // Flags current (uint8_t)
                size_t flagsCopyByteSize = numElementsToCopyToCurrentBuffer * sizeof(uint8_t);
                size_t flagsStartByteOffset = startIndex * sizeof(uint8_t);
                currentParams.srcPtr =
                    make_cudaPitchedPtr((void*)hostGMO->elements.flags, flagsCopyByteSize, flagsCopyByteSize, 1);
                currentParams.dstPtr =
                    make_cudaPitchedPtr((void*)((char*)flagsBuffers[m_currentBuffer].data() + flagsStartByteOffset),
                                        m_maxPoints * sizeof(uint8_t), m_maxPoints * sizeof(uint8_t), 1);
                currentParams.extent = make_cudaExtent(flagsCopyByteSize, 1, 1);
                CUDA_CHECK(cudaGraphExecMemcpyNodeSetParams(m_basicDataCopyGraphExec, m_flagsCurrentNode, &currentParams));
            }
            else
            {
                // Minimal valid operations to keep nodes consistent
                cudaMemcpy3DParms zeroParams = {};
                zeroParams.srcPos = make_cudaPos(0, 0, 0);
                zeroParams.dstPos = make_cudaPos(0, 0, 0);
                zeroParams.kind = cudaMemcpyDeviceToDevice;
                zeroParams.extent = make_cudaExtent(1, 1, 1); // Minimal self-copy

                zeroParams.srcPtr = make_cudaPitchedPtr(azimuthBuffers[m_currentBuffer].data(), 1, 1, 1);
                zeroParams.dstPtr = make_cudaPitchedPtr(azimuthBuffers[m_currentBuffer].data(), 1, 1, 1);
                CUDA_CHECK(cudaGraphExecMemcpyNodeSetParams(m_basicDataCopyGraphExec, m_azimuthCurrentNode, &zeroParams));

                zeroParams.srcPtr = make_cudaPitchedPtr(elevationBuffers[m_currentBuffer].data(), 1, 1, 1);
                zeroParams.dstPtr = make_cudaPitchedPtr(elevationBuffers[m_currentBuffer].data(), 1, 1, 1);
                CUDA_CHECK(
                    cudaGraphExecMemcpyNodeSetParams(m_basicDataCopyGraphExec, m_elevationCurrentNode, &zeroParams));

                zeroParams.srcPtr = make_cudaPitchedPtr(distanceBuffers[m_currentBuffer].data(), 1, 1, 1);
                zeroParams.dstPtr = make_cudaPitchedPtr(distanceBuffers[m_currentBuffer].data(), 1, 1, 1);
                CUDA_CHECK(cudaGraphExecMemcpyNodeSetParams(m_basicDataCopyGraphExec, m_distanceCurrentNode, &zeroParams));

                zeroParams.srcPtr = make_cudaPitchedPtr(flagsBuffers[m_currentBuffer].data(), 1, 1, 1);
                zeroParams.dstPtr = make_cudaPitchedPtr(flagsBuffers[m_currentBuffer].data(), 1, 1, 1);
                CUDA_CHECK(cudaGraphExecMemcpyNodeSetParams(m_basicDataCopyGraphExec, m_flagsCurrentNode, &zeroParams));
            }

            return true;
        }
        catch (...)
        {
            CARB_LOG_WARN("Failed to update CUDA current graph parameters");
            return false;
        }
    }

    bool updateOverflowGraphParameters(const omni::sensors::GenericModelOutput* hostGMO,
                                       size_t numElementsToCopyToCurrentBuffer,
                                       size_t numElementsToCopyToNextBuffer)
    {
        if (!m_graphsInitialized)
            return false;
        if (numElementsToCopyToNextBuffer == 0)
            return true; // Nothing to update

        try
        {
            cudaMemcpy3DParms nextParams = {};
            nextParams.srcPos = make_cudaPos(0, 0, 0);
            nextParams.dstPos = make_cudaPos(0, 0, 0);
            nextParams.kind = cudaMemcpyDeviceToDevice;

            size_t nextCopyByteSize = numElementsToCopyToNextBuffer * sizeof(float);
            size_t srcByteOffset = numElementsToCopyToCurrentBuffer * sizeof(float);

            // Azimuth next
            nextParams.srcPtr = make_cudaPitchedPtr(
                (void*)((char*)hostGMO->elements.x + srcByteOffset), nextCopyByteSize, nextCopyByteSize, 1);
            nextParams.dstPtr = make_cudaPitchedPtr(
                azimuthBuffers[m_nextBuffer].data(), m_maxPoints * sizeof(float), m_maxPoints * sizeof(float), 1);
            nextParams.extent = make_cudaExtent(nextCopyByteSize, 1, 1);
            CUDA_CHECK(cudaGraphExecMemcpyNodeSetParams(m_overflowCopyGraphExec, m_azimuthNextNode, &nextParams));

            // Elevation next
            nextParams.srcPtr = make_cudaPitchedPtr(
                (void*)((char*)hostGMO->elements.y + srcByteOffset), nextCopyByteSize, nextCopyByteSize, 1);
            nextParams.dstPtr = make_cudaPitchedPtr(
                elevationBuffers[m_nextBuffer].data(), m_maxPoints * sizeof(float), m_maxPoints * sizeof(float), 1);
            CUDA_CHECK(cudaGraphExecMemcpyNodeSetParams(m_overflowCopyGraphExec, m_elevationNextNode, &nextParams));

            // Distance next
            nextParams.srcPtr = make_cudaPitchedPtr(
                (void*)((char*)hostGMO->elements.z + srcByteOffset), nextCopyByteSize, nextCopyByteSize, 1);
            nextParams.dstPtr = make_cudaPitchedPtr(
                distanceBuffers[m_nextBuffer].data(), m_maxPoints * sizeof(float), m_maxPoints * sizeof(float), 1);
            CUDA_CHECK(cudaGraphExecMemcpyNodeSetParams(m_overflowCopyGraphExec, m_distanceNextNode, &nextParams));

            // Flags next (uint8_t)
            size_t flagsNextCopyByteSize = numElementsToCopyToNextBuffer * sizeof(uint8_t);
            size_t flagsSrcByteOffset = numElementsToCopyToCurrentBuffer * sizeof(uint8_t);
            nextParams.srcPtr = make_cudaPitchedPtr((void*)((char*)hostGMO->elements.flags + flagsSrcByteOffset),
                                                    flagsNextCopyByteSize, flagsNextCopyByteSize, 1);
            nextParams.dstPtr = make_cudaPitchedPtr(
                flagsBuffers[m_nextBuffer].data(), m_maxPoints * sizeof(uint8_t), m_maxPoints * sizeof(uint8_t), 1);
            nextParams.extent = make_cudaExtent(flagsNextCopyByteSize, 1, 1);
            CUDA_CHECK(cudaGraphExecMemcpyNodeSetParams(m_overflowCopyGraphExec, m_flagsNextNode, &nextParams));

            return true;
        }
        catch (...)
        {
            CARB_LOG_WARN("Failed to update CUDA overflow graph parameters");
            return false;
        }
    }


    static bool compute(OgnIsaacCreateRTXLidarScanBufferDatabase& db)
    {
        CARB_PROFILE_ZONE(0, "[IsaacSim] Create RTX Lidar Scan Buffer");
        // Enable downstream execution by default, so that downstream nodes can do any initialization
        db.outputs.exec() = kExecutionAttributeStateEnabled;
        auto& state = db.perInstanceState<OgnIsaacCreateRTXLidarScanBuffer>();

        // Set default output values
        auto& matrixOutput = *reinterpret_cast<omni::math::linalg::matrix4d*>(&db.outputs.transform());
        {
            db.outputs.dataPtr() = 0;
            db.outputs.cudaDeviceIndex() = db.inputs.cudaDeviceIndex();
            db.outputs.bufferSize() = 0;
            db.outputs.width() = 0;
            db.outputs.height() = 1;
            matrixOutput.SetIdentity();
            db.outputs.azimuthPtr() = 0;
            db.outputs.azimuthBufferSize() = 0;
            db.outputs.elevationPtr() = 0;
            db.outputs.elevationBufferSize() = 0;
            db.outputs.distancePtr() = 0;
            db.outputs.distanceBufferSize() = 0;
            db.outputs.intensityPtr() = 0;
            db.outputs.intensityBufferSize() = 0;
            db.outputs.timestampPtr() = 0;
            db.outputs.timestampBufferSize() = 0;
            db.outputs.emitterIdPtr() = 0;
            db.outputs.emitterIdBufferSize() = 0;
            db.outputs.channelIdPtr() = 0;
            db.outputs.channelIdBufferSize() = 0;
            db.outputs.materialIdPtr() = 0;
            db.outputs.materialIdBufferSize() = 0;
            db.outputs.tickIdPtr() = 0;
            db.outputs.tickIdBufferSize() = 0;
            db.outputs.hitNormalPtr() = 0;
            db.outputs.hitNormalBufferSize() = 0;
            db.outputs.velocityPtr() = 0;
            db.outputs.velocityBufferSize() = 0;
            db.outputs.objectIdPtr() = 0;
            db.outputs.objectIdBufferSize() = 0;
            db.outputs.echoIdPtr() = 0;
            db.outputs.echoIdBufferSize() = 0;
            db.outputs.tickStatePtr() = 0;
            db.outputs.tickStateBufferSize() = 0;
            db.outputs.radialVelocityMSPtr() = 0;
            db.outputs.radialVelocityMSBufferSize() = 0;
        }

        if (!db.inputs.dataPtr())
        {
            CARB_LOG_INFO("IsaacCreateRTXLidarScanBuffer: dataPtr input is empty. Skipping execution.");
            return false;
        }

        // Get the CUDA device index
        int cudaDeviceIndex = db.inputs.cudaDeviceIndex() == -1 ? 0 : db.inputs.cudaDeviceIndex();
        isaacsim::core::includes::ScopedDevice scopedDev(cudaDeviceIndex);

        if (state.m_firstFrame)
        {
            state.m_isInitialized = state.initialize(db);
            state.m_firstFrame = false;
        }

        if (!state.m_isInitialized)
        {
            CARB_LOG_INFO("IsaacCreateRTXLidarScanBuffer: Failed to initialize correctly. Skipping execution.");
            return false;
        }

        // Copy just the GMO basic structure to the host using async copy
        size_t numElements;
        omni::sensors::AuxType auxType;
        omni::sensors::Modality modality;
        {
            cudaStream_t headerStream = state.m_cudaStreams[0];
            CUDA_CHECK(cudaMemcpyAsync(state.hostGMO, reinterpret_cast<void*>(db.inputs.dataPtr()),
                                       sizeof(omni::sensors::GenericModelOutput), cudaMemcpyDeviceToHost, headerStream));
            CUDA_CHECK(cudaStreamSynchronize(headerStream));
            numElements = static_cast<size_t>(state.hostGMO->numElements);
            auxType = state.hostGMO->auxType;
            modality = state.hostGMO->modality;
        }

        if (numElements == 0)
        {
            CARB_LOG_INFO("IsaacCreateRTXLidarScanBuffer: No returns in the input buffer. Skipping execution.");
            return false;
        }

        // Copy the auxiliary data structure to the host using async copy
        if (auxType > omni::sensors::AuxType::NONE)
        {
            cudaStream_t auxStream = state.m_cudaStreams[1]; // Use second stream for aux data
            if (modality == omni::sensors::Modality::LIDAR)
            {
                CUDA_CHECK(cudaMemcpyAsync(state.hostLidarAuxPoints, state.hostGMO->auxiliaryData,
                                           sizeof(omni::sensors::LidarAuxiliaryData), cudaMemcpyDeviceToHost, auxStream));
            }
            else if (modality == omni::sensors::Modality::RADAR)
            {
                CUDA_CHECK(cudaMemcpyAsync(state.hostRadarAuxPoints, state.hostGMO->auxiliaryData,
                                           sizeof(omni::sensors::RadarAuxiliaryData), cudaMemcpyDeviceToHost, auxStream));
            }
            CUDA_CHECK(cudaStreamSynchronize(auxStream));
        }

        // Use persistent streams for all copy operations
        auto& cudaStreams = state.m_cudaStreams;

        // Get indices and element counts for the current and next buffer
        size_t startIndex = 0;
        size_t numElementsToCopyToCurrentBuffer = 0;
        size_t numElementsToCopyToNextBuffer = 0;
        {
            if (db.inputs.enablePerFrameOutput())
            {
                numElementsToCopyToCurrentBuffer = numElements;
                numElementsToCopyToNextBuffer = 0;
            }
            else
            {
                startIndex = state.m_totalElements % state.m_maxPoints;
                numElementsToCopyToCurrentBuffer = std::min(numElements, state.m_maxPoints - startIndex);
                numElementsToCopyToNextBuffer = numElements - numElementsToCopyToCurrentBuffer;
            }
        }

        // persistent events to track all copy operations
        auto& copyEvents = state.m_cudaEvents;


        // CUDA Graph based data copy operations
        {
            if (state.m_graphsInitialized &&
                state.updateCurrentGraphParameters(state.hostGMO, numElementsToCopyToCurrentBuffer, startIndex))
            {
                CARB_PROFILE_ZONE(0, "CUDA Graph Launch - Current Buffer");
                // Launch current buffer graph
                auto graphStream = cudaStreams[state.STREAM_AZIMUTH_CURRENT];
                CUDA_CHECK(cudaGraphLaunch(state.m_basicDataCopyGraphExec, graphStream));
                // Record completion events for dependency tracking (current)
                CUDA_CHECK(cudaEventRecord(copyEvents[state.STREAM_AZIMUTH_CURRENT], graphStream));
                CUDA_CHECK(cudaEventRecord(copyEvents[state.STREAM_ELEVATION_CURRENT], graphStream));
                CUDA_CHECK(cudaEventRecord(copyEvents[state.STREAM_DISTANCE_CURRENT], graphStream));
                CUDA_CHECK(cudaEventRecord(copyEvents[state.STREAM_FLAGS_CURRENT], graphStream));

                auto overflowStream = cudaStreams[state.STREAM_AZIMUTH_NEXT];
                // Handle overflow with separate graph if needed
                if (numElementsToCopyToNextBuffer > 0 &&
                    state.updateOverflowGraphParameters(
                        state.hostGMO, numElementsToCopyToCurrentBuffer, numElementsToCopyToNextBuffer))
                {
                    CUDA_CHECK(cudaGraphLaunch(state.m_overflowCopyGraphExec, overflowStream));

                    CUDA_CHECK(cudaEventRecord(copyEvents[state.STREAM_AZIMUTH_NEXT], overflowStream));
                    CUDA_CHECK(cudaEventRecord(copyEvents[state.STREAM_ELEVATION_NEXT], overflowStream));
                    CUDA_CHECK(cudaEventRecord(copyEvents[state.STREAM_DISTANCE_NEXT], overflowStream));
                    CUDA_CHECK(cudaEventRecord(copyEvents[state.STREAM_FLAGS_NEXT], overflowStream));
                }
                else
                {
                    // No overflow: immediately mark next-buffer events as completed to avoid waits
                    CUDA_CHECK(cudaEventRecord(copyEvents[state.STREAM_AZIMUTH_NEXT], overflowStream));
                    CUDA_CHECK(cudaEventRecord(copyEvents[state.STREAM_ELEVATION_NEXT], overflowStream));
                    CUDA_CHECK(cudaEventRecord(copyEvents[state.STREAM_DISTANCE_NEXT], overflowStream));
                    CUDA_CHECK(cudaEventRecord(copyEvents[state.STREAM_FLAGS_NEXT], overflowStream));
                }
            }
            else
            {
                // Fallback to individual operations if graph fails or not initialized
                CUDA_CHECK(cudaMemcpyAsync(state.azimuthBuffers[state.m_currentBuffer].data() + startIndex,
                                           state.hostGMO->elements.x, numElementsToCopyToCurrentBuffer * sizeof(float),
                                           cudaMemcpyDeviceToDevice, cudaStreams[state.STREAM_AZIMUTH_CURRENT]));
                CUDA_CHECK(cudaEventRecord(
                    copyEvents[state.STREAM_AZIMUTH_CURRENT], cudaStreams[state.STREAM_AZIMUTH_CURRENT]));

                CUDA_CHECK(cudaMemcpyAsync(state.elevationBuffers[state.m_currentBuffer].data() + startIndex,
                                           state.hostGMO->elements.y, numElementsToCopyToCurrentBuffer * sizeof(float),
                                           cudaMemcpyDeviceToDevice, cudaStreams[state.STREAM_ELEVATION_CURRENT]));
                CUDA_CHECK(cudaEventRecord(
                    copyEvents[state.STREAM_ELEVATION_CURRENT], cudaStreams[state.STREAM_ELEVATION_CURRENT]));

                CUDA_CHECK(cudaMemcpyAsync(state.distanceBuffers[state.m_currentBuffer].data() + startIndex,
                                           state.hostGMO->elements.z, numElementsToCopyToCurrentBuffer * sizeof(float),
                                           cudaMemcpyDeviceToDevice, cudaStreams[state.STREAM_DISTANCE_CURRENT]));
                CUDA_CHECK(cudaEventRecord(
                    copyEvents[state.STREAM_DISTANCE_CURRENT], cudaStreams[state.STREAM_DISTANCE_CURRENT]));

                CUDA_CHECK(cudaMemcpyAsync(state.flagsBuffers[state.m_currentBuffer].data() + startIndex,
                                           state.hostGMO->elements.flags,
                                           numElementsToCopyToCurrentBuffer * sizeof(uint8_t), cudaMemcpyDeviceToDevice,
                                           cudaStreams[state.STREAM_FLAGS_CURRENT]));
                CUDA_CHECK(
                    cudaEventRecord(copyEvents[state.STREAM_FLAGS_CURRENT], cudaStreams[state.STREAM_FLAGS_CURRENT]));

                // Next buffer copies
                CUDA_CHECK(cudaMemcpyAsync(state.azimuthBuffers[state.m_nextBuffer].data(),
                                           state.hostGMO->elements.x + numElementsToCopyToCurrentBuffer,
                                           numElementsToCopyToNextBuffer * sizeof(float), cudaMemcpyDeviceToDevice,
                                           cudaStreams[state.STREAM_AZIMUTH_NEXT]));
                CUDA_CHECK(cudaEventRecord(copyEvents[state.STREAM_AZIMUTH_NEXT], cudaStreams[state.STREAM_AZIMUTH_NEXT]));

                CUDA_CHECK(cudaMemcpyAsync(state.elevationBuffers[state.m_nextBuffer].data(),
                                           state.hostGMO->elements.y + numElementsToCopyToCurrentBuffer,
                                           numElementsToCopyToNextBuffer * sizeof(float), cudaMemcpyDeviceToDevice,
                                           cudaStreams[state.STREAM_ELEVATION_NEXT]));
                CUDA_CHECK(
                    cudaEventRecord(copyEvents[state.STREAM_ELEVATION_NEXT], cudaStreams[state.STREAM_ELEVATION_NEXT]));

                CUDA_CHECK(cudaMemcpyAsync(state.distanceBuffers[state.m_nextBuffer].data(),
                                           state.hostGMO->elements.z + numElementsToCopyToCurrentBuffer,
                                           numElementsToCopyToNextBuffer * sizeof(float), cudaMemcpyDeviceToDevice,
                                           cudaStreams[state.STREAM_DISTANCE_NEXT]));
                CUDA_CHECK(
                    cudaEventRecord(copyEvents[state.STREAM_DISTANCE_NEXT], cudaStreams[state.STREAM_DISTANCE_NEXT]));

                CUDA_CHECK(cudaMemcpyAsync(state.flagsBuffers[state.m_nextBuffer].data(),
                                           state.hostGMO->elements.flags + numElementsToCopyToCurrentBuffer,
                                           numElementsToCopyToNextBuffer * sizeof(uint8_t), cudaMemcpyDeviceToDevice,
                                           cudaStreams[state.STREAM_FLAGS_NEXT]));
                CUDA_CHECK(cudaEventRecord(copyEvents[state.STREAM_FLAGS_NEXT], cudaStreams[state.STREAM_FLAGS_NEXT]));
            }
        }

        // Copy optional data on dedicated streams
        {
            // Lambda function to handle standard async copy pattern for auxiliary data
            auto copyAuxDataAsync = [&](bool enabled, const char* profileName, size_t streamIndex, auto& bufferArray,
                                        auto* sourcePtr, size_t elementSize, size_t stride = 1)
            {
                if (enabled)
                {
                    CARB_PROFILE_ZONE(0, profileName);
                    auto stream = cudaStreams[streamIndex];
                    CUDA_CHECK(cudaMemcpyAsync(bufferArray[state.m_currentBuffer].data() + startIndex * stride,
                                               sourcePtr, numElementsToCopyToCurrentBuffer * elementSize * stride,
                                               cudaMemcpyDeviceToDevice, stream));
                    CUDA_CHECK(cudaMemcpyAsync(
                        bufferArray[state.m_nextBuffer].data(), sourcePtr + numElementsToCopyToCurrentBuffer * stride,
                        numElementsToCopyToNextBuffer * elementSize * stride, cudaMemcpyDeviceToDevice, stream));
                    CUDA_CHECK(cudaEventRecord(copyEvents[streamIndex], stream));
                }
            };

            // Copy intensity data
            copyAuxDataAsync(state.m_outputIntensity, "Copy Intensity Data", state.STREAM_INTENSITY,
                             state.intensityBuffers, state.hostGMO->elements.scalar, sizeof(float));

            // Copy timestamp data (special case with custom kernel)
            if (state.m_outputTimestamp)
            {
                CARB_PROFILE_ZONE(0, "Copy Timestamp Data");
                auto timestampStream = cudaStreams[state.STREAM_TIMESTAMP];
                copyTimestamps(state.timestampBuffers[state.m_currentBuffer].data() + startIndex,
                               state.hostGMO->timestampNs, state.hostGMO->elements.timeOffsetNs,
                               static_cast<int>(numElementsToCopyToCurrentBuffer), state.m_maxThreadsPerBlock,
                               state.m_multiProcessorCount, cudaDeviceIndex, timestampStream);
                copyTimestamps(state.timestampBuffers[state.m_nextBuffer].data(), state.hostGMO->timestampNs,
                               state.hostGMO->elements.timeOffsetNs + numElementsToCopyToCurrentBuffer,
                               static_cast<int>(numElementsToCopyToNextBuffer), state.m_maxThreadsPerBlock,
                               state.m_multiProcessorCount, cudaDeviceIndex, timestampStream);
                CUDA_CHECK(cudaEventRecord(copyEvents[state.STREAM_TIMESTAMP], timestampStream));
            }

            // Copy lidar auxiliary data
            copyAuxDataAsync(state.m_outputEmitterId, "Copy Emitter ID Data", state.STREAM_EMITTER_ID,
                             state.emitterIdBuffers, state.hostLidarAuxPoints->emitterId, sizeof(uint32_t));
            copyAuxDataAsync(state.m_outputChannelId, "Copy Channel ID Data", state.STREAM_CHANNEL_ID,
                             state.channelIdBuffers, state.hostLidarAuxPoints->channelId, sizeof(uint32_t));
            copyAuxDataAsync(state.m_outputMaterialId, "Copy Material ID Data", state.STREAM_MATERIAL_ID,
                             state.materialIdBuffers, state.hostLidarAuxPoints->matId, sizeof(uint32_t));
            copyAuxDataAsync(state.m_outputTickId, "Copy Tick ID Data", state.STREAM_TICK_ID, state.tickIdBuffers,
                             state.hostLidarAuxPoints->tickId, sizeof(uint32_t));
            copyAuxDataAsync(state.m_outputHitNormal, "Copy Normal Data", state.STREAM_NORMAL, state.normalBuffers,
                             state.hostLidarAuxPoints->hitNormals, sizeof(float3));
            copyAuxDataAsync(state.m_outputVelocity, "Copy Velocity Data", state.STREAM_VELOCITY, state.velocityBuffers,
                             state.hostLidarAuxPoints->velocities, sizeof(float3));
            copyAuxDataAsync(state.m_outputObjectId, "Copy Object ID Data", state.STREAM_OBJECT_ID,
                             state.objectIdBuffers, state.hostLidarAuxPoints->objId, sizeof(uint8_t), 16);
            copyAuxDataAsync(state.m_outputEchoId, "Copy Echo ID Data", state.STREAM_ECHO_ID, state.echoIdBuffers,
                             state.hostLidarAuxPoints->echoId, sizeof(uint8_t));
            copyAuxDataAsync(state.m_outputTickState, "Copy Tick States Data", state.STREAM_TICK_STATE,
                             state.tickStateBuffers, state.hostLidarAuxPoints->tickStates, sizeof(uint8_t));

            // Copy radar auxiliary data
            copyAuxDataAsync(state.m_outputRadialVelocityMS, "Copy Radial Velocity MS Data",
                             state.STREAM_RADIAL_VELOCITY_MS, state.radialVelocityMSBuffers,
                             state.hostRadarAuxPoints->rv_ms, sizeof(float));
        }

        // Event-based sync: Output processing waits for current buffer completion
        // Next buffer streams continue asynchronously for future frames

        if (db.inputs.enablePerFrameOutput() || startIndex + numElementsToCopyToCurrentBuffer == state.m_maxPoints)
        {
            // We've reached the end of the current buffer, and the buffers are guaranteed to be filled.
            // Kick off the point cloud kernel on its own stream

            auto pointCloudStream = cudaStreams[state.STREAM_POINT_CLOUD];
            auto outputEvent = copyEvents[state.STREAM_POINT_CLOUD];

            // Output processing waits for current buffer data completion
            if (state.m_graphsInitialized)
            {
                // When using graphs, all basic data operations complete on the same stream
                CUDA_CHECK(cudaStreamWaitEvent(pointCloudStream, copyEvents[state.STREAM_AZIMUTH_CURRENT], 0));
            }
            else
            {
                // When using individual operations, wait for all current buffer streams
                CUDA_CHECK(cudaStreamWaitEvent(pointCloudStream, copyEvents[state.STREAM_AZIMUTH_CURRENT], 0));
                CUDA_CHECK(cudaStreamWaitEvent(pointCloudStream, copyEvents[state.STREAM_ELEVATION_CURRENT], 0));
                CUDA_CHECK(cudaStreamWaitEvent(pointCloudStream, copyEvents[state.STREAM_DISTANCE_CURRENT], 0));
                CUDA_CHECK(cudaStreamWaitEvent(pointCloudStream, copyEvents[state.STREAM_FLAGS_CURRENT], 0));
            }

            // Select only valid indices on point cloud stream
            size_t numPointsToCheck = db.inputs.enablePerFrameOutput() ? numElements : state.m_maxPoints;

            cudaEvent_t findValidIndicesEvent;
            CUDA_CHECK(cudaEventCreate(&findValidIndicesEvent));
            findValidIndices(state.indicesBuffer.data(), state.indicesValidBuffer.data(), state.numValidPointsDevice,
                             numPointsToCheck, state.flagsBuffers[state.m_currentBuffer].data(), cudaDeviceIndex,
                             pointCloudStream, &state.m_d_temp_storage, &state.m_temp_storage_bytes, &state.m_numPoints);
            // Copy the number of valid points to the host
            CUDA_CHECK(cudaMemcpyAsync(state.numValidPointsHost, state.numValidPointsDevice, sizeof(int),
                                       cudaMemcpyDeviceToHost, pointCloudStream));
            CUDA_CHECK(cudaEventRecord(findValidIndicesEvent, pointCloudStream));
            CUDA_CHECK(cudaStreamSynchronize(pointCloudStream));
            CUDA_CHECK(cudaEventDestroy(findValidIndicesEvent));

            // Fill the valid cartesian points
            fillValidCartesianPoints(
                state.azimuthBuffers[state.m_currentBuffer].data(), state.elevationBuffers[state.m_currentBuffer].data(),
                state.distanceBuffers[state.m_currentBuffer].data(), state.pcBufferValid.data(),
                state.indicesValidBuffer.data(), state.numValidPointsDevice, numPointsToCheck,
                state.m_maxThreadsPerBlock, state.m_multiProcessorCount, cudaDeviceIndex, pointCloudStream);
            CUDA_CHECK(cudaEventRecord(outputEvent, pointCloudStream));

            std::vector<cudaEvent_t> outputCompletionEvents; // Track output completion for final sync

            // Individual data selection kernels
            {
                // Set output pointers and sizes for enabled outputs
                if (state.m_outputAzimuth)
                {
                    db.outputs.azimuthPtr() = reinterpret_cast<uint64_t>(state.azimuthBufferValid.data());
                    db.outputs.azimuthBufferSize() = state.numValidPointsHost[0] * sizeof(float);
                }
                if (state.m_outputElevation)
                {
                    db.outputs.elevationPtr() = reinterpret_cast<uint64_t>(state.elevationBufferValid.data());
                    db.outputs.elevationBufferSize() = state.numValidPointsHost[0] * sizeof(float);
                }
                if (state.m_outputDistance)
                {
                    db.outputs.distancePtr() = reinterpret_cast<uint64_t>(state.distanceBufferValid.data());
                    db.outputs.distanceBufferSize() = state.numValidPointsHost[0] * sizeof(float);
                }
                if (state.m_outputIntensity)
                {
                    db.outputs.intensityPtr() = reinterpret_cast<uint64_t>(state.intensityBufferValid.data());
                    db.outputs.intensityBufferSize() = state.numValidPointsHost[0] * sizeof(float);
                }
                if (state.m_outputTimestamp)
                {
                    db.outputs.timestampPtr() = reinterpret_cast<uint64_t>(state.timestampBufferValid.data());
                    db.outputs.timestampBufferSize() = state.numValidPointsHost[0] * sizeof(uint64_t);
                }
                if (state.m_outputEmitterId)
                {
                    db.outputs.emitterIdPtr() = reinterpret_cast<uint64_t>(state.emitterIdBufferValid.data());
                    db.outputs.emitterIdBufferSize() = state.numValidPointsHost[0] * sizeof(uint32_t);
                }
                if (state.m_outputChannelId)
                {
                    db.outputs.channelIdPtr() = reinterpret_cast<uint64_t>(state.channelIdBufferValid.data());
                    db.outputs.channelIdBufferSize() = state.numValidPointsHost[0] * sizeof(uint32_t);
                }
                if (state.m_outputMaterialId)
                {
                    db.outputs.materialIdPtr() = reinterpret_cast<uint64_t>(state.materialIdBufferValid.data());
                    db.outputs.materialIdBufferSize() = state.numValidPointsHost[0] * sizeof(uint32_t);
                }
                if (state.m_outputTickId)
                {
                    db.outputs.tickIdPtr() = reinterpret_cast<uint64_t>(state.tickIdBufferValid.data());
                    db.outputs.tickIdBufferSize() = state.numValidPointsHost[0] * sizeof(uint32_t);
                }
                if (state.m_outputHitNormal)
                {
                    db.outputs.hitNormalPtr() = reinterpret_cast<uint64_t>(state.normalBufferValid.data());
                    db.outputs.hitNormalBufferSize() = state.numValidPointsHost[0] * sizeof(float3);
                }
                if (state.m_outputVelocity)
                {
                    db.outputs.velocityPtr() = reinterpret_cast<uint64_t>(state.velocityBufferValid.data());
                    db.outputs.velocityBufferSize() = state.numValidPointsHost[0] * sizeof(float3);
                }
                if (state.m_outputObjectId)
                {
                    db.outputs.objectIdPtr() = reinterpret_cast<uint64_t>(state.objectIdBufferValid.data());
                    db.outputs.objectIdBufferSize() = state.numValidPointsHost[0] * sizeof(uint8_t) * 16;
                }
                if (state.m_outputEchoId)
                {
                    db.outputs.echoIdPtr() = reinterpret_cast<uint64_t>(state.echoIdBufferValid.data());
                    db.outputs.echoIdBufferSize() = state.numValidPointsHost[0] * sizeof(uint8_t);
                }
                if (state.m_outputTickState)
                {
                    db.outputs.tickStatePtr() = reinterpret_cast<uint64_t>(state.tickStateBufferValid.data());
                    db.outputs.tickStateBufferSize() = state.numValidPointsHost[0] * sizeof(uint8_t);
                }
                if (state.m_outputRadialVelocityMS)
                {
                    db.outputs.radialVelocityMSPtr() =
                        reinterpret_cast<uint64_t>(state.radialVelocityMSBufferValid.data());
                    db.outputs.radialVelocityMSBufferSize() = state.numValidPointsHost[0] * sizeof(float);
                }

                // Launch fused kernel for common outputs (azimuth, elevation, distance, intensity)
                bool hasCommonOutputs = state.m_outputAzimuth || state.m_outputElevation || state.m_outputDistance ||
                                        state.m_outputIntensity;
                if (hasCommonOutputs)
                {
                    auto commonStream = cudaStreams[state.STREAM_AZIMUTH_CURRENT]; // Use one stream for common outputs
                    CUDA_CHECK(cudaStreamWaitEvent(commonStream, outputEvent, 0));
                    if (state.m_outputIntensity)
                    {
                        CUDA_CHECK(cudaStreamWaitEvent(commonStream, copyEvents[state.STREAM_INTENSITY], 0));
                    }

                    selectRequiredValidPoints(state.azimuthBuffers[state.m_currentBuffer].data(),
                                              state.elevationBuffers[state.m_currentBuffer].data(),
                                              state.distanceBuffers[state.m_currentBuffer].data(),
                                              state.intensityBuffers[state.m_currentBuffer].data(),
                                              state.azimuthBufferValid.data(), state.elevationBufferValid.data(),
                                              state.distanceBufferValid.data(), state.intensityBufferValid.data(),
                                              state.indicesValidBuffer.data(), state.numValidPointsDevice,
                                              numPointsToCheck, state.m_requiredOutputsMask, state.m_maxThreadsPerBlock,
                                              cudaDeviceIndex, commonStream);

                    cudaEvent_t commonCompleteEvent;
                    CUDA_CHECK(cudaEventCreate(&commonCompleteEvent));
                    CUDA_CHECK(cudaEventRecord(commonCompleteEvent, commonStream));
                    outputCompletionEvents.push_back(commonCompleteEvent);
                }

                // Launch fused kernel for auxiliary outputs (timestamp, IDs, normals, velocities)
                bool hasAuxOutputs = state.m_outputTimestamp || state.m_outputEmitterId || state.m_outputChannelId ||
                                     state.m_outputMaterialId || state.m_outputTickId || state.m_outputHitNormal ||
                                     state.m_outputVelocity || state.m_outputObjectId || state.m_outputEchoId ||
                                     state.m_outputTickState || state.m_outputRadialVelocityMS;
                if (hasAuxOutputs)
                {
                    auto auxStream = cudaStreams[state.STREAM_TIMESTAMP]; // Use one stream for aux outputs
                    CUDA_CHECK(cudaStreamWaitEvent(auxStream, outputEvent, 0));

                    // Wait for all relevant aux data copy events
                    if (state.m_outputTimestamp)
                        CUDA_CHECK(cudaStreamWaitEvent(auxStream, copyEvents[state.STREAM_TIMESTAMP], 0));
                    if (state.m_outputEmitterId)
                        CUDA_CHECK(cudaStreamWaitEvent(auxStream, copyEvents[state.STREAM_EMITTER_ID], 0));
                    if (state.m_outputChannelId)
                        CUDA_CHECK(cudaStreamWaitEvent(auxStream, copyEvents[state.STREAM_CHANNEL_ID], 0));
                    if (state.m_outputMaterialId)
                        CUDA_CHECK(cudaStreamWaitEvent(auxStream, copyEvents[state.STREAM_MATERIAL_ID], 0));
                    if (state.m_outputTickId)
                        CUDA_CHECK(cudaStreamWaitEvent(auxStream, copyEvents[state.STREAM_TICK_ID], 0));
                    if (state.m_outputHitNormal)
                        CUDA_CHECK(cudaStreamWaitEvent(auxStream, copyEvents[state.STREAM_NORMAL], 0));
                    if (state.m_outputVelocity)
                        CUDA_CHECK(cudaStreamWaitEvent(auxStream, copyEvents[state.STREAM_VELOCITY], 0));
                    if (state.m_outputObjectId)
                        CUDA_CHECK(cudaStreamWaitEvent(auxStream, copyEvents[state.STREAM_OBJECT_ID], 0));
                    if (state.m_outputEchoId)
                        CUDA_CHECK(cudaStreamWaitEvent(auxStream, copyEvents[state.STREAM_ECHO_ID], 0));
                    if (state.m_outputTickState)
                        CUDA_CHECK(cudaStreamWaitEvent(auxStream, copyEvents[state.STREAM_TICK_STATE], 0));
                    if (state.m_outputRadialVelocityMS)
                        CUDA_CHECK(cudaStreamWaitEvent(auxStream, copyEvents[state.STREAM_RADIAL_VELOCITY_MS], 0));

                    selectOptionalValidPoints(state.timestampBuffers[state.m_currentBuffer].data(),
                                              state.emitterIdBuffers[state.m_currentBuffer].data(),
                                              state.channelIdBuffers[state.m_currentBuffer].data(),
                                              state.materialIdBuffers[state.m_currentBuffer].data(),
                                              state.tickIdBuffers[state.m_currentBuffer].data(),
                                              state.normalBuffers[state.m_currentBuffer].data(),
                                              state.velocityBuffers[state.m_currentBuffer].data(),
                                              state.objectIdBuffers[state.m_currentBuffer].data(),
                                              state.echoIdBuffers[state.m_currentBuffer].data(),
                                              state.tickStateBuffers[state.m_currentBuffer].data(),
                                              state.radialVelocityMSBuffers[state.m_currentBuffer].data(),
                                              state.timestampBufferValid.data(), state.emitterIdBufferValid.data(),
                                              state.channelIdBufferValid.data(), state.materialIdBufferValid.data(),
                                              state.tickIdBufferValid.data(), state.normalBufferValid.data(),
                                              state.velocityBufferValid.data(), state.objectIdBufferValid.data(),
                                              state.echoIdBufferValid.data(), state.tickStateBufferValid.data(),
                                              state.radialVelocityMSBufferValid.data(), state.indicesValidBuffer.data(),
                                              state.numValidPointsDevice, numPointsToCheck, state.m_optionalOutputsMask,
                                              state.m_maxThreadsPerBlock, cudaDeviceIndex, auxStream);

                    cudaEvent_t auxCompleteEvent;
                    CUDA_CHECK(cudaEventCreate(&auxCompleteEvent));
                    CUDA_CHECK(cudaEventRecord(auxCompleteEvent, auxStream));
                    outputCompletionEvents.push_back(auxCompleteEvent);
                }
            }

            // Event-based: Wait for all output processing
            CUDA_CHECK(cudaStreamSynchronize(pointCloudStream));

            // Wait for all output completion events
            for (auto& event : outputCompletionEvents)
            {
                CUDA_CHECK(cudaEventSynchronize(event));
            }

            // Cleanup temporary output events
            for (auto& event : outputCompletionEvents)
            {
                CUDA_CHECK(cudaEventDestroy(event));
            }

            // Note: Next buffer streams intentionally NOT synchronized

            // Set output buffers
            db.outputs.dataPtr() = reinterpret_cast<uint64_t>(state.pcBufferValid.data());
            db.outputs.bufferSize() = state.numValidPointsHost[0] * sizeof(float3);
            db.outputs.width() = state.numValidPointsHost[0];
            db.outputs.height() = 1;

            auto frameEnd = state.hostGMO->frameEnd;
            getTransformFromSensorPose(frameEnd, matrixOutput);

            // Swap the current and next buffers
            std::swap(state.m_currentBuffer, state.m_nextBuffer);
        }

        // Increment the total number of elements written to the buffers
        state.m_totalElements += numElements;

        // Wait for basic data completion events (optimized for graph execution)
        if (state.m_graphsInitialized)
        {
            // When using graphs, all basic operations complete on single stream
            CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_AZIMUTH_CURRENT]));
        }
        else
        {
            // When using individual operations, sync all basic data streams
            CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_AZIMUTH_CURRENT]));
            CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_ELEVATION_CURRENT]));
            CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_DISTANCE_CURRENT]));
            CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_FLAGS_CURRENT]));

            // Wait for next buffer completion events only if data was copied
            if (numElementsToCopyToNextBuffer > 0)
            {
                CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_AZIMUTH_NEXT]));
                CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_ELEVATION_NEXT]));
                CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_DISTANCE_NEXT]));
                CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_FLAGS_NEXT]));
            }
        }

        // Wait for optional data completion events only if they were used
        if (state.m_outputIntensity)
            CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_INTENSITY]));
        if (state.m_outputTimestamp)
            CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_TIMESTAMP]));
        if (state.m_outputEmitterId)
            CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_EMITTER_ID]));
        if (state.m_outputChannelId)
            CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_CHANNEL_ID]));
        if (state.m_outputMaterialId)
            CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_MATERIAL_ID]));
        if (state.m_outputTickId)
            CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_TICK_ID]));
        if (state.m_outputHitNormal)
            CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_NORMAL]));
        if (state.m_outputVelocity)
            CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_VELOCITY]));
        if (state.m_outputObjectId)
            CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_OBJECT_ID]));
        if (state.m_outputEchoId)
            CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_ECHO_ID]));
        if (state.m_outputTickState)
            CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_TICK_STATE]));
        if (state.m_outputRadialVelocityMS)
            CUDA_CHECK(cudaEventSynchronize(copyEvents[state.STREAM_RADIAL_VELOCITY_MS]));

        return true;
    }
};


REGISTER_OGN_NODE()
} // namespace rtx
} // namespace sensors
} // namespace isaacsim
