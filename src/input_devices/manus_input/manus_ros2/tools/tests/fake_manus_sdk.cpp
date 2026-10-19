// Offline test double only: never linked to the vendor shared library.
#include "ManusSDK.h"
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
namespace {
constexpr uint32_t target = 229813950;
LandscapeStreamCallback_t landscape_callback = nullptr;
RawSkeletonStreamCallback_t raw_callback = nullptr;
void record(const char* operation, uint32_t id = target) {
    std::ofstream(std::getenv("MANUS_TEST_TRACE"), std::ios::app) << operation << " " << id << '\n';
}
bool mode(const char* value) {
    const char* configured = std::getenv("MANUS_TEST_MODE");
    return configured && std::string(configured) == value;
}
}
extern "C" {
void ManusHost_Init(ManusHost* p) { *p = {}; }
void CoordinateSystemVUH_Init(CoordinateSystemVUH* p) { *p = {}; }
void GloveCalibrationArgs_Init(GloveCalibrationArgs* p) { *p = {}; }
void GloveCalibrationStepArgs_Init(GloveCalibrationStepArgs* p) { *p = {}; }
void GloveCalibrationStepData_Init(GloveCalibrationStepData* p) { *p = {}; }
SDKReturnCode CoreSdk_InitializeIntegrated() { record("initialize"); return SDKReturnCode_Success; }
SDKReturnCode CoreSdk_ShutDown() { record("shutdown"); return SDKReturnCode_Success; }
SDKReturnCode CoreSdk_RegisterCallbackForOnLog(LoggingCallback_t) { return SDKReturnCode_Success; }
SDKReturnCode CoreSdk_RegisterCallbackForLandscapeStream(LandscapeStreamCallback_t c) { landscape_callback = c; return SDKReturnCode_Success; }
SDKReturnCode CoreSdk_RegisterCallbackForRawSkeletonStream(RawSkeletonStreamCallback_t c) { raw_callback = c; return SDKReturnCode_Success; }
SDKReturnCode CoreSdk_InitializeCoordinateSystemWithVUH(CoordinateSystemVUH c, bool world) {
    if (c.handedness != Side_Right || c.up != AxisPolarity_PositiveZ || c.view != AxisView_XFromViewer || c.unitScale != 1.0f || !world)
        return SDKReturnCode_Error;
    return SDKReturnCode_Success;
}
SDKReturnCode CoreSdk_ConnectToHost(ManusHost) {
    record("connect");
    auto landscape = std::make_unique<Landscape>();
    landscape->gloveDevices.gloveCount = 2;
    auto& glove = landscape->gloveDevices.gloves[0];
    glove.id = target; glove.side = mode("wrong_side") ? Side_Left : Side_Right;
    glove.pairedState = DevicePairedState_Paired; glove.familyType = DeviceFamilyType_MetaglovePro;
    landscape->gloveDevices.gloves[1] = glove;
    landscape->gloveDevices.gloves[1].id = 123; // A second paired glove must never be calibrated.
    landscape_callback(landscape.get());
    SkeletonStreamInfo stream{}; stream.skeletonsCount = 1; raw_callback(&stream);
    return SDKReturnCode_Success;
}
SDKReturnCode CoreSdk_GetIsConnectedToCore(bool* connected) { *connected = true; return SDKReturnCode_Success; }
SDKReturnCode CoreSdk_GetRawSkeletonInfo(uint32_t, RawSkeletonInfo* info) { *info = {}; info->gloveId = target; info->nodesCount = 3; return SDKReturnCode_Success; }
SDKReturnCode CoreSdk_GetRawSkeletonNodeCount(uint32_t id, uint32_t& count) { record("count", id); count = 0; return SDKReturnCode_Error; }
SDKReturnCode CoreSdk_GetRawSkeletonNodeInfoArray(uint32_t id, NodeInfo* nodes, uint32_t count) {
    record("hierarchy", id);
    if (count != 3) return SDKReturnCode_Error;
    nodes[0] = {25, 24, ChainType_FingerThumb, Side_Right, FingerJointType_Tip};
    nodes[1] = {1, 1, ChainType_Hand, Side_Right, FingerJointType_Invalid};
    nodes[2] = {18, 17, ChainType_FingerPinky, Side_Right, FingerJointType_Proximal};
    if (mode("duplicate")) nodes[2].nodeId = 25;
    return SDKReturnCode_Success;
}
SDKReturnCode CoreSdk_GloveCalibrationGetNumberOfSteps(GloveCalibrationArgs a, uint32_t* count) { record("steps", a.gloveId); *count = 2; return SDKReturnCode_Success; }
SDKReturnCode CoreSdk_GloveCalibrationGetStepData(GloveCalibrationStepArgs a, GloveCalibrationStepData* data) {
    record("step_data", a.gloveId); *data = {}; data->index = a.stepIndex; data->time = a.stepIndex ? -3.0f : 2.0f;
    std::strcpy(data->title, a.stepIndex ? "Bend" : "Open"); std::strcpy(data->description, "Vendor-provided pose instructions.");
    return SDKReturnCode_Success;
}
SDKReturnCode CoreSdk_GloveCalibrationStart(GloveCalibrationArgs a, bool* accepted) { record("start", a.gloveId); *accepted = true; return SDKReturnCode_Success; }
SDKReturnCode CoreSdk_GloveCalibrationStartStep(GloveCalibrationStepArgs a, bool* accepted) {
    record("run_step", a.gloveId); *accepted = !mode("step_rejected");
    return mode("step_sdk_failure") ? SDKReturnCode_Error : SDKReturnCode_Success;
}
SDKReturnCode CoreSdk_GloveCalibrationStop(GloveCalibrationArgs a, bool* accepted) { record("stop", a.gloveId); *accepted = true; return SDKReturnCode_Success; }
SDKReturnCode CoreSdk_GloveCalibrationFinish(GloveCalibrationArgs a, bool* accepted) { record("finish", a.gloveId); *accepted = true; return SDKReturnCode_Success; }
SDKReturnCode CoreSdk_GetGloveCalibrationSize(uint32_t id, uint32_t* size) { record("export_size", id); *size = 4; return SDKReturnCode_Success; }
SDKReturnCode CoreSdk_GetGloveCalibration(unsigned char* data, uint32_t size) { record("export_bytes"); std::memset(data, 42, size); return mode("export_failure") ? SDKReturnCode_Error : SDKReturnCode_Success; }
}
