// Single-glove interactive calibration using the bundled MANUS Integrated SDK.
// SDK connection is process-wide; only the explicitly verified glove is calibrated.
#include "ManusSDK.h"
#include "manus_skeleton_metadata.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void on_signal(int) { interrupted = 1; }

// Vendor logs can include licensing data. Keep only our bounded, selected fields
// on the original console, including while the SDK writes directly to stdio.
class Console {
public:
    Console() {
        std::fflush(nullptr);
        saved_out_ = ::dup(STDOUT_FILENO);
        saved_err_ = ::dup(STDERR_FILENO);
        const int sink = ::open("/dev/null", O_WRONLY);
        stream_ = saved_out_ >= 0 ? ::fdopen(::dup(saved_out_), "w") : nullptr;
        if (!stream_ || saved_err_ < 0 || sink < 0)
            throw std::runtime_error("Cannot establish private SDK logging sink");
        if (::dup2(sink, STDOUT_FILENO) < 0 || ::dup2(sink, STDERR_FILENO) < 0)
            throw std::runtime_error("Cannot suppress SDK console output");
        ::close(sink);
    }
    ~Console() {
        std::fflush(nullptr);
        ::dup2(saved_out_, STDOUT_FILENO);
        ::dup2(saved_err_, STDERR_FILENO);
        ::close(saved_out_); ::close(saved_err_);
        std::fclose(stream_);
    }
    void say(const char* format, ...) const {
        va_list args; va_start(args, format);
        std::vfprintf(stream_, format, args);
        va_end(args); std::fflush(stream_);
    }
    bool confirm(const std::string& token) const {
        if (interrupted) return false;
        say("Type '%s' to continue; anything else or EOF cancels: ", token.c_str());
        // getline can remain blocked after SIGINT (SA_RESTART), including on a
        // partial pipe write. Read incrementally and check cancellation instead.
        const int original_flags = ::fcntl(STDIN_FILENO, F_GETFL);
        if (original_flags < 0 || ::fcntl(STDIN_FILENO, F_SETFL, original_flags | O_NONBLOCK) < 0)
            throw std::runtime_error("Cannot make prompt input interruptible");
        struct RestoreInput {
            int flags;
            ~RestoreInput() { ::fcntl(STDIN_FILENO, F_SETFL, flags); }
        } restore{original_flags};
        std::string line;
        while (!interrupted) {
            pollfd input{STDIN_FILENO, POLLIN, 0};
            const int ready = ::poll(&input, 1, 100);
            if (ready < 0 && errno == EINTR) continue;
            if (ready < 0) throw std::runtime_error("Cannot poll prompt input");
            if (ready == 0) continue;
            char c = 0;
            const auto count = ::read(STDIN_FILENO, &c, 1);
            if (count < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
            if (count <= 0) return false;
            if (c == '\n') return !interrupted && line == token;
            line.push_back(c);
            if (line.size() > 256) return false;
        }
        return false;
    }

private:
    int saved_out_ = -1, saved_err_ = -1;
    FILE* stream_ = nullptr;
};

template<size_t N> std::string text_field(const char (&text)[N]) {
    std::string value;
    for (size_t i = 0; i < N && text[i]; ++i) {
        const auto c = static_cast<unsigned char>(text[i]);
        if (c >= 32 || c == '\n' || c == '\t') value.push_back(text[i]);
    }
    return value;
}
void check(SDKReturnCode code, const char* operation) {
    if (code != SDKReturnCode_Success)
        throw std::runtime_error(std::string(operation) + " failed: SDK code " + std::to_string(code));
}
struct Options {
    uint32_t id = 0;
    Side side = Side_Invalid;
    bool calibrate = false, robots_disabled = false;
    std::filesystem::path output;
};
void usage() {
    std::cout << "Usage: manus_calibrate --glove-id <decimal|0xHEX> --side left|right [--inspect]\n"
        "       manus_calibrate --glove-id <ID> --side left|right --calibrate\n"
        "         --robots-disabled --output new-profile.mcal\n"
        "Output is a filename in the package calibration directory, not a path.\n"
        "Inspect is the default and never starts calibration. Stop other MANUS clients first.\n"
        "Calibrate requires interactive confirmation before each vendor step and before finish.\n";
}
Options options(int argc, char** argv) {
    Options o;
    bool got_id = false, got_side = false, inspect = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto value = [&]() -> std::string {
            if (++i >= argc) throw std::runtime_error("Missing value for " + arg);
            return argv[i];
        };
        if (arg == "--glove-id") {
            const auto input = value(); size_t end = 0;
            const auto id = std::stoull(input, &end, input.rfind("0x", 0) == 0 || input.rfind("0X", 0) == 0 ? 16 : 10);
            if (end != input.size() || input.empty() || input[0] == '-' || id == 0 || id > UINT32_MAX)
                throw std::runtime_error("Invalid glove ID");
            if (got_id) throw std::runtime_error("Duplicate --glove-id");
            got_id = true; o.id = static_cast<uint32_t>(id);
        } else if (arg == "--side") {
            const auto side = value();
            if (got_side || (side != "left" && side != "right")) throw std::runtime_error("Select exactly one side: left or right");
            got_side = true; o.side = side == "left" ? Side_Left : Side_Right;
        } else if (arg == "--inspect") inspect = true;
        else if (arg == "--calibrate") o.calibrate = true;
        else if (arg == "--robots-disabled") o.robots_disabled = true;
        else if (arg == "--output") o.output = value();
        else throw std::runtime_error("Unknown argument: " + arg);
    }
    if (!got_id || !got_side) throw std::runtime_error("Both --glove-id and --side are required; no automatic glove selection");
    if (inspect && o.calibrate) throw std::runtime_error("Choose inspect or calibrate");
    if (o.calibrate) {
        if (!o.robots_disabled) throw std::runtime_error("Confirm robot motors are disabled with --robots-disabled");
        if (o.output.empty() || o.output.has_parent_path() || o.output == "." ||
            o.output == ".." || o.output.string().find('\\') != std::string::npos)
            throw std::runtime_error("--output must be a filename without directories");
        const auto bundled_profile =
            std::filesystem::path(ament_index_cpp::get_package_share_directory("manus_ros2")) /
            "calibration" / "tpxLeftMetaglovePro.mcal";
        o.output = std::filesystem::canonical(bundled_profile).parent_path() / o.output;
        struct stat info{};
        if (::lstat(o.output.c_str(), &info) == 0 || errno != ENOENT)
            throw std::runtime_error("Output already exists or cannot be inspected; refusing overwrite");
        if (!std::filesystem::is_directory(o.output.parent_path())) throw std::runtime_error("Output parent directory must already exist");
    } else if (!o.output.empty()) throw std::runtime_error("--output requires --calibrate");
    return o;
}

struct Identity { bool found = false; Side side = Side_Invalid; DevicePairedState paired{}; DeviceFamilyType family{}; };
std::mutex identity_mutex;
uint32_t selected_id = 0;
Identity identity;
uint32_t raw_node_count = 0;
void on_raw_skeleton(const SkeletonStreamInfo* const stream) {
    if (!stream) return;
    for (uint32_t i = 0; i < stream->skeletonsCount; ++i) {
        RawSkeletonInfo info{};
        if (CoreSdk_GetRawSkeletonInfo(i, &info) == SDKReturnCode_Success && info.gloveId == selected_id) {
            std::lock_guard<std::mutex> lock(identity_mutex);
            raw_node_count = info.nodesCount;
        }
    }
}
uint32_t current_raw_count() { std::lock_guard<std::mutex> lock(identity_mutex); return raw_node_count; }
void on_landscape(const Landscape* const landscape) {
    if (!landscape) return;
    Identity next;
    for (uint32_t i = 0; i < landscape->gloveDevices.gloveCount; ++i) {
        const auto& glove = landscape->gloveDevices.gloves[i];
        if (glove.id == selected_id) {
            next = {true, glove.side, glove.pairedState, glove.familyType};
            break;
        }
    }
    std::lock_guard<std::mutex> lock(identity_mutex);
    identity = next;
}
Identity current_identity() { std::lock_guard<std::mutex> lock(identity_mutex); return identity; }
void discard_log(LogSeverity, const char* const, uint32_t) {}

class SdkSession {
public:
    SdkSession(const Options& o, const Console& console): o_(o), console_(console) {}
    ~SdkSession() {
        cancel();
        if (initialized_) {
            const auto code = CoreSdk_ShutDown();
            if (code != SDKReturnCode_Success) console_.say("SDK shutdown failed: code %d\n", static_cast<int>(code));
        }
    }
    void connect() {
        selected_id = o_.id;
        check(CoreSdk_InitializeIntegrated(), "InitializeIntegrated"); initialized_ = true;
        check(CoreSdk_RegisterCallbackForOnLog(discard_log), "RegisterLogCallback");
        check(CoreSdk_RegisterCallbackForLandscapeStream(on_landscape), "RegisterLandscapeCallback");
        check(CoreSdk_RegisterCallbackForRawSkeletonStream(on_raw_skeleton), "RegisterRawSkeletonCallback");
        CoordinateSystemVUH coords; CoordinateSystemVUH_Init(&coords);
        coords.handedness = Side_Right; coords.up = AxisPolarity_PositiveZ;
        coords.view = AxisView_XFromViewer; coords.unitScale = 1.0f;
        check(CoreSdk_InitializeCoordinateSystemWithVUH(coords, true), "InitializeCoordinateSystem");
        ManusHost host; ManusHost_Init(&host);
        check(CoreSdk_ConnectToHost(host), "ConnectToHost");
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
        while (!interrupted && std::chrono::steady_clock::now() < deadline) {
            if (current_identity().found) { verify(); return; }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        throw std::runtime_error(interrupted ? "Interrupted before glove discovery" : "Requested glove was not found within 20 seconds");
    }
    Identity verify() const {
        if (interrupted) throw std::runtime_error("Interrupted; pending step must return before cancellation");
        bool connected = false; check(CoreSdk_GetIsConnectedToCore(&connected), "GetIsConnectedToCore");
        const auto id = current_identity();
        if (!connected || !id.found) throw std::runtime_error("Requested glove is no longer connected");
        if (id.side != o_.side) throw std::runtime_error("Requested glove ID has the wrong side; no calibration was started for another glove");
        if (id.paired != DevicePairedState_Paired) throw std::runtime_error("Requested glove is not paired");
        return id;
    }
    GloveCalibrationArgs args() const { GloveCalibrationArgs a; GloveCalibrationArgs_Init(&a); a.gloveId = o_.id; return a; }
    GloveCalibrationStepArgs step_args(uint32_t index) const { GloveCalibrationStepArgs a; GloveCalibrationStepArgs_Init(&a); a.gloveId = o_.id; a.stepIndex = index; return a; }
    void begin() {
        verify();
        // A failing RPC can still have begun a session, so request Stop on failure.
        active_ = true;
        bool accepted = false; check(CoreSdk_GloveCalibrationStart(args(), &accepted), "CalibrationStart");
        if (!accepted) throw std::runtime_error("CalibrationStart was rejected by Core");
    }
    void step(uint32_t index) {
        verify();
        bool accepted = false; check(CoreSdk_GloveCalibrationStartStep(step_args(index), &accepted), "CalibrationStartStep");
        if (!accepted) throw std::runtime_error("Calibration step was rejected by Core");
        if (interrupted) throw std::runtime_error("Interrupted during blocking calibration step");
    }
    void finish() {
        verify();
        bool accepted = false; check(CoreSdk_GloveCalibrationFinish(args(), &accepted), "CalibrationFinish");
        if (!accepted) throw std::runtime_error("CalibrationFinish was rejected by Core");
        active_ = false; applied_ = true;
    }
    void cancel() noexcept {
        if (!active_) return;
        bool stopped = false;
        const auto code = CoreSdk_GloveCalibrationStop(args(), &stopped);
        console_.say("Calibration Stop: SDK=%d, accepted=%s.\n", static_cast<int>(code), stopped ? "true" : "false");
        if (code != SDKReturnCode_Success || !stopped)
            console_.say("Stop was not confirmed. Check MANUS Core before retrying; no Finish was requested.\n");
        active_ = false;
    }
    bool applied() const { return applied_; }
private:
    const Options& o_; const Console& console_;
    bool initialized_ = false, active_ = false, applied_ = false;
};

class NewProfile {
public:
    explicit NewProfile(const std::filesystem::path& path): path_(path) {
        fd_ = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW, 0600);
        if (fd_ < 0) throw std::runtime_error("Cannot reserve new output file; no calibration started");
    }
    ~NewProfile() { if (fd_ >= 0) ::close(fd_); if (!saved_) ::unlink(path_.c_str()); }
    void save(const std::vector<unsigned char>& bytes) {
        size_t offset = 0;
        while (offset < bytes.size()) {
            const auto count = ::write(fd_, bytes.data() + offset, bytes.size() - offset);
            if (count < 0 && errno == EINTR) continue;
            if (count <= 0) throw std::runtime_error("Cannot write new calibration file");
            offset += static_cast<size_t>(count);
        }
        if (::fsync(fd_) != 0) throw std::runtime_error("Cannot sync new calibration file");
        saved_ = true;
    }
private:
    std::filesystem::path path_; int fd_ = -1; bool saved_ = false;
};
const char* chain_name(ChainType c) {
    switch(c) {
        case ChainType_Hand: return "Hand"; case ChainType_FingerThumb: return "Thumb";
        case ChainType_FingerIndex: return "Index"; case ChainType_FingerMiddle: return "Middle";
        case ChainType_FingerRing: return "Ring"; case ChainType_FingerPinky: return "Pinky";
        default: return "Other/Invalid";
    }
}
const char* joint_name(FingerJointType j) {
    switch(j) {
        case FingerJointType_Metacarpal: return "Metacarpal"; case FingerJointType_Proximal: return "Proximal";
        case FingerJointType_Intermediate: return "Intermediate"; case FingerJointType_Distal: return "Distal";
        case FingerJointType_Tip: return "Tip"; default: return "Invalid";
    }
}
void hierarchy(uint32_t id, const Console& console) {
    // Integrated Core may announce the glove before it has built its raw skeleton.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    manus_tools::NodeMetadata indexed;
    uint32_t count = 0;
    SDKReturnCode last_code = SDKReturnCode_Error;
    while (!interrupted && std::chrono::steady_clock::now() < deadline) {
        // RawSkeletonInfo is the authoritative count used by the SDK stream.
        count = current_raw_count();
        if (count == 0) last_code = CoreSdk_GetRawSkeletonNodeCount(id, count);
        if (count > 0 && count <= 1024) {
            std::vector<NodeInfo> nodes(count);
            last_code = CoreSdk_GetRawSkeletonNodeInfoArray(id, nodes.data(), count);
            if (last_code == SDKReturnCode_Success) {
                indexed = manus_tools::index_metadata(nodes);
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (indexed.empty()) throw std::runtime_error("Raw hierarchy unavailable after 10 seconds: SDK code " + std::to_string(last_code));
    console.say("Raw hierarchy for glove %u (%u nodes; sorted by nodeId, never by array offset):\n", id, count);
    for (const auto& [node_id, n] : indexed)
        console.say("  id=%u parent=%u side=%d chain=%s(%d) joint=%s(%d)\n", node_id, n.parentId,
            static_cast<int>(n.side), chain_name(n.chainType), static_cast<int>(n.chainType),
            joint_name(n.fingerJointType), static_cast<int>(n.fingerJointType));
}
std::vector<GloveCalibrationStepData> read_steps(const SdkSession& sdk, const Console& console) {
    uint32_t count = 0;
    check(CoreSdk_GloveCalibrationGetNumberOfSteps(sdk.args(), &count), "GetNumberOfSteps");
    if (!count || count > 100) throw std::runtime_error("Invalid calibration step count");
    std::vector<GloveCalibrationStepData> steps(count);
    for (uint32_t i = 0; i < count; ++i) {
        GloveCalibrationStepData_Init(&steps[i]);
        check(CoreSdk_GloveCalibrationGetStepData(sdk.step_args(i), &steps[i]), "GetStepData");
        if (steps[i].index != i || !std::isfinite(steps[i].time)) throw std::runtime_error("Invalid calibration step metadata");
        console.say("Step %u: %s\n  %s\n  duration=%g s%s\n", i,
            text_field(steps[i].title).c_str(), text_field(steps[i].description).c_str(),
            static_cast<double>(steps[i].time), steps[i].time < 0 ? " (negative: continuous, estimated)" : "");
    }
    return steps;
}
int run(const Options& o, const Console& console) {
    console.say("Selected glove %u (0x%08X), side=%s. Integrated Core may enumerate other gloves; all calibration operations target this ID only.\n",
        o.id, o.id, o.side == Side_Left ? "left" : "right");
    if (o.calibrate) console.say("Output calibration: %s\n", o.output.c_str());
    SdkSession sdk(o, console);
    try {
        sdk.connect();
        const auto id = sdk.verify();
        console.say("Verified ID/side/paired state; device family=%d. No existing calibration file was imported.\n", static_cast<int>(id.family));
        const auto steps = read_steps(sdk, console);
        hierarchy(o.id, console);
        if (!o.calibrate) { console.say("Inspection complete. No Start, StartStep, Finish, or SetGloveCalibration call was made.\n"); return 0; }
        if (id.family != DeviceFamilyType_Metaglove && id.family != DeviceFamilyType_MetaglovePro)
            throw std::runtime_error("File export is supported here only for Metaglove / Metaglove Pro");
        console.say("Keep robot motors disabled and other MANUS clients stopped. Finish applies/saves calibration in Core.\n"
            "No steps run automatically. Ctrl-C requests cancellation; an in-flight SDK step is blocking and must return first.\n");
        if (!console.confirm("start " + std::to_string(o.id))) { console.say("Cancelled before calibration Start.\n"); return 2; }
        NewProfile profile(o.output);
        sdk.begin();
        for (const auto& step : steps) {
            console.say("Prepare step %u: %s\n%s\nDuration: %g s.\n", step.index,
                text_field(step.title).c_str(), text_field(step.description).c_str(), static_cast<double>(step.time));
            if (!console.confirm("step " + std::to_string(step.index))) { sdk.cancel(); return 2; }
            sdk.step(step.index);
            console.say("Step %u accepted.\n", step.index);
        }
        console.say("All steps accepted. The next confirmation applies the result and exports the new profile.\n");
        if (!console.confirm("finish " + std::to_string(o.id))) { sdk.cancel(); return 2; }
        sdk.finish();
        // GetSize prepares the SDK's shared export buffer: consume it immediately.
        uint32_t size = 0;
        check(CoreSdk_GetGloveCalibrationSize(o.id, &size), "GetGloveCalibrationSize");
        if (!size || size > 16 * 1024 * 1024) throw std::runtime_error("Invalid calibration export size");
        std::vector<unsigned char> bytes(size);
        check(CoreSdk_GetGloveCalibration(bytes.data(), size), "GetGloveCalibration");
        profile.save(bytes);
        console.say("Calibration applied and exported (%u bytes) to %s\n", size, o.output.c_str());
        console.say("Configure the publisher's matching calibration_file parameter explicitly before restarting it. Validate raw poses with robot motors disabled.\n");
        return 0;
    } catch (const std::exception& e) {
        console.say("Error: %s\n", e.what());
        if (sdk.applied()) console.say("Finish already applied calibration in Core, but export did not complete. Do not assume a new profile is available.\n");
        return 1;
    }
}
}  // namespace
int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") { usage(); return 0; }
    try {
        const auto o = options(argc, argv);
        std::signal(SIGINT, on_signal); std::signal(SIGTERM, on_signal);
        const Console console;
        return run(o, console);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n'; usage(); return 1;
    }
}
