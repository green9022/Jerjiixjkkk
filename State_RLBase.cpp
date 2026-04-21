#include "FSM/State_RLBase.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"

#include <unordered_map>
#include <iostream>
#include <cmath>
#include <algorithm>

#include <ifaddrs.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <errno.h>

// ===== G1 麦克风 UDP 参数 =====
#define GROUP_IP "239.168.123.161"
#define PORT 5555

#define SAMPLE_RATE 16000
#define CHANNELS 1

// 单次 recv buffer，大约160ms
#define WAV_LEN_ONCE (16000 * 2 * 160 / 1000)

// ===== Python 音频/文本服务端参数 =====
#define PYTHON_AUDIO_SERVER_IP "127.0.0.1"
#define PYTHON_AUDIO_SERVER_PORT 9000

namespace isaaclab
{
REGISTER_OBSERVATION(keyboard_velocity_commands)
{
    std::string key = FSMState::keyboard->key();

    static std::unordered_map<std::string, std::vector<float>> key_commands = {
        {"w", { 1.0f,  0.0f,  0.0f}},
        {"s", {-1.0f,  0.0f,  0.0f}},
        {"a", { 0.0f,  1.0f,  0.0f}},
        {"d", { 0.0f, -1.0f,  0.0f}},
        {"q", { 0.0f,  0.0f,  1.0f}},
        {"e", { 0.0f,  0.0f, -1.0f}}
    };

    auto it = key_commands.find(key);
    if (it != key_commands.end()) {
        return it->second;
    }

    return State_RLBase::GetVoiceCommand();
}

REGISTER_OBSERVATION(gait_phase_my)
{
    float period = params["period"].as<float>();
    float delta_phase = env->step_dt * (1.0f / period);

    env->global_phase += delta_phase;
    env->global_phase = std::fmod(env->global_phase, 1.0f);

    auto cmd = isaaclab::mdp::velocity_commands(env, params);
    float cmd_norm = std::sqrt(
        cmd[0] * cmd[0] +
        cmd[1] * cmd[1] +
        cmd[2] * cmd[2]
    );

    std::vector<float> obs(2);
    obs[0] = std::sin(env->global_phase * 2 * M_PI);
    obs[1] = std::cos(env->global_phase * 2 * M_PI);

    if (cmd_norm < 0.1f)
    {
        obs[0] = 0.0f;
        obs[1] = 0.0f;
    }

    return obs;
}
} // namespace isaaclab


State_RLBase* State_RLBase::current_instance_ = nullptr;


static bool send_all(int sock, const void* data, size_t len)
{
    const char* ptr = reinterpret_cast<const char*>(data);
    size_t total = 0;

    while (total < len) {
        ssize_t sent = send(sock, ptr + total, len - total, 0);
        if (sent <= 0) {
            return false;
        }
        total += static_cast<size_t>(sent);
    }
    return true;
}

static bool recv_all_cpp(int sock, void* data, size_t len)
{
    char* ptr = reinterpret_cast<char*>(data);
    size_t total = 0;

    while (total < len) {
        ssize_t recvd = recv(sock, ptr + total, len - total, 0);
        if (recvd <= 0) {
            return false;
        }
        total += static_cast<size_t>(recvd);
    }
    return true;
}


std::vector<float> State_RLBase::GetVoiceCommand()
{
    if (current_instance_ != nullptr) {
        return current_instance_->voice_cmd_;
    }
    return {0.0f, 0.0f, 0.0f};
}


void State_RLBase::initAudioClientWithoutChannelInit()
{
    if (audio_inited_) {
        return;
    }

    try {
        audio_client_.Init();
        audio_client_.SetTimeout(10.0f);

        uint8_t volume = 0;
        int32_t ret = audio_client_.GetVolume(volume);
        std::cout << "[State_RLBase] GetVolume ret = " << ret
                  << ", current volume = " << static_cast<int>(volume) << std::endl;

        ret = audio_client_.SetVolume(100);
        std::cout << "[State_RLBase] SetVolume ret = " << ret << std::endl;

        audio_inited_ = true;
    }
    catch (const std::exception& e) {
        std::cerr << "[State_RLBase] initAudioClientWithoutChannelInit exception: "
                  << e.what() << std::endl;
    }
    catch (...) {
        std::cerr << "[State_RLBase] initAudioClientWithoutChannelInit unknown exception."
                  << std::endl;
    }
}


bool State_RLBase::connectPythonAudioSocket()
{
    std::lock_guard<std::mutex> lock(python_audio_state_mutex_);

    if (python_audio_sock_ >= 0) {
        return true;
    }

    python_audio_sock_ = socket(AF_INET, SOCK_STREAM, 0);
    if (python_audio_sock_ < 0) {
        perror("[State_RLBase] python socket");
        return false;
    }

    sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PYTHON_AUDIO_SERVER_PORT);

    if (inet_pton(AF_INET, PYTHON_AUDIO_SERVER_IP, &serv_addr.sin_addr) <= 0) {
        std::cerr << "[State_RLBase] invalid python server ip" << std::endl;
        close(python_audio_sock_);
        python_audio_sock_ = -1;
        return false;
    }

    if (connect(python_audio_sock_, (sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("[State_RLBase] connect python audio socket");
        close(python_audio_sock_);
        python_audio_sock_ = -1;
        return false;
    }

    // 避免接收线程无限阻塞
    timeval tv{};
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(python_audio_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    std::cout << "[State_RLBase] connected to Python audio server: "
              << PYTHON_AUDIO_SERVER_IP << ":" << PYTHON_AUDIO_SERVER_PORT << std::endl;

    return true;
}


void State_RLBase::closePythonAudioSocket()
{
    std::lock_guard<std::mutex> lock(python_audio_state_mutex_);

    if (python_audio_sock_ >= 0) {
        close(python_audio_sock_);
        python_audio_sock_ = -1;
    }
}


bool State_RLBase::sendAudioPacketToPython(
    const int16_t* samples,
    size_t sample_count,
    int sample_rate,
    int channels)
{
    std::lock_guard<std::mutex> send_lock(python_audio_send_mutex_);

    int sock = python_audio_sock_;
    if (sock < 0) {
        return false;
    }

    struct AudioPacketHeader {
        int32_t sample_rate;
        int32_t num_samples;
        int32_t channels;
        int32_t reserved;
    } header;

    header.sample_rate = sample_rate;
    header.num_samples = static_cast<int32_t>(sample_count);
    header.channels = channels;
    header.reserved = 0;

    if (!send_all(sock, &header, sizeof(header))) {
        std::cerr << "[State_RLBase] failed to send audio header to python" << std::endl;
        closePythonAudioSocket();
        return false;
    }

    const size_t pcm_bytes = sample_count * sizeof(int16_t);
    if (pcm_bytes > 0) {
        if (!send_all(sock, samples, pcm_bytes)) {
            std::cerr << "[State_RLBase] failed to send audio pcm to python" << std::endl;
            closePythonAudioSocket();
            return false;
        }
    }

    return true;
}


bool State_RLBase::receiveUtf8ResultFromPython(std::string& text)
{
    text.clear();

    std::lock_guard<std::mutex> recv_lock(python_audio_recv_mutex_);

    int sock = python_audio_sock_;
    if (sock < 0) {
        return false;
    }

    int32_t text_len = 0;
    if (!recv_all_cpp(sock, &text_len, sizeof(text_len))) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return false;
        }
        std::cerr << "[State_RLBase] failed to receive text length from python" << std::endl;
        closePythonAudioSocket();
        return false;
    }

    if (text_len < 0 || text_len > 1024 * 1024) {
        std::cerr << "[State_RLBase] invalid text length from python: " << text_len << std::endl;
        closePythonAudioSocket();
        return false;
    }

    if (text_len == 0) {
        return true;
    }

    text.resize(static_cast<size_t>(text_len));
    if (!recv_all_cpp(sock, text.data(), static_cast<size_t>(text_len))) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return false;
        }
        std::cerr << "[State_RLBase] failed to receive utf-8 text body from python" << std::endl;
        closePythonAudioSocket();
        return false;
    }

    return true;
}


std::string State_RLBase::get_local_ip_for_multicast()
{
    struct ifaddrs *ifaddr = nullptr, *ifa = nullptr;
    char host[NI_MAXHOST];
    std::string result = "";

    if (getifaddrs(&ifaddr) == -1) {
        perror("[State_RLBase] getifaddrs");
        return "";
    }

    for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET) {
            continue;
        }

        if (getnameinfo(
                ifa->ifa_addr,
                sizeof(struct sockaddr_in),
                host,
                NI_MAXHOST,
                nullptr,
                0,
                NI_NUMERICHOST) != 0) {
            continue;
        }

        std::string ip(host);
        if (ip.find("192.168.123.") == 0) {
            result = ip;
            break;
        }
    }

    freeifaddrs(ifaddr);
    return result;
}


void State_RLBase::startMicReceiver()
{
    std::cout << "====================ENTER startMicReceiver====================" << std::endl;

    if (mic_running_) {
        return;
    }

    mic_running_ = true;
    mic_thread_ = std::thread(&State_RLBase::micReceiverLoop, this);

    std::cout << "[State_RLBase] mic receiver thread started." << std::endl;
}


void State_RLBase::stopMicReceiver()
{
    mic_running_ = false;

    if (mic_sock_ >= 0) {
        close(mic_sock_);
        mic_sock_ = -1;
    }

    if (mic_thread_.joinable()) {
        mic_thread_.join();
    }

    std::cout << "[State_RLBase] mic receiver thread stopped." << std::endl;
}


void State_RLBase::startPythonResultReceiver()
{
    if (python_recv_running_) {
        return;
    }

    python_recv_running_ = true;
    python_recv_thread_ = std::thread(&State_RLBase::pythonResultLoop, this);

    std::cout << "[State_RLBase] python result receiver thread started." << std::endl;
}


void State_RLBase::stopPythonResultReceiver()
{
    python_recv_running_ = false;

    if (python_recv_thread_.joinable()) {
        python_recv_thread_.join();
    }

    std::cout << "[State_RLBase] python result receiver thread stopped." << std::endl;
}


void State_RLBase::micReceiverLoop()
{
    std::cout << "====================ENTER micReceiverLoop====================" << std::endl;

    mic_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (mic_sock_ < 0) {
        perror("[State_RLBase] udp socket");
        return;
    }

    int reuse = 1;
    setsockopt(mic_sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(PORT);
    local_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(mic_sock_, (sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        perror("[State_RLBase] bind");
        close(mic_sock_);
        mic_sock_ = -1;
        return;
    }

    ip_mreq mreq{};
    inet_pton(AF_INET, GROUP_IP, &mreq.imr_multiaddr);

    std::string local_ip = get_local_ip_for_multicast();
    std::cout << "[State_RLBase] local ip for multicast: " << local_ip << std::endl;

    if (local_ip.empty()) {
        std::cerr << "[State_RLBase] failed to find local 192.168.123.x ip" << std::endl;
        close(mic_sock_);
        mic_sock_ = -1;
        return;
    }

    mreq.imr_interface.s_addr = inet_addr(local_ip.c_str());
    if (setsockopt(mic_sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
        perror("[State_RLBase] setsockopt IP_ADD_MEMBERSHIP");
        close(mic_sock_);
        mic_sock_ = -1;
        return;
    }

    std::cout << "[State_RLBase] micReceiverLoop started, forwarding raw audio to Python..." << std::endl;

    timeval tv{};
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(mic_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    while (mic_running_) {
        if (!connectPythonAudioSocket()) {
            usleep(200 * 1000);
            continue;
        }

        char buffer[WAV_LEN_ONCE];

        std::cout << "====================ENTER mic_running_1122====================" << std::endl;
        std::cout << "[State_RLBase] waiting recvfrom..." << std::endl;

        ssize_t len = recvfrom(mic_sock_, buffer, sizeof(buffer), 0, nullptr, nullptr);

        std::cout << "[State_RLBase] recvfrom returned len=" << len << std::endl;

        if (len < 0) {
            perror("[State_RLBase] recvfrom");
            continue;
        }

        if (len == 0) {
            continue;
        }

        size_t sample_count = static_cast<size_t>(len) / sizeof(int16_t);
        const int16_t* samples = reinterpret_cast<const int16_t*>(buffer);

        bool ok = sendAudioPacketToPython(samples, sample_count, SAMPLE_RATE, CHANNELS);
        if (!ok) {
            std::cerr << "[State_RLBase] failed to forward audio packet to Python." << std::endl;
            usleep(100 * 1000);
        }
    }

    std::cout << "[State_RLBase] micReceiverLoop exit." << std::endl;
}


void State_RLBase::pythonResultLoop()
{
    std::cout << "[State_RLBase] pythonResultLoop started." << std::endl;

    while (python_recv_running_) {
        if (!connectPythonAudioSocket()) {
            usleep(200 * 1000);
            continue;
        }

        std::string text;
        bool ok = receiveUtf8ResultFromPython(text);

        if (!ok) {
            usleep(20 * 1000);
            continue;
        }

        if (!text.empty()) {
            std::cout << "[State_RLBase] Python UTF-8 text: " << text << std::endl;
            handleASRMessage(text);
        }
    }

    std::cout << "[State_RLBase] pythonResultLoop exit." << std::endl;
}


void State_RLBase::handleASRMessage(const std::string& text)
{
    {
        std::lock_guard<std::mutex> lock(asr_text_mutex_);
        last_asr_text_ = text;
    }
    has_new_asr_ = true;

    if (text.find("开始跳舞") != std::string::npos ||
        text.find("跳舞") != std::string::npos ||
        text.find("舞蹈") != std::string::npos) {
        pending_voice_cmd_ = static_cast<int>(VoiceCommand::DANCE);
        return;
    }

    if (text.find("站立") != std::string::npos ||
        text.find("停止") != std::string::npos ||
        text.find("停下") != std::string::npos) {
        pending_voice_cmd_ = static_cast<int>(VoiceCommand::STAND);
        return;
    }

    if (text.find("前进") != std::string::npos ||
        text.find("往前") != std::string::npos) {
        pending_voice_cmd_ = static_cast<int>(VoiceCommand::FORWARD);
        return;
    }

    if (text.find("后退") != std::string::npos ||
        text.find("往后") != std::string::npos) {
        pending_voice_cmd_ = static_cast<int>(VoiceCommand::BACKWARD);
        return;
    }

    if (text.find("左移") != std::string::npos) {
        pending_voice_cmd_ = static_cast<int>(VoiceCommand::LEFT);
        return;
    }

    if (text.find("右移") != std::string::npos) {
        pending_voice_cmd_ = static_cast<int>(VoiceCommand::RIGHT);
        return;
    }

    if (text.find("左转") != std::string::npos) {
        pending_voice_cmd_ = static_cast<int>(VoiceCommand::TURN_LEFT);
        return;
    }

    if (text.find("右转") != std::string::npos) {
        pending_voice_cmd_ = static_cast<int>(VoiceCommand::TURN_RIGHT);
        return;
    }

    pending_voice_cmd_ = static_cast<int>(VoiceCommand::NONE);
}


void State_RLBase::playStandTTSOnce()
{
    if (!audio_inited_) {
        return;
    }
    if (tts_played_) {
        return;
    }

    int32_t ret = audio_client_.TtsMaker("你好。我现在处于站立状态。", 0);
    std::cout << "[State_RLBase] TtsMaker ret = " << ret << std::endl;

    if (ret == 0) {
        tts_played_ = true;
    }
}


void State_RLBase::applyPendingVoiceCommand()
{
    VoiceCommand cmd = static_cast<VoiceCommand>(pending_voice_cmd_.load());
    pending_voice_cmd_ = static_cast<int>(VoiceCommand::NONE);

    switch (cmd)
    {
    case VoiceCommand::STAND:
        voice_motion_active_ = false;
        voice_cmd_ = {0.0f, 0.0f, 0.0f};
        std::cout << "[State_RLBase] VoiceCommand -> STAND" << std::endl;
        if (audio_inited_) {
            audio_client_.TtsMaker("收到，保持站立。", 0);
        }
        break;

    case VoiceCommand::FORWARD:
        voice_motion_active_ = true;
        voice_cmd_ = {0.5f, 0.0f, 0.0f};
        std::cout << "[State_RLBase] VoiceCommand -> FORWARD" << std::endl;
        if (audio_inited_) {
            audio_client_.TtsMaker("收到，开始前进。", 0);
        }
        break;

    case VoiceCommand::BACKWARD:
        voice_motion_active_ = true;
        voice_cmd_ = {-0.4f, 0.0f, 0.0f};
        std::cout << "[State_RLBase] VoiceCommand -> BACKWARD" << std::endl;
        if (audio_inited_) {
            audio_client_.TtsMaker("收到，开始后退。", 0);
        }
        break;

    case VoiceCommand::LEFT:
        voice_motion_active_ = true;
        voice_cmd_ = {0.0f, 0.4f, 0.0f};
        std::cout << "[State_RLBase] VoiceCommand -> LEFT" << std::endl;
        if (audio_inited_) {
            audio_client_.TtsMaker("收到，开始左移。", 0);
        }
        break;

    case VoiceCommand::RIGHT:
        voice_motion_active_ = true;
        voice_cmd_ = {0.0f, -0.4f, 0.0f};
        std::cout << "[State_RLBase] VoiceCommand -> RIGHT" << std::endl;
        if (audio_inited_) {
            audio_client_.TtsMaker("收到，开始右移。", 0);
        }
        break;

    case VoiceCommand::TURN_LEFT:
        voice_motion_active_ = true;
        voice_cmd_ = {0.0f, 0.0f, 0.5f};
        std::cout << "[State_RLBase] VoiceCommand -> TURN_LEFT" << std::endl;
        if (audio_inited_) {
            audio_client_.TtsMaker("收到，开始左转。", 0);
        }
        break;

    case VoiceCommand::TURN_RIGHT:
        voice_motion_active_ = true;
        voice_cmd_ = {0.0f, 0.0f, -0.5f};
        std::cout << "[State_RLBase] VoiceCommand -> TURN_RIGHT" << std::endl;
        if (audio_inited_) {
            audio_client_.TtsMaker("收到，开始右转。", 0);
        }
        break;

    case VoiceCommand::DANCE:
        voice_motion_active_ = false;
        voice_cmd_ = {0.0f, 0.0f, 0.0f};
        std::cout << "[State_RLBase] VoiceCommand -> DANCE" << std::endl;
        if (audio_inited_) {
            audio_client_.TtsMaker("收到，开始跳舞。", 0);
        }
        requestDanceTransition();
        break;

    case VoiceCommand::NONE:
    default:
        break;
    }

    if (!voice_motion_active_) {
        voice_cmd_ = {0.0f, 0.0f, 0.0f};
    }
}


void State_RLBase::requestDanceTransition()
{
    this->nextStateName = FSMStringMap.right.at("Mimic");
}


State_RLBase::State_RLBase(int state_mode, std::string state_string)
    : FSMState(state_mode, state_string)
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate)
    );
    env->alg = std::make_unique<isaaclab::OrtRunner>(
        policy_dir / "exported" / "policy.onnx"
    );

    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool { return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
            FSMStringMap.right.at("Passive")
        )
    );

    current_instance_ = this;

    initAudioClientWithoutChannelInit();

    tts_played_ = false;
    voice_motion_active_ = false;
    voice_cmd_ = {0.0f, 0.0f, 0.0f};

    connectPythonAudioSocket();
    startMicReceiver();
    startPythonResultReceiver();
}


State_RLBase::~State_RLBase()
{
    stopMicReceiver();
    stopPythonResultReceiver();
    closePythonAudioSocket();
}


void State_RLBase::enter()
{
    tts_played_ = false;
    voice_motion_active_ = false;
    voice_cmd_ = {0.0f, 0.0f, 0.0f};
    pending_voice_cmd_ = static_cast<int>(VoiceCommand::NONE);

    current_instance_ = this;
}


void State_RLBase::run()
{
    applyPendingVoiceCommand();

    auto action = env->action_manager->processed_actions();

    for (int i = 0; i < env->robot->data.joint_ids_map.size(); i++) {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }

    playStandTTSOnce();

    if (has_new_asr_) {
        has_new_asr_ = false;

        std::lock_guard<std::mutex> lock(asr_text_mutex_);
        std::cout << "[State_RLBase] latest ASR text = "
                  << last_asr_text_ << std::endl;
    }
}
