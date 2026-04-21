ssize_t len = recvfrom(mic_sock_, buffer, sizeof(buffer), 0, nullptr, nullptr);

if (len < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // 超时：当前这段时间没收到UDP音频，不算致命错误
        continue;
    } else {
        perror("[State_RLBase] recvfrom");
        continue;
    }
}

if (len == 0) {
    continue;
}
