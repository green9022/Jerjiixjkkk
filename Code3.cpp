size_t total_size = pcm.size();
size_t offset = 0;

std::string stream_id =
    std::to_string(unitree::common::GetCurrentTimeMillisecond());

while (offset < total_size) {
    size_t remaining = total_size - offset;
    size_t current_chunk_size =
        std::min(static_cast<size_t>(CHUNK_SIZE), remaining);

    std::vector<uint8_t> chunk(
        pcm.begin() + offset,
        pcm.begin() + offset + current_chunk_size
    );

    client.PlayStream("example", stream_id, chunk);

    double chunk_seconds =
        static_cast<double>(current_chunk_size) / (sample_rate * num_channels * 2);

    // 给播放器至少这一块的真实播放时间
    unitree::common::Sleep(static_cast<uint32_t>(std::ceil(chunk_seconds)));

    offset += current_chunk_size;
}

// 最后一块再多等一点余量
unitree::common::Sleep(1);
client.PlayStop(stream_id);
