void State_RLBase::requestDanceTransition()
{
    if (pending_dance_state_name_.empty()) {
        std::cerr << "[State_RLBase] pending_dance_state_name_ is empty" << std::endl;
        return;
    }

    auto it = FSMStringMap.right.find(pending_dance_state_name_);
    if (it == FSMStringMap.right.end()) {
        std::cerr << "[State_RLBase] requestDanceTransition failed, invalid state name = "
                  << pending_dance_state_name_ << std::endl;
        return;
    }

    this->dynamic_next_state_mode = it->second;
    this->request_dynamic_transition = true;

    std::cout << "[State_RLBase] requestDanceTransition -> "
              << pending_dance_state_name_
              << ", id = " << it->second << std::endl;
}



void State_RLBase::enter()
{
    tts_played_ = false;
    voice_motion_active_ = false;
    voice_cmd_ = {0.0f, 0.0f, 0.0f};
    pending_voice_cmd_ = static_cast<int>(VoiceCommand::NONE);
    pending_dance_type_ = DanceType::NONE;
    pending_dance_state_name_.clear();

    request_dynamic_transition = false;
    dynamic_next_state_mode = 0;

    current_instance_ = this;
}
