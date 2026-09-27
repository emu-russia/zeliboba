// zeliboba - SDL3 audio backend.
//
// The machine has no audio path yet (the kernel is still the target), so the
// stream is fed silence - or, on request, a quiet 440 Hz tone that proves the
// whole chain (device -> callback -> stream) works. Opening a real SDL3 device
// at 48 kHz / stereo / S16 is deliberate: when a sample-producing block appears
// it only has to push frames into UiAudio instead of touching SDL.
#include <SDL3/SDL.h>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "common/util.h"
#include "ui/ui.h"

namespace zlb {

namespace {
constexpr int kSampleRate = 48000;
constexpr int kChannels = 2;
constexpr int kBytesPerFrame = kChannels * 2;  // S16
constexpr double kTwoPi = 6.283185307179586;
constexpr double kTestToneHz = 440.0;
constexpr double kTestToneAmplitude = 4000.0;  // about -18 dBFS
}  // namespace

UiAudio::~UiAudio() { close(); }

void SDLCALL UiAudio::trampoline(void* userdata, SDL_AudioStream* stream, int additional, int total) {
    (void)total;
    if (!userdata) return;
    static_cast<UiAudio*>(userdata)->render(stream, additional);
}

bool UiAudio::open() {
    close();

    SDL_AudioSpec spec{};
    spec.format = SDL_AUDIO_S16;
    spec.channels = kChannels;
    spec.freq = kSampleRate;

    stream_ = SDL_OpenAudioDeviceStream(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, &spec, &UiAudio::trampoline, this);
    if (!stream_) {
        status_ = format("audio: SDL_OpenAudioDeviceStream(%d Hz, %d ch, SDL_AUDIO_S16) FAILED: %s "
                         "- continuing without audio",
                         kSampleRate, kChannels, SDL_GetError());
        std::printf("zeliboba_ui: %s\n", status_.c_str());
        return false;
    }

    device_ = SDL_GetAudioStreamDevice(stream_);
    if (!SDL_ResumeAudioStreamDevice(stream_)) {
        status_ = format("audio: SDL_ResumeAudioStreamDevice FAILED: %s - continuing without audio",
                         SDL_GetError());
        std::printf("zeliboba_ui: %s\n", status_.c_str());
        SDL_DestroyAudioStream(stream_);
        stream_ = nullptr;
        device_ = 0;
        return false;
    }

    status_ = format("audio: SDL_OpenAudioDeviceStream ok - %d Hz, %d ch, SDL_AUDIO_S16 (device %u)",
                     kSampleRate, kChannels, static_cast<unsigned>(device_));
    std::printf("zeliboba_ui: %s\n", status_.c_str());
    return true;
}

void UiAudio::close() {
    if (!stream_) {
        device_ = 0;
        return;
    }
    // Destroying a stream created by SDL_OpenAudioDeviceStream also closes the
    // device it opened.
    SDL_DestroyAudioStream(stream_);
    stream_ = nullptr;
    device_ = 0;
}

void UiAudio::render(SDL_AudioStream* stream, int additional) {
    if (additional <= 0) return;

    callbacks_.fetch_add(1);
    frames_.fetch_add(static_cast<u64>(additional / kBytesPerFrame));

    scratch_.assign(static_cast<size_t>(additional), 0);
    if (test_tone_.load() && !muted_.load()) {
        const int frames = additional / kBytesPerFrame;
        s16* out = reinterpret_cast<s16*>(scratch_.data());
        const double step = kTwoPi * kTestToneHz / static_cast<double>(kSampleRate);
        for (int i = 0; i < frames; ++i) {
            const s16 sample = static_cast<s16>(std::sin(phase_) * kTestToneAmplitude);
            out[i * 2 + 0] = sample;
            out[i * 2 + 1] = sample;
            phase_ += step;
            if (phase_ >= kTwoPi) phase_ -= kTwoPi;
        }
    }
    SDL_PutAudioStreamData(stream, scratch_.data(), additional);
}

std::string UiAudio::summary() const {
    if (!stream_) return status_;
    const bool tone = test_tone_.load();
    const bool muted = muted_.load();
    return format("%s  callbacks=%llu frames=%llu%s", status_.c_str(),
                  static_cast<unsigned long long>(callbacks_.load()),
                  static_cast<unsigned long long>(frames_.load()),
                  tone ? (muted ? " (muted, test tone armed)" : " (440 Hz test tone)") : "");
}

}  // namespace zlb
