#pragma once

#include "rc_car/control.hpp"
#include <array>

namespace rc_car {

class Device {
public:
    Device(Profile profile, wire::Identity identity, uint64_t start = 0)
        : control_(profile, start), identity_(identity) {}

    void tick(uint64_t now, Inputs input = {}) {
        now_ = now;
        control_.tick(now, input);
    }

    void disconnect() {
        if (!session_.empty()) control_.link_lost();
        session_ = {};
        challenge_ = {};
        seq_ = 0;
        ticks_.fill(std::nullopt);
        tick_index_ = 0;
    }

    wire::Line receive(wire::Text line, uint64_t next_session) {
        using namespace wire;
        Fields fields(line);
        if (Hello hello; fields.type() == "HELLO" && read(fields, hello)) {
            if (hello.challenge != challenge_) {
                disconnect();
                std::copy(hello.challenge.begin(), hello.challenge.end(), challenge_data_.begin());
                challenge_ = {challenge_data_.data(), 16};
                std::snprintf(session_data_.data(), session_data_.size(), "%016llX",
                    static_cast<unsigned long long>(next_session));
                session_ = {session_data_.data(), 16};
            }
            auto identity = identity_;
            identity.session = session_;
            identity.challenge = challenge_;
            return encode(identity);
        }
        if (session_.empty()) return {};
        if (ModeRequest mode; fields.type() == "MODE" && read(fields, mode)) {
            return reply(mode.key, [&](const auto &m) { return control_.request(m); }, mode);
        }
        if (Bench bench; fields.type() == "BENCH" && read(fields, bench)) {
            return reply(bench.key, [&](const auto &b) { return control_.bench(b); }, bench);
        }
        if (Drive drive; fields.type() == "DRIVE" && read(fields, drive)) {
            apply(drive.key, [&] { return control_.drive(drive); });
        }
        if (Pan pan; fields.type() == "PAN" && read(fields, pan)) {
            apply(pan.key, [&] { return control_.pan(pan); });
        }
        return {};
    }

    wire::Status status() const { return control_.status(session_); }
    bool connected() const { return !session_.empty(); }
    void status_sent(uint64_t tick) {
        ticks_[tick_index_] = tick;
        tick_index_ = (tick_index_ + 1) % ticks_.size();
    }

private:
    template<class Fn> wire::Code apply(const wire::Mutation &key, Fn fn) {
        using wire::Code;
        if (key.session != session_) return Code::stale;
        if (key.seq <= seq_) return Code::bad_sequence;
        if (key.tick > now_ || now_ - key.tick >= wire::status_timeout_ms ||
            std::find(ticks_.begin(), ticks_.end(), key.tick) == ticks_.end()) return Code::stale;
        const auto result = fn();
        if (result == Code::ok) seq_ = key.seq;
        return result;
    }
    template<class Fn, class Msg>
    wire::Line reply(const wire::Mutation &key, Fn fn, const Msg &msg) {
        if (key.session != session_) return {};
        const auto result = apply(key, [&] { return fn(msg); });
        return wire::encode(wire::Result{session_, key.seq, result, control_.status().generation});
    }

    Control control_;
    wire::Identity identity_;
    uint64_t now_{};
    uint32_t seq_{};
    std::array<char, 17> session_data_{}, challenge_data_{};
    wire::Text session_, challenge_;
    std::array<std::optional<uint64_t>, 4> ticks_{};
    std::size_t tick_index_{};
};

} // namespace rc_car
