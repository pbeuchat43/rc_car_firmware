#pragma once

#include "rc_car/wire.hpp"
#include <algorithm>
#include <optional>

namespace rc_car {

struct InputProfile {
    uint16_t min, center, max, deadband;
    bool invert;
};
struct OutputProfile {
    uint16_t min, center, max;
    bool invert;
};
struct Profile {
    InputProfile drive_in, steer_in;
    OutputProfile drive, steer, pan;
    uint16_t width_min, width_max, period_min, period_max, maintenance_min, maintenance_max;
    uint16_t pan_delay_ms;
};
constexpr uint64_t pwm_ms = 20;
constexpr uint64_t rc_timeout_ms = 100;
constexpr uint64_t rc_baseline_ms = 150;
constexpr uint64_t drive_timeout_ms = 100;
constexpr uint64_t first_drive_ms = 500;
constexpr uint64_t maintenance_lease_ms = 500;
constexpr uint64_t maintenance_dwell_ms = 15000;
constexpr uint64_t maintenance_max_ms = 120000;
// Include a frame for asynchronous staging before counting two complete frames.
constexpr uint64_t two_frames_ms = 3 * pwm_ms;

constexpr bool valid(Profile p) {
    const auto input = [&](InputProfile in) {
        return p.width_min >= 400 && p.width_max <= 2600 && p.width_min <= in.min &&
            in.max <= p.width_max && in.min + in.deadband < in.center && in.center + in.deadband < in.max;
    };
    const auto output = [](OutputProfile out) {
        return out.min >= 400 && out.min < out.center && out.center < out.max && out.max <= 2600;
    };
    return input(p.drive_in) && input(p.steer_in) && output(p.drive) && output(p.steer) && output(p.pan) &&
        p.period_min > p.width_max && p.period_min <= p.period_max && p.period_max < rc_timeout_ms * 1000 &&
        p.maintenance_min >= 400 && p.maintenance_min <= p.drive.min && p.maintenance_max >= p.drive.max && p.maintenance_max <= 2600;
}

struct Pulse {
    uint64_t at_ms;
    uint16_t width_us, period_us;
};

struct Inputs {
    std::optional<Pulse> drive, steer;
    bool drive_fault{}, steer_fault{};
};

class Channel {
public:
    bool sample(uint64_t now, const std::optional<Pulse> &pulse, bool fault, const Profile &p) {
        if (fault || (streak_ && now - last_ms_ >= rc_timeout_ms)) streak_ = 0;
        if (fault || !pulse) return false;
        width_ = pulse->width_us;
        if (pulse->at_ms > now || now - pulse->at_ms >= rc_timeout_ms ||
            width_ < p.width_min || width_ > p.width_max ||
            pulse->period_us < p.period_min || pulse->period_us > p.period_max) {
            streak_ = 0;
            return false;
        }
        last_ms_ = pulse->at_ms;
        if (streak_ < 3) ++streak_;
        return true;
    }
    bool good() const { return streak_ == 3; }
    uint16_t width() const { return width_; }

private:
    unsigned streak_{};
    uint64_t last_ms_{};
    uint16_t width_{};
};

inline bool neutral(uint16_t pulse, InputProfile profile) {
    return pulse >= profile.center - profile.deadband && pulse <= profile.center + profile.deadband;
}

inline int normalized(uint16_t pulse, InputProfile p) {
    int input = std::clamp(pulse, p.min, p.max);
    int q = 0;
    if (input < p.center - p.deadband)
        q = -((p.center - p.deadband - input) * wire::full) / (p.center - p.deadband - p.min);
    if (input > p.center + p.deadband)
        q = ((input - p.center - p.deadband) * wire::full) / (p.max - p.center - p.deadband);
    return p.invert ? -q : q;
}

inline uint16_t pwm(int q, OutputProfile p) {
    q = std::clamp(q, -wire::full, wire::full);
    if (p.invert) q = -q;
    const int span = q < 0 ? p.center - p.min : p.max - p.center;
    const int magnitude = q < 0 ? -q : q;
    const int delta = (magnitude * span + wire::full / 2) / wire::full;
    return static_cast<uint16_t>(q < 0 ? p.center - delta : p.center + delta);
}

class Control {
public:
    explicit Control(Profile profile, uint64_t start = 0) : profile_(profile), start_(start),
        mode_since_(start), valid_(valid(profile)) {
        if (!valid_) mode_ = wire::Mode::hold;
    }

    void tick(uint64_t now, Inputs samples = {}) {
        now_ = now;
        const bool fresh_drive = rc_drive_.sample(now, samples.drive, samples.drive_fault, profile_);
        rc_steer_.sample(now, samples.steer, samples.steer_fault, profile_);
        const bool both = rc_drive_.good() && rc_steer_.good();
        const auto quality = both ? wire::Rc::good :
            (quality_ == wire::Rc::unknown && now - start_ < rc_baseline_ms ? wire::Rc::unknown : wire::Rc::bad);
        bool edge = false;
        if (quality != quality_) {
            edge = quality_ != wire::Rc::unknown;
            quality_ = quality;
            if (edge) {
                ++generation_;
                if (mode_ == wire::Mode::automatic)
                    cancel(quality == wire::Rc::bad ? wire::Reason::rc_lost : wire::Reason::rc_returned, false);
            }
        }
        if (mode_ == wire::Mode::manual && !both) change(wire::Mode::neutral, !edge);
        if (mode_ == wire::Mode::automatic) {
            const auto elapsed = target_ ? now - target_->at : now - mode_since_;
            if (elapsed >= (target_ ? drive_timeout_ms : first_drive_ms)) {
                cancel(wire::Reason::drive_timeout);
                // An earlier sample from this tick may predate the deadline.
                return update_pan();
            }
        }
        if (mode_ == wire::Mode::maintenance &&
            (now - maintenance_start_ >= maintenance_max_ms || now - maintenance_at_ >= maintenance_lease_ms ||
             (maintenance_output_ != profile_.drive.center && now - nonneutral_at_ >= maintenance_dwell_ms) ||
             !maintenance_rc_ok())) {
            leave_maintenance(!maintenance_rc_ok() ? wire::Reason::maintenance_abort : wire::Reason::maintenance_timeout);
        }
        if (mode_ == wire::Mode::neutral && both && fresh_drive && neutral(rc_drive_.width(), profile_.drive_in))
            change(wire::Mode::manual, !edge);
        update_pan();
    }

    void link_lost() {
        if (mode_ == wire::Mode::automatic) cancel(wire::Reason::link_lost);
        if (mode_ == wire::Mode::maintenance) leave_maintenance(wire::Reason::link_lost);
        accepted_drive_ = 0;
        pan_seq_ = 0;
        pan_pending_.reset();
    }

    wire::Code request(const wire::ModeRequest &m) {
        using namespace wire;
        if (!valid_) return Code::conflict;
        if (m.mode != Mode::hold && m.generation != generation_) return Code::wrong_generation;
        if (m.mode != Mode::automatic && (m.drive != Choice::none || m.steer != Choice::none))
            return Code::bad_range;
        if (m.mode == Mode::hold) {
            if (mode_ == Mode::maintenance) leave_maintenance(Reason::maintenance_abort);
            else if (mode_ != Mode::hold) {
                if (mode_ == Mode::automatic) record_cancel(Reason::requested);
                change(Mode::hold);
                hold_reason_ = Reason::requested;
            }
            return Code::ok;
        }
        if (mode_ == Mode::maintenance) return Code::conflict;
        if (m.mode == Mode::manual) {
            if (mode_ == Mode::automatic) cancel(Reason::requested);
            else if (mode_ == Mode::hold) change(Mode::neutral);
            return Code::ok;
        }
        if (m.mode != Mode::automatic || m.drive == Choice::none || m.steer == Choice::none ||
            (m.drive == Choice::rc && m.steer == Choice::rc)) return Code::bad_range;
        if (quality_ == Rc::unknown || mode_ == Mode::hold) return Code::conflict;
        if (mode_ == Mode::automatic) {
            if (m.drive == drive_choice_ && m.steer == steer_choice_) return Code::conflict;
            record_cancel(Reason::new_choices);
        }
        change(Mode::automatic);
        drive_choice_ = m.drive;
        steer_choice_ = m.steer;
        return Code::ok;
    }

    wire::Code drive(const wire::Drive &d) {
        using namespace wire;
        if (d.generation != generation_) return Code::wrong_generation;
        if (mode_ != Mode::automatic) return Code::conflict;
        if (!demand(d.drive) || !demand(d.steer)) return Code::bad_range;
        accepted_drive_ = d.key.seq;
        target_ = Target{now_, d.drive, d.steer};
        return Code::ok;
    }

    wire::Code pan(const wire::Pan &p) {
        using namespace wire;
        if (!valid_ || mode_ == Mode::maintenance) return Code::conflict;
        if (!demand(p.demand)) return Code::bad_range;
        pan_pending_ = PanTarget{p.key.seq, p.demand};
        update_pan();
        return Code::ok;
    }

    wire::Code maintenance(const wire::Maintenance &b) {
        using namespace wire;
        if (b.generation != generation_) return Code::wrong_generation;
        if (b.operation != Operation::pulse && b.pulse != 0) return Code::bad_range;
        if (!valid_) return Code::conflict;
        if (b.operation == Operation::enter) {
            if (mode_ != Mode::hold || now_ - mode_since_ < two_frames_ms || !maintenance_rc_ok())
                return Code::conflict;
            change(Mode::maintenance);
            maintenance_start_ = maintenance_at_ = neutral_at_ = now_;
            maintenance_output_ = profile_.drive.center;
            pan_pending_.reset();
            return Code::ok;
        }
        if (mode_ != Mode::maintenance) return Code::conflict;
        if (b.operation == Operation::exit || b.operation == Operation::abort) {
            leave_maintenance(b.operation == Operation::exit ? Reason::requested : Reason::maintenance_abort);
            return Code::ok;
        }
        if (b.pulse < profile_.maintenance_min || b.pulse > profile_.maintenance_max) return Code::bad_range;
        if (maintenance_output_ == profile_.drive.center && b.pulse != profile_.drive.center) {
            if (now_ - neutral_at_ < two_frames_ms) return Code::conflict;
            nonneutral_at_ = now_;
        }
        if (maintenance_output_ != profile_.drive.center && b.pulse == profile_.drive.center) neutral_at_ = now_;
        maintenance_output_ = b.pulse;
        maintenance_at_ = now_;
        return Code::ok;
    }

    wire::Status status(wire::Text session = {}) const {
        using namespace wire;
        Status s{};
        s.session = session;
        s.tick = now_;
        s.generation = generation_;
        s.mode = mode_;
        s.cancel_generation = cancel_generation_;
        s.cancel_reason = cancel_reason_;
        s.rc = quality_;
        s.valid_axes = uint8_t(rc_drive_.good()) | uint8_t(rc_steer_.good() ? 2 : 0);
        s.rc_drive = rc_drive_.width();
        s.rc_steer = rc_steer_.width();
        s.accepted_drive = accepted_drive_;
        s.drive_pwm = profile_.drive.center;
        s.steer_pwm = profile_.steer.center;
        s.pan_seq = pan_seq_;
        s.pan_pwm = pan_output_;
        if (mode_ == Mode::manual) {
            s.drive_source = s.steer_source = Choice::rc;
            s.drive_pwm = pwm(normalized(s.rc_drive, profile_.drive_in), profile_.drive);
            s.steer_pwm = pwm(normalized(s.rc_steer, profile_.steer_in), profile_.steer);
        } else if (mode_ == Mode::automatic) {
            s.drive_choice = drive_choice_;
            s.steer_choice = steer_choice_;
            if (!target_) s.reason = Reason::wait_drive;
            else {
                const auto d = select(drive_choice_, target_->drive, normalized(s.rc_drive, profile_.drive_in));
                const auto t = select(steer_choice_, target_->steer, normalized(s.rc_steer, profile_.steer_in));
                s.drive_source = d.source;
                s.steer_source = t.source;
                s.saturated = uint8_t(d.saturated) | uint8_t(t.saturated ? 2 : 0);
                s.selected_drive = accepted_drive_;
                s.drive_pwm = pwm(d.demand, profile_.drive);
                s.steer_pwm = pwm(t.demand, profile_.steer);
            }
        } else if (mode_ == Mode::maintenance) {
            s.drive_pwm = maintenance_output_;
        } else if (mode_ == Mode::hold) {
            s.reason = valid_ ? hold_reason_ : Reason::fault;
        } else {
            s.reason = quality_ == Rc::unknown ? Reason::rc_unknown :
                (quality_ == Rc::bad ? Reason::rc_bad : Reason::wait_neutral);
        }
        return s;
    }

private:
    struct Target { uint64_t at; int drive, steer; };
    struct PanTarget { uint32_t seq; int value; };
    struct Selected { wire::Choice source; int demand; bool saturated; };

    Selected select(wire::Choice choice, int host, int rc) const {
        using wire::Choice;
        const bool good = quality_ == wire::Rc::good;
        if (choice == Choice::rc) return good ? Selected{Choice::rc, rc, false} : Selected{Choice::none, 0, false};
        if (choice == Choice::host || !good) return {Choice::host, host, false};
        const int sum = host + rc;
        return {Choice::sum, std::clamp(sum, -wire::full, wire::full), !wire::demand(sum)};
    }
    void change(wire::Mode mode, bool advance = true) {
        mode_ = mode;
        mode_since_ = now_;
        target_.reset();
        if (advance) ++generation_;
    }
    void record_cancel(wire::Reason why, bool advance = true) {
        cancel_generation_ = generation_ + uint64_t(advance);
        cancel_reason_ = why;
    }
    void cancel(wire::Reason why, bool advance = true) {
        record_cancel(why, advance);
        change(wire::Mode::neutral, advance);
    }
    bool maintenance_rc_ok() const {
        return !rc_drive_.good() || neutral(rc_drive_.width(), profile_.drive_in);
    }
    void leave_maintenance(wire::Reason why) {
        change(wire::Mode::hold);
        hold_reason_ = why;
    }
    void update_pan() {
        if (!valid_ || mode_ == wire::Mode::maintenance || now_ - start_ < profile_.pan_delay_ms) return;
        if (!pan_output_) {
            pan_output_ = profile_.pan.center;
            pan_center_at_ = now_;
        }
        if (pan_pending_ && now_ - pan_center_at_ >= 2 * pwm_ms) {
            pan_output_ = pwm(pan_pending_->value, profile_.pan);
            pan_seq_ = pan_pending_->seq;
            pan_pending_.reset();
        }
    }

    Profile profile_;
    uint64_t start_, mode_since_;
    bool valid_;
    uint64_t now_{}, generation_{1}, cancel_generation_{}, pan_center_at_{};
    wire::Mode mode_{wire::Mode::neutral};
    wire::Reason cancel_reason_{}, hold_reason_{};
    wire::Choice drive_choice_{}, steer_choice_{};
    wire::Rc quality_{};
    Channel rc_drive_, rc_steer_;
    std::optional<Target> target_;
    std::optional<PanTarget> pan_pending_;
    uint32_t accepted_drive_{}, pan_seq_{};
    uint16_t pan_output_{}, maintenance_output_{};
    uint64_t maintenance_start_{}, maintenance_at_{}, neutral_at_{}, nonneutral_at_{};
};

} // namespace rc_car
