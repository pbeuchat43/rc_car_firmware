#include "rc_car/device.hpp"
#include "build_config.hpp"
#include "vehicle_profile.hpp"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "pico/rand.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "tusb.h"

namespace {

constexpr uint32_t frame_us = 20000;
constexpr uint32_t loop_fault_us = 2000;
static_assert(board::drive_output < 30 && board::steer_output < 30 && board::pan_output < 30);
static_assert(board::drive_input < 30 && board::steer_input < 30);
static_assert(board::drive_input != board::steer_input);
static_assert(board::drive_input != board::drive_output && board::drive_input != board::steer_output &&
    board::drive_input != board::pan_output && board::steer_input != board::drive_output &&
    board::steer_input != board::steer_output && board::steer_input != board::pan_output);
static_assert(board::drive_output % 2 == 0 && board::steer_output == board::drive_output + 1);
static_assert(((board::drive_output >> 1) & 7) != ((board::pan_output >> 1) & 7));
static_assert(rc_car::valid(board::vehicle));
static_assert(PICO_UNIQUE_BOARD_ID_SIZE_BYTES == 8);

class Capture {
public:
    void edge(uint32_t events, uint64_t at) {
        if (events == GPIO_IRQ_EDGE_RISE) {
            if (awaiting_fall_) fault_ = true;
            period_ = have_rise_ ? span(at - rise_) : 0;
            rise_ = at;
            have_rise_ = true;
            awaiting_fall_ = true;
        } else if (events == GPIO_IRQ_EDGE_FALL && awaiting_fall_) {
            if (sample_) fault_ = true;
            sample_ = rc_car::Pulse{at / 1000, span(at - rise_), period_};
            awaiting_fall_ = false;
        } else {
            fault_ = true;
            have_rise_ = awaiting_fall_ = false;
        }
    }

    void take(std::optional<rc_car::Pulse> &pulse, bool &fault, bool loop_gap) {
        fault = fault_ || loop_gap;
        if (!fault) pulse = sample_;
        else have_rise_ = awaiting_fall_ = false;
        sample_.reset();
        fault_ = false;
    }

private:
    static uint16_t span(uint64_t us) { return us <= UINT16_MAX ? static_cast<uint16_t>(us) : 0; }
    uint64_t rise_{};
    uint16_t period_{};
    bool awaiting_fall_{}, have_rise_{}, fault_{};
    std::optional<rc_car::Pulse> sample_;
};

Capture drive_capture, steer_capture;

void edge(unsigned pin, uint32_t events) {
    if (pin == board::drive_input) drive_capture.edge(events, time_us_64());
    if (pin == board::steer_input) steer_capture.edge(events, time_us_64());
}

void setup_capture() {
    for (auto pin : {board::drive_input, board::steer_input}) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_down(pin);
    }
    constexpr auto edges = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    gpio_set_irq_enabled_with_callback(board::drive_input, edges, true, edge);
    gpio_set_irq_enabled(board::steer_input, edges, true);
}

rc_car::Inputs capture(bool gap, uint64_t &sampled_at) {
    rc_car::Inputs input;
    const uint32_t interrupts = save_and_disable_interrupts();
    drive_capture.take(input.drive, input.drive_fault, gap);
    steer_capture.take(input.steer, input.steer_fault, gap);
    sampled_at = time_us_64();
    restore_interrupts(interrupts);
    return input;
}

void init_slice(unsigned slice) {
    auto config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, static_cast<float>(clock_get_hz(clk_sys)) / 1000000.0f);
    pwm_config_set_wrap(&config, frame_us - 1);
    pwm_init(slice, &config, false);
}

class Outputs {
public:
    Outputs() {
        init_slice(drive_slice_);
        pwm_set_both_levels(drive_slice_, board::vehicle.drive.center, board::vehicle.steer.center);
        gpio_set_function(board::drive_output, GPIO_FUNC_PWM);
        gpio_set_function(board::steer_output, GPIO_FUNC_PWM);
        pwm_set_enabled(drive_slice_, true);
        gpio_init(board::pan_output);
        gpio_put(board::pan_output, false);
        gpio_set_dir(board::pan_output, GPIO_OUT);
        init_slice(pan_slice_);
    }

    void stage(const rc_car::wire::Status &status) {
        if (status.drive_pwm != drive_ || status.steer_pwm != steer_) {
            pwm_set_both_levels(drive_slice_, status.drive_pwm, status.steer_pwm);
            drive_ = status.drive_pwm;
            steer_ = status.steer_pwm;
        }
        if (status.pan_pwm && status.pan_pwm != pan_) {
            pwm_set_chan_level(pan_slice_, pwm_gpio_to_channel(board::pan_output), status.pan_pwm);
            if (!pan_) {
                gpio_set_function(board::pan_output, GPIO_FUNC_PWM);
                pwm_set_enabled(pan_slice_, true);
            }
            pan_ = status.pan_pwm;
        }
    }

private:
    const unsigned drive_slice_{pwm_gpio_to_slice_num(board::drive_output)};
    const unsigned pan_slice_{pwm_gpio_to_slice_num(board::pan_output)};
    uint16_t drive_{board::vehicle.drive.center}, steer_{board::vehicle.steer.center}, pan_{};
};

bool send(const rc_car::wire::Line &record) {
    if (!record.size || tud_cdc_write_available() < record.size) return false;
    if (tud_cdc_write(record.data.data(), record.size) != record.size) return false;
    tud_cdc_write_flush();
    return true;
}
}

int main() {
    // Drive and steer stage neutral before USB and receiver setup.
    Outputs outputs;
    const auto started = time_us_64() / 1000;
    setup_capture();
    char device_id[17], boot_id[17];
    pico_get_unique_board_id_string(device_id, sizeof(device_id));
    auto session_seed = get_rand_64();
    std::snprintf(boot_id, sizeof(boot_id), "%016llX", static_cast<unsigned long long>(session_seed));
    rc_car::Device device(board::vehicle,
        {{}, {}, device_id, boot_id, "RCCAR", RC_BOARD, RC_BUILD_ID, RC_PROFILE_ID}, started);
    const tusb_rhport_init_t usb{TUSB_ROLE_DEVICE, TUSB_SPEED_FULL};
    tusb_init(0, &usb);
    rc_car::wire::Stream input;
    rc_car::wire::Line response;
    uint64_t last_status = started, last_pass = time_us_64();
    bool connected = false;
    watchdog_enable(50, false);
    while (true) {
        uint64_t sampled_at;
        const auto samples = capture(time_us_64() - last_pass >= loop_fault_us, sampled_at);
        const auto now = sampled_at / 1000;
        const bool transport_gap = sampled_at - last_pass >= rc_car::wire::frame_timeout_ms * 1000;
        last_pass = sampled_at;
        device.tick(now, samples);
        tud_task_ext(0, false);
        const bool usb_connected = tud_cdc_connected();
        if (!usb_connected || transport_gap) {
            if (connected || transport_gap) {
                device.disconnect();
                input.reset();
                response = {};
                tud_cdc_read_flush();
            }
        } else {
            if (response.size && send(response)) response = {};
            for (unsigned i = 0; i < 128 && !response.size; ++i) {
                const auto ch = tud_cdc_read_char();
                if (ch < 0) break;
                const auto record = input.push(static_cast<char>(ch), now);
                if (record.empty()) continue;
                response = device.receive(record, ++session_seed);
            }
            if (response.size && send(response)) response = {};
        }
        connected = usb_connected;
        const auto snapshot = device.status();
        outputs.stage(snapshot);
        if (connected && device.connected() && !response.size && now - last_status >= rc_car::wire::status_period_ms &&
            send(rc_car::wire::encode(snapshot))) {
            device.status_sent(snapshot.tick);
            last_status = now;
        }
        watchdog_update();
        sleep_us(250);
    }
}
