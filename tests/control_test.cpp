#include "rc_car/device.hpp"
#include "../profiles/research_car.example.hpp"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

using namespace rc_car;
using namespace rc_car::wire;

#define CHECK(x) do { if (!(x)) { std::cerr << __LINE__ << ": " #x "\n"; std::abort(); } } while (0)

namespace {
constexpr auto profile = board::vehicle;
constexpr Text device_id = "1010101010101010", boot_id = "2020202020202020", challenge = "0123456789ABCDEF";
constexpr Text first_session = "0000000000000001";
constexpr Text second_session = "0000000000000002";

void sample(Control &c, uint64_t at, uint16_t drive = 1500, uint16_t steer = 1500) {
    c.tick(at, {Pulse{at, drive, 20000}, Pulse{at, steer, 20000}});
}

void good(Control &c, uint64_t first, uint16_t drive = 1500, uint16_t steer = 1500) {
    for (uint64_t t : {first, first + 20, first + 40}) sample(c, t, drive, steer);
}

void arm(Control &c, Choice drive = Choice::host, Choice steer = Choice::host) {
    CHECK(c.request({{}, c.status().generation, Mode::automatic, drive, steer}) == Code::ok);
}

void drive(Control &c, int d, int s, uint32_t seq = 1) {
    CHECK(c.drive({{{}, seq, 0}, c.status().generation, d, s}) == Code::ok);
}

void quality_and_handover() {
    Control c(profile);
    CHECK(c.status().rc == Rc::unknown);
    CHECK(c.request({{}, 1, Mode::automatic, Choice::host, Choice::host}) == Code::conflict);
    c.tick(149);
    CHECK(c.status().rc == Rc::unknown);
    c.tick(150);
    CHECK(c.status().rc == Rc::bad);
    arm(c);
    drive(c, 5000, -5000);
    const auto auto_gen = c.status().generation;
    good(c, 160, 2000, 1000);
    CHECK(c.status().mode == Mode::neutral && c.status().rc == Rc::good);
    CHECK(c.status().drive_pwm == 1500 && c.status().steer_pwm == 1500);
    CHECK(c.status().cancel_reason == Reason::rc_returned && c.status().cancel_generation > auto_gen);
    CHECK(c.request({{}, auto_gen, Mode::automatic, Choice::host, Choice::host}) == Code::wrong_generation);
    sample(c, 220, 1520, 1000);
    CHECK(c.status().mode == Mode::manual && c.status().steer_pwm == 1000);
    sample(c, 240, 2000, 1000);
    CHECK(c.status().drive_pwm == profile.drive.max);
    arm(c);
    drive(c, -10000, 0);
    sample(c, 260, 1000, 1500);
    CHECK(c.status().mode == Mode::automatic && c.status().drive_pwm == profile.drive.min);
    c.tick(265, {{}, Pulse{265, 1500, 1}});
    CHECK(c.status().mode == Mode::neutral && c.status().cancel_reason == Reason::rc_lost);
    CHECK(c.status().valid_axes == 1 && c.status().drive_pwm == 1500);
    CHECK(c.status().generation > auto_gen);
    arm(c);
    drive(c, 10000, 0);
    CHECK(c.status().drive_pwm == profile.drive.max);
}

void choices() {
    const std::array selections{Choice::host, Choice::rc, Choice::sum};
    for (bool healthy : {true, false}) {
        for (auto d : selections) for (auto s : selections) {
            Control c(profile);
            if (healthy) good(c, 20, 2000, 1000);
            else c.tick(150);
            if (d == Choice::rc && s == Choice::rc) {
                CHECK(c.request({{}, c.status().generation, Mode::automatic, d, s}) == Code::bad_range);
                continue;
            }
            arm(c, d, s);
            CHECK(c.status().reason == Reason::wait_drive);
            drive(c, 5000, -5000);
            const auto snap = c.status();
            CHECK(snap.drive_pwm == (d == Choice::rc ? (healthy ? 1750 : 1500) :
                (d == Choice::sum && healthy ? 1750 : 1625)));
            CHECK(snap.steer_pwm == (s == Choice::rc ? (healthy ? 1000 : 1500) :
                (s == Choice::sum && healthy ? 1000 : 1250)));
            CHECK(snap.saturated == (unsigned(d == Choice::sum && healthy) |
                (unsigned(s == Choice::sum && healthy) << 1)));
        }
    }
    Control c(profile);
    good(c, 20);
    arm(c);
    drive(c, 4000, 2000);
    const auto old = c.status().generation;
    arm(c, Choice::rc, Choice::sum);
    CHECK(c.status().generation != old && c.status().reason == Reason::wait_drive && !c.status().selected_drive);
}

void bench_and_pan() {
    Control c(profile);
    c.tick(150);
    arm(c);
    drive(c, 2500, 0);
    c.tick(249);
    CHECK(c.pan({{{}, 2, 0}, -10000}) == Code::ok);
    c.tick(250);
    CHECK(c.status().mode == Mode::neutral && c.status().cancel_reason == Reason::drive_timeout);
    CHECK(c.status().pan_pwm == 0);
    c.tick(500);
    CHECK(c.status().pan_pwm == 1500);
    c.tick(540);
    CHECK(c.status().pan_pwm == 600);
    c.link_lost();
    CHECK(c.status().pan_pwm == 600 && c.status().pan_seq == 0);
    CHECK(c.request({{}, 0, Mode::hold}) == Code::ok);
    const auto request = [&](Operation op, uint16_t pulse = 0) {
        return c.bench({{}, c.status().generation, op, pulse});
    };
    CHECK(request(Operation::enter) == Code::conflict);
    c.tick(600);
    CHECK(request(Operation::enter) == Code::ok);
    CHECK(c.pan({{{}, 3, 0}, 10000}) == Code::conflict);
    CHECK(request(Operation::pulse, 2000) == Code::conflict);
    c.tick(660);
    CHECK(request(Operation::pulse, 2001) == Code::bad_range);
    CHECK(request(Operation::pulse, 2000) == Code::ok);
    CHECK(c.status().drive_pwm == 2000 && c.status().steer_pwm == 1500 && c.status().pan_pwm == 600);
    for (uint64_t t = 760; t < 15660; t += 100) {
        c.tick(t);
        CHECK(request(Operation::pulse, 2000) == Code::ok);
    }
    c.tick(15660);
    CHECK(c.status().mode == Mode::hold && c.status().drive_pwm == 1500);
    CHECK(request(Operation::pulse, 2000) == Code::conflict);
    c.tick(15720);
    CHECK(request(Operation::enter) == Code::ok);
    c.tick(15780);
    CHECK(request(Operation::pulse, 2000) == Code::ok);
    c.link_lost();
    CHECK(c.status().mode == Mode::hold && c.status().drive_pwm == 1500);
}

template<class T> Text packet(T msg, Line &line) {
    line = encode(msg);
    CHECK(line.size > 0 && line.view().back() == '\n');
    return line.view().substr(0, line.size - 1);
}

void session() {
    Device d(profile, {{}, {}, device_id, boot_id, "RCCAR", "board", "build", "profile"});
    Line line;
    d.tick(150);
    CHECK(d.receive(packet(Hello{challenge}, line), 1).size);
    CHECK(d.status().session == first_session);
    d.status_sent(150);
    const auto old = d.status().generation;
    auto result = d.receive(packet(ModeRequest{{first_session, 1, 150}, old,
        Mode::automatic, Choice::host, Choice::host}, line), 1);
    Fields result_fields(result.view().substr(0, result.size - 1));
    Result r;
    CHECK(read(result_fields, r) && r.code == Code::ok);
    d.receive(packet(Drive{{first_session, 2, 149}, r.generation, 10000, 0}, line), 1);
    CHECK(d.status().drive_pwm == 1500);
    d.receive(packet(Drive{{first_session, 2, 150}, old, 10000, 0}, line), 1);
    CHECK(d.status().drive_pwm == 1500);
    d.receive(packet(Drive{{first_session, 2, 150}, r.generation, 10000, 0}, line), 1);
    CHECK(d.status().drive_pwm == 1750);
    d.receive(packet(Drive{{first_session, 2, 150}, r.generation, -10000, 0}, line), 1);
    CHECK(d.status().drive_pwm == 1750);
    d.receive(packet(Hello{challenge}, line), 2);
    CHECK(d.status().session == first_session && d.status().drive_pwm == 1750);
    d.receive(packet(Hello{"123456789ABCDEF0"}, line), 2);
    CHECK(d.status().session == second_session && d.status().drive_pwm == 1500);
    CHECK(d.status().cancel_reason == Reason::link_lost);
    d.status_sent(150);
    d.tick(300);
    result = d.receive(packet(ModeRequest{{second_session, 1, 150}, d.status().generation,
        Mode::automatic, Choice::host, Choice::host}, line), 2);
    Fields stale(result.view().substr(0, result.size - 1));
    CHECK(read(stale, r) && r.code == Code::stale);
}

std::string fixture(const std::string &dir, Text name) {
    std::ifstream file(dir + "/" + std::string(name) + ".wire", std::ios::binary);
    CHECK(file);
    return {std::istreambuf_iterator<char>(file), {}};
}

void codec(const std::string &dir) {
    CHECK(crc32("123456789") == 0xcbf43926u);
    CHECK(encode(Hello{challenge}).view() == fixture(dir, "hello"));
    CHECK(encode(Drive{{challenge, 2, 1234}, 7, -2500, 500}).view() == fixture(dir, "drive"));
    Stream stream;
    const auto valid_line = fixture(dir, "drive");
    for (char ch : std::string(max_frame + 2, 'X') + "\n") CHECK(stream.push(ch, 0).empty());
    auto check = [&](uint64_t t) {
        for (std::size_t i = 0; i < valid_line.size(); ++i) {
            auto record = stream.push(valid_line[i], t);
            if (i + 1 == valid_line.size()) {
                Fields fields(record);
                Drive d;
                CHECK(read(fields, d) && d.drive == -2500);
            } else CHECK(record.empty());
        }
    };
    check(1);
    CHECK(stream.push('@', 5).empty());
    for (char ch : valid_line) CHECK(stream.push(ch, 105).empty());
    check(106);
    auto bad = valid_line;
    bad[12] = '!';
    Fields corrupt(Text(bad).substr(0, bad.size() - 1));
    Drive d;
    CHECK(!read(corrupt, d));
    int q{};
    uint64_t u{};
    CHECK(!integer("-0", q) && !integer("+1", q) && !integer("01", q) &&
        !integer("18446744073709551616", u) && integer("18446744073709551615", u));
}
}

int main(int argc, char **argv) {
    CHECK(argc == 2);
    codec(argv[1]);
    quality_and_handover();
    choices();
    bench_and_pan();
    session();
    std::cout << "RC1 codec, handover, axis choices, maintenance and session checks passed\n";
}
