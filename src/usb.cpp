#include "build_config.hpp"
#include "pico/unique_id.h"
#include "tusb.h"
#include <array>

namespace {
#if defined(PICO_RP2040)
constexpr uint16_t product_id = 0x000a;
#else
constexpr uint16_t product_id = 0x0009;
#endif
constexpr tusb_desc_device_t device{
    sizeof(tusb_desc_device_t), TUSB_DESC_DEVICE, 0x0200,
    TUSB_CLASS_MISC, MISC_SUBCLASS_COMMON, MISC_PROTOCOL_IAD,
    CFG_TUD_ENDPOINT0_SIZE,
    0x2e8a, product_id, 0x0100,
    1, 2, 3, 1
};
constexpr auto config_size = TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN;
constexpr uint8_t config[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, config_size, 0, 100),
    TUD_CDC_DESCRIPTOR(0, 0, 0x81, 8, 0x02, 0x82, 64),
};
}

uint8_t const *tud_descriptor_device_cb() {
    return reinterpret_cast<const uint8_t *>(&device);
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    return index == 0 ? config : nullptr;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t language) {
    static std::array<uint16_t, 32> descriptor{};
    if (index == 0) {
        descriptor[0] = (TUSB_DESC_STRING << 8) | 4;
        descriptor[1] = 0x0409;
        return descriptor.data();
    }
    if (language != 0x0409) return nullptr;
    char id[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    const char *text = nullptr;
    if (index == 1) text = "RC Car";
    if (index == 2) text = "RC Car Pico";
    if (index == 3) {
        pico_get_unique_board_id_string(id, sizeof(id));
        text = id;
    }
    if (!text) return nullptr;
    unsigned length = 0;
    while (text[length] && length < descriptor.size() - 1) {
        descriptor[length + 1] = static_cast<uint8_t>(text[length]);
        ++length;
    }
    descriptor[0] = (TUSB_DESC_STRING << 8) | (2 + 2 * length);
    return descriptor.data();
}
