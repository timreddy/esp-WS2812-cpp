#pragma once

#include <stdint.h>

#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_types.h"

#define RMT_FREQUENCY 20000000  // 20 Mhz, 1 tick = 0.05 us
#define T0H_TICKS (350 / (1000000000 / RMT_FREQUENCY))
#define T0L_TICKS (700 / (1000000000 / RMT_FREQUENCY))
#define T1H_TICKS (800 / (1000000000 / RMT_FREQUENCY))
#define T1L_TICKS (600 / (1000000000 / RMT_FREQUENCY))
#define RESET_TICKS (50000 / (1000000000 / RMT_FREQUENCY))


class WS2812 {
    private:
        gpio_num_t power_pin;
        gpio_num_t tx_pin;

        uint8_t r, g, b;

        constexpr static rmt_symbol_word_t zero  = {T0H_TICKS,1,T0L_TICKS,0}; // { .duration0, .level0, .duration1, .level1 }
        constexpr static rmt_symbol_word_t one   = {T1H_TICKS,1,T1L_TICKS,0};
        constexpr static rmt_symbol_word_t reset = {RESET_TICKS,0,0,0};

        rmt_tx_channel_config_t rmt_tx_channel_config;
        rmt_channel_handle_t tx_chan;
        rmt_encoder_handle_t simple_encoder;

        static size_t callback_encoder(const void *data, size_t data_size,
                                    size_t symbols_written, size_t symbols_free,
                                    rmt_symbol_word_t *symbols, bool *done, void *arg);

        void init(void);

    public:
        WS2812();
        WS2812(gpio_num_t power_pin, gpio_num_t tx_pin);

        void on(void);
        void off(void);

        esp_err_t set_rgb(uint8_t r, uint8_t g, uint8_t b);
};




