#include "WS2812.h"

#include "esp_log.h"
#include "esp_err.h"
#include "driver/rmt_types.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"

static const char* TAG = "WS2812";

/**
 * @brief Construct a new WS2812::WS2812 object
 * using by default pin 8 for power and pin 5 for rmt
 * 
 */
WS2812::WS2812() {
    this->power_pin = GPIO_NUM_8;
    this->tx_pin = GPIO_NUM_5;
    this->init();
}

/**
 * @brief Construct a new WS2812::WS2812 object
 * 
 * @param power_pin 
 * @param tx_pin 
 */
WS2812::WS2812(gpio_num_t power_pin, gpio_num_t tx_pin) {
    this->power_pin = power_pin;
    this->tx_pin = tx_pin;
    this->init();
}

/**
 * @brief  Initialize the gpio pins and the rmt channel
 * 
 */
void WS2812::init(void) {
    ESP_ERROR_CHECK(gpio_reset_pin(this->power_pin));
    ESP_ERROR_CHECK(gpio_set_direction(this->power_pin,GPIO_MODE_OUTPUT));

    //start turned off
    this->off();
    
    this->rmt_tx_channel_config.gpio_num = this->tx_pin;
    this->rmt_tx_channel_config.clk_src = RMT_CLK_SRC_DEFAULT;
    this->rmt_tx_channel_config.resolution_hz = RMT_FREQUENCY;
    this->rmt_tx_channel_config.mem_block_symbols = 64;
    this->rmt_tx_channel_config.trans_queue_depth = 4;
    this->rmt_tx_channel_config.intr_priority = 0;
    this->rmt_tx_channel_config.flags.invert_out = 0;
    this->rmt_tx_channel_config.flags.io_loop_back = 0;
    this->rmt_tx_channel_config.flags.io_od_mode = 0;
    this->rmt_tx_channel_config.flags.with_dma = 0;
    
    rmt_simple_encoder_config_t rgb_encoder;
    rgb_encoder.callback = &this->callback_encoder;
    rgb_encoder.arg = 0;
    rgb_encoder.min_chunk_size = 8;

    ESP_ERROR_CHECK(rmt_new_simple_encoder(&rgb_encoder, &this->simple_encoder));
    ESP_ERROR_CHECK(rmt_new_tx_channel(&this->rmt_tx_channel_config, &this->tx_chan));
}

void WS2812::on(void) {
    ESP_ERROR_CHECK(gpio_set_level(this->power_pin,1));
}

void WS2812::off(void) {
    ESP_ERROR_CHECK(gpio_set_level(this->power_pin,0));
}

/**
 * @brief 
 * 
 * @param r: red color scales from 0 (0%) to 255 (100%)
 * @param g: green color scaled from 0 to 255 (100%)
 * @param b: blue color scaled from 0 to 255 (100%)
 * @return esp_err_t 
 */
esp_err_t WS2812::set_rgb(uint8_t r, uint8_t g, uint8_t b) {

    uint8_t payload[3] = {g, r, b};

    rmt_transmit_config_t tx_conf;
    tx_conf.loop_count = 0;
    tx_conf.flags.eot_level = 0;
    tx_conf.flags.queue_nonblocking = true;

    ESP_ERROR_CHECK(rmt_enable(this->tx_chan));
    ESP_ERROR_CHECK(rmt_transmit(this->tx_chan, this->simple_encoder, payload, 3, &tx_conf));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(this->tx_chan, -1));
    ESP_ERROR_CHECK(rmt_disable(this->tx_chan));
    ESP_LOGV(TAG, "Transmitted RGB {%d, %d, %d}",r, g, b);

    return ESP_OK;
}

/**
 * @brief 
 * 
 * @param data: {G,R,B} values to send to the WS2812
 * @param data_size: Number of bytes to transmit
 * @param symbols_written: Number of symboles already written to the RMT device
 * @param symbols_free: Number of symbols free to be added to the RMT device queue
 * @param symbols: rmt_symbol_word_t array that the callback fills
 * @param done: completion indicator
 * @param arg: argument (unused)
 * @return size_t: The number of symbols transmitted
 */
size_t WS2812::callback_encoder(const void *data, size_t data_size,
                                         size_t symbols_written, size_t symbols_free,
                                         rmt_symbol_word_t *symbols, bool *done, void *arg) {

    if(symbols_free < 8) { return 0; }

    size_t data_pos = symbols_written / 8;
    uint8_t *data_bytes = (uint8_t*)data;
    if (data_pos < data_size) {
        size_t symbol_pos = 0;
        for (int bitmask = 0x80; bitmask != 0; bitmask >>= 1) {
            if (data_bytes[data_pos]&bitmask) {
                symbols[symbol_pos++] = WS2812::one;
            } else {
                symbols[symbol_pos++] = WS2812::zero;
            }
        }
        return symbol_pos;
    } else {
        symbols[0] = WS2812::reset;
        *done = 1;
        return 1;
    }
}