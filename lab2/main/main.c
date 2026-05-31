#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_task_wdt.h"
#include "driver/i2s_std.h"

#include "config.h"
#include "bufferpp.h"
#include "bufferc.h"
#include "filtrofir.h"
#include "nota.h"
#include "evento.h"
#include "energy.h"
#include "fft.h"
#include "mediamovel.h"

//variáveis do pp
pingpong_buffer ppbuf;
static volatile bool overflow = false;

//variáveis do cb
circular_buffer cbuf;

volatile bool p1 = false;
volatile bool p2 = false;
volatile bool p3 = false;

static uint32_t tick_64ms = 0;

static bool IRAM_ATTR i2s_rx_on_recv_cb(i2s_chan_handle_t handle, i2s_event_data_t *event_data, void *user_ctx)
{
    tick_64ms ++;

    p1 = true;

    if(tick_64ms % 2 ==0){
        p2 = true;
    }
    if(tick_64ms % 4 ==0){
        p3 = true;
    }
    if (tick_64ms >= 4)
    {
        tick_64ms = 0;
    }
    return false;
}

i2s_chan_handle_t rx_chan;

void init_i2s_inmp441()
{
    // 1. Aloca o canal I2S de recepção (RX)
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

    chan_cfg.intr_priority = 3;

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));

    // 2. Configura os parâmetros de taxa de amostragem, bits e pinos
    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = 16000,
            .clk_src = I2S_CLK_SRC_DEFAULT,         // Adicione isso
            .mclk_multiple = I2S_MCLK_MULTIPLE_256, // Tente 256 ou 384
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_SCK,
            .ws = I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_SD,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    // 3. INICIALIZA O MODO PADRÃO (Esta é a linha crítica que estava faltando!)
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));

    i2s_event_callbacks_t cbs = {
        .on_recv = i2s_rx_on_recv_cb,
        .on_recv_q_ovf = NULL,
    };
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(rx_chan, &cbs, NULL));

    // 5. Finalmente, habilita (liga) a leitura do I2S
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
}

void tarefa_aquisicao_i2s(void)
{
    static int32_t raw_buffer[256];
    size_t bytes_read = 0;

    esp_err_t ret = i2s_channel_read(rx_chan, raw_buffer, sizeof(raw_buffer),
                                     &bytes_read, 0); // timeout = 0

    if (ret == ESP_OK && bytes_read > 0)
    {
        size_t samples_count = bytes_read / 4;

        for (int i = 0; i < samples_count; i++)
        {
            int16_t sample = (int16_t)(raw_buffer[i] >> 16);
            if (!pp_write(&ppbuf, sample))
            {
                overflow = true;
            }
        }
    }
}

void app_main(void)
{
    esp_task_wdt_deinit();
    pp_init(&ppbuf);
    init_i2s_inmp441();

    while(1){
        if(p1){
            p1=false;

        }
        if(p2){
            p2=false;

        }
        if(p3){
            p3=false;

        }

    }

}