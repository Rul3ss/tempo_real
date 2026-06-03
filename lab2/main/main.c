#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_task_wdt.h"
#include "driver/i2s_std.h"
#include "driver/gptimer.h"
#include "esp_timer.h"

#include "config.h"
#include "bufferpp.h"
#include "bufferc.h"
#include "filtrofir.h"
#include "nota.h"
#include "evento.h"
#include "energy.h"
#include "fft.h"
#include "mediamovel.h"

// variáveis do pp
pingpong_buffer ppbuf;
static volatile bool overflow = false;

// variáveis do cb
circular_buffer cbuf;

#define NUM_TAREFAS 3
#define MS_TO_US(ms) ((ms) * 1000ULL)

typedef struct
{
    const char *nome;

    uint64_t periodo_us;
    uint64_t proxima_execucao_us;

    volatile bool flag;
} Tarefa;

Tarefa tabela[NUM_TAREFAS] = {
    {"T1", MS_TO_US(64),  0, false},
    {"T2", MS_TO_US(128), 0, false},
    {"T3", MS_TO_US(256), 0, false}
};

static bool IRAM_ATTR timer_64ms_cb(
    gptimer_handle_t timer,
    const gptimer_alarm_event_data_t *edata,
    void *user_ctx
)
{
    uint64_t agora = esp_timer_get_time();

    for (int i = 0; i < NUM_TAREFAS; i++)
    {
        if (agora >= tabela[i].proxima_execucao_us)
        {
            tabela[i].flag = true;

            tabela[i].proxima_execucao_us = agora + tabela[i].periodo_us;
        }
    }

    return false;
}

gptimer_handle_t gptimer_64ms = NULL;

void init_gptimer_64ms(void)
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 MHz -> 1 tick = 1 us
        .intr_priority = 3,
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer_64ms));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_64ms_cb,
    };

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer_64ms, &cbs, NULL));

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 32000, // 64 ms = 64000 us
        .flags.auto_reload_on_alarm = true,
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_64ms, &alarm_config));

    ESP_ERROR_CHECK(gptimer_enable(gptimer_64ms));
    ESP_ERROR_CHECK(gptimer_start(gptimer_64ms));
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

    // 5. Finalmente, habilita (liga) a leitura do I2S
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
}

void tarefa_aquisicao_i2s(void)
{
    static int32_t raw_buffer[BLOCK_SIZE];
    size_t bytes_read = 0;

    esp_err_t ret = i2s_channel_read(rx_chan, raw_buffer, sizeof(raw_buffer),
                                     &bytes_read, 0); // timeout = 0

    if (ret == ESP_OK && bytes_read > 0)
    {
        size_t samples_count = bytes_read / sizeof(int32_t);

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
    init_gptimer_64ms();

    static int16_t filtered_data[BLOCK_SIZE];

    circular_buffer freq_cbuf;
    float soma_freqs = 0.0f;

    cb_init(&freq_cbuf, WINDOW_SIZE);

    float energia = 0.0f;
    float energia_evento = 0.0f;

    float freq_dominante = 0.0f;
    float freq_suavizada = 0.0f;

    NotaInfo nota_atual = {"--", 0.0f, "silencio"};

    bool flag_evento_pendente = false;

    while (1)
    {
        if (tabela[0].flag)
        {
            tabela[0].flag = false;

            tarefa_aquisicao_i2s();

            int block = pp_get_ready_block(&ppbuf);

            if (block != -1)
            {
                int16_t *raw_data = pp_get_block_data(&ppbuf, block);

                apply_fir(raw_data, filtered_data, BLOCK_SIZE);

                energia = system_energy(filtered_data, BLOCK_SIZE);

                if (detecta_evento(energia))
                {
                    flag_evento_pendente = true;
                    energia_evento = energia;
                }

                pp_release_block(&ppbuf, block);
            }
        }
        else if (tabela[1].flag)
        {
            tabela[1].flag = false;

            freq_dominante = 0.0f;

            calcular_fft_e_frequencia(filtered_data, &freq_dominante);

            freq_suavizada = calcular_media_movel(
                &freq_cbuf,
                &soma_freqs,
                freq_dominante);

            nota_atual = identificar_nota_musical(freq_suavizada);
        }
        else if (tabela[2].flag)
        {
            tabela[2].flag = false;

            imprime_evento(
                flag_evento_pendente,
                energia_evento,
                freq_dominante,
                freq_suavizada,
                nota_atual);

            flag_evento_pendente = false;
        }
    }
}