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

#define NUM_TAREFAS 9
#define MS_TO_US(ms) ((ms) * 1000ULL)

static int bloco_atual = -1;
static int16_t *raw_data_atual = NULL;

static int16_t filtered_data[BLOCK_SIZE];

static circular_buffer freq_cbuf;
static float soma_freqs = 0.0f;

static float energia = 0.0f;
static float energia_evento = 0.0f;

static float freq_dominante = 0.0f;
static float freq_suavizada = 0.0f;

static NotaInfo nota_atual = {"--", 0.0f, "silencio"};

static bool flag_evento_pendente = false;
static bool flag_bloco_pronto = false;
static bool flag_filtrado_pronto = false;
static bool flag_energia_pronta = false;
static bool flag_fft_pronta = false;
static bool flag_media_pronta = false;

void tarefa_aquisicao_i2s(void);
void tarefa_pega_bloco(void);
void tarefa_filtro_fir(void);
void tarefa_calcula_energia(void);
void tarefa_detecta_evento(void);
void tarefa_fft(void);
void tarefa_media_movel(void);
void tarefa_identifica_nota(void);
void tarefa_impressao(void);

typedef void (*FuncTarefa)(void);

typedef struct
{
    const char *nome;

    uint64_t periodo_us;
    uint64_t proxima_execucao_us;

    volatile bool flag;

    FuncTarefa func;

} Tarefa;

Tarefa tabela[NUM_TAREFAS] = {
    {"AQUISICAO", MS_TO_US(1000), 500, false, tarefa_aquisicao_i2s},
    {"PEGA_BLOCO", MS_TO_US(1000), 0, false, tarefa_pega_bloco},
    {"FILTRO_FIR", MS_TO_US(1000), 0, false, tarefa_filtro_fir},
    {"ENERGIA", MS_TO_US(1000), 0, false, tarefa_calcula_energia},
    {"DETECTA_EVENTO", MS_TO_US(1000), 0, false, tarefa_detecta_evento},
    {"FFT", MS_TO_US(1280), 0, false, tarefa_fft},
    {"MEDIA_MOVEL", MS_TO_US(1280), 0, false, tarefa_media_movel},
    {"IDENTIFICA_NOTA", MS_TO_US(1280), 0, false, tarefa_identifica_nota},
    {"IMPRESSAO", MS_TO_US(2560), 0, false, tarefa_impressao}};

static bool IRAM_ATTR timer_64ms_cb(
    gptimer_handle_t timer,
    const gptimer_alarm_event_data_t *edata,
    void *user_ctx)
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
        .alarm_count = 32000, // 64 ms = 64000 us   32000
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

void tarefa_pega_bloco(void)
{
    if (flag_bloco_pronto)
    {
        return;
    }

    bloco_atual = pp_get_ready_block(&ppbuf);

    if (bloco_atual == -1)
    {
        raw_data_atual = NULL;
        return;
    }

    raw_data_atual = pp_get_block_data(&ppbuf, bloco_atual);

    flag_bloco_pronto = true;
}

void tarefa_filtro_fir(void)
{
    if (!flag_bloco_pronto)
    {
        return;
    }

    if (raw_data_atual == NULL)
    {
        return;
    }

    apply_fir(raw_data_atual, filtered_data, BLOCK_SIZE);

    flag_filtrado_pronto = true;
}

void tarefa_calcula_energia(void)
{
    if (!flag_filtrado_pronto)
    {
        return;
    }

    energia = system_energy(filtered_data, BLOCK_SIZE);

    flag_energia_pronta = true;
}

void tarefa_detecta_evento(void)
{
    if (!flag_energia_pronta)
    {
        return;
    }

    if (detecta_evento(energia))
    {
        flag_evento_pendente = true;
        energia_evento = energia;
    }

    if (bloco_atual != -1)
    {
        pp_release_block(&ppbuf, bloco_atual);
    }

    bloco_atual = -1;
    raw_data_atual = NULL;

    flag_bloco_pronto = false;
    flag_filtrado_pronto = false;
    flag_energia_pronta = false;
}

void tarefa_fft(void)
{
    freq_dominante = 0.0f;

    calcular_fft_e_frequencia(filtered_data, &freq_dominante);

    flag_fft_pronta = true;
}

void tarefa_media_movel(void)
{
    if (!flag_fft_pronta)
    {
        return;
    }

    freq_suavizada = calcular_media_movel(
        &freq_cbuf,
        &soma_freqs,
        freq_dominante);

    flag_media_pronta = true;
}

void tarefa_identifica_nota(void)
{
    if (!flag_media_pronta)
    {
        return;
    }

    nota_atual = identificar_nota_musical(freq_suavizada);

    flag_fft_pronta = false;
    flag_media_pronta = false;
}

void tarefa_impressao(void)
{
    imprime_evento(
        flag_evento_pendente,
        energia_evento,
        freq_dominante,
        freq_suavizada,
        nota_atual);

    flag_evento_pendente = false;
}

void app_main(void)
{
    esp_task_wdt_deinit();
    pp_init(&ppbuf);
    init_i2s_inmp441();
    init_gptimer_64ms();

    while (1)
    {
        for (int i = 0; i < NUM_TAREFAS; i++)
        {
            if (tabela[i].flag)
            {
                tabela[i].flag = false;
                tabela[i].func();

                break;
            }
        }
    }
}