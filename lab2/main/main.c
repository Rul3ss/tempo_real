#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdint.h>
#include "driver/gptimer.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "driver/i2s_std.h"
#include "esp_timer.h"
#include <stdbool.h>

// 1. Define I2S pins
#define I2S_WS 18
#define I2S_SD 32
#define I2S_SCK 14

#define BLOCK_SIZE 1024
#define SAMPLE_RATE 16000

#define LIMIAR 0.2f
#define FIR_TAPS 64

#define REFRACTORY_BLOCKS 10
#define WINDOW_SIZE 16

typedef struct
{
    char nome[16]; // Espaço extra para evitar o erro de -Werror=format-overflow=
    float cents;
    const char *status;
} NotaInfo;

typedef struct
{
    bool pendente;
    uint32_t bloco;
    float energia;
    float freq_dominante;
    float freq_suavizada;
    NotaInfo nota;
} EventoInfo;

// Vetor global de notas
const char *nomes_notas[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

const float fir_coeffs[FIR_TAPS] = {
    -0.0007935162222913056, 0.0007146182515437266, -0.0005346847066427102, 0.00021902057417850376, 0.0002624315353002918, -0.0009084314475446052, 0.0016612654023352095, -0.002392700808796657, 0.0029092282193549867, -0.002980718040576488, 0.002390795911811896, -0.0010008903993720315, -0.0011854414029316925, 0.003974204060515007, -0.006964398549262518, 0.009574591351556886, -0.011116423615189388, 0.010909359085913353, -0.00842143064288873, 0.003413273843233875, 0.003940780612420356, -0.012979047798182979, 0.022543279889327986, -0.031037952686057555, 0.03654396343464151, -0.036947916612718554, 0.030007788234816264, -0.01318186517114571, -0.017264416877092386, 0.06978778910915504, -0.17567925503844853, 0.6245367005030369, 0.6245367005030369, -0.1756792550384485, 0.06978778910915504, -0.017264416877092386, -0.01318186517114571, 0.030007788234816264, -0.03694791661271855, 0.03654396343464151, -0.03103795268605755, 0.022543279889327982, -0.012979047798182972, 0.003940780612420356, 0.0034132738432338734, -0.00842143064288873, 0.010909359085913348, -0.011116423615189388, 0.009574591351556883, -0.006964398549262513, 0.0039742040605150065, -0.0011854414029316917, -0.0010008903993720315, 0.0023907959118118945, -0.002980718040576488, 0.0029092282193549845, -0.002392700808796654, 0.0016612654023352078, -0.0009084314475446041, 0.0002624315353002918, 0.00021902057417850376, -0.0005346847066427102, 0.0007146182515437266, -0.0007935162222913056};

typedef struct
{
    int16_t buffer[2][BLOCK_SIZE];
    int write_index[2];
    volatile int active;
    volatile bool block_ready[2];
    volatile bool block_in_use[2];
} pingpong_buffer;

pingpong_buffer ppbuf;
static volatile bool overflow = false;

void pp_init(pingpong_buffer *pp)
{
    pp->active = 0;
    // buffer 1
    pp->write_index[0] = 0;
    pp->block_ready[0] = false;
    pp->block_in_use[0] = false;
    // buffer 2
    pp->write_index[1] = 0;
    pp->block_ready[1] = false;
    pp->block_in_use[1] = false;
}

int pp_write(pingpong_buffer *pp, int sample)
{
    int a = pp->active;

    // poderia colocar o overflow aqui direto
    if (pp->block_ready[a] || pp->block_in_use[a])
        return 0;

    pp->buffer[a][pp->write_index[a]++] = sample;

    if (pp->write_index[a] == BLOCK_SIZE)
    {
        pp->block_ready[a] = true;
        pp->write_index[a] = 0;
        pp->active = !a;
    }

    return 1;
}

int pp_get_ready_block(pingpong_buffer *pp)
{
    for (int i = 0; i < 2; i++)
    {
        if (pp->block_ready[i] && !pp->block_in_use[i])
        {
            pp->block_in_use[i] = true;
            return i;
        }
    }
    return -1;
}

int16_t *pp_get_block_data(pingpong_buffer *pp, int block_index)
{
    return pp->buffer[block_index];
}

void pp_release_block(pingpong_buffer *pp, int block_index)
{
    pp->block_ready[block_index] = false;
    pp->block_in_use[block_index] = false;
}

volatile bool tick_64ms = false;

static bool IRAM_ATTR i2s_rx_on_recv_cb(i2s_chan_handle_t handle, i2s_event_data_t *event_data, void *user_ctx)
{
    tick_64ms = true;
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

int16_t fir_history[FIR_TAPS - 1] = {0};

void apply_fir(int16_t *input, int16_t *output, int num_samples)
{
    for (int i = 0; i < num_samples; i++)
    {
        float acc = 0.0f;

        for (int k = 0; k < FIR_TAPS; k++)
        {
            int hist_idx = i - k;

            // Se o índice for negativo, pegamos do histórico do bloco passado
            float sample_val;
            if (hist_idx >= 0)
            {
                sample_val = input[hist_idx];
            }
            else
            {
                sample_val = fir_history[(FIR_TAPS - 1) + hist_idx];
            }

            acc += fir_coeffs[k] * sample_val;
        }
        output[i] = (int16_t)acc;
    }
    for (int i = 0; i < FIR_TAPS - 1; i++)
    {
        fir_history[i] = input[num_samples - (FIR_TAPS - 1) + i];
    }
}

float system_energy(int16_t *data, int num_samples)
{
    float soma_quadrados = 0.0f;

    for (int i = 0; i < num_samples; i++)
    {
        float sample = (float)data[i];
        // Normaliza a amostra individualmente para o intervalo [-1.0, 1.0]
        // antes de multiplicar, evitando estourar a capacidade do float (overflow)
        float sample_norm = sample / 32768.0f;
        soma_quadrados += (sample_norm * sample_norm);
    }

    // Calcula a energia média do bloco (agora garantida entre 0.0 e 1.0)
    float energia_media = soma_quadrados / num_samples;

    // Retorna o valor RMS (Root Mean Square)
    // Deixa a escala de energia linear e muito mais fácil de ler no console
    return sqrtf(energia_media);
}

void bit_reversal(float *real, float *imag, int n)
{
    int j = 0;
    for (int i = 0; i < n - 1; i++)
    {
        if (i < j)
        {
            float temp_r = real[i];
            float temp_i = imag[i];
            real[i] = real[j];
            imag[i] = imag[j];
            real[j] = temp_r;
            imag[j] = temp_i;
        }
        int k = n / 2;
        while (k <= j)
        {
            j -= k;
            k /= 2;
        }
        j += k;
    }
}

// Algoritmo FFT in-place (Substitui o dsps_fft2r_fc32)
void calcular_fft_radix2(float *real, float *imag, int n)
{
    // 1. Reorganiza os dados de entrada
    bit_reversal(real, imag, n);

    // 2. Executa os estágios da borboleta (butterflies)
    for (int len = 2; len <= n; len <<= 1)
    {
        float angle = -2.0f * M_PI / len;
        float wlen_r = cosf(angle);
        float wlen_i = sinf(angle);

        for (int i = 0; i < n; i += len)
        {
            int half_len = len / 2;
            float w_r = 1.0f;
            float w_i = 0.0f;

            for (int j = 0; j < half_len; j++)
            {
                int u_idx = i + j;
                int v_idx = i + j + half_len;

                // Multiplicação complexa: v * w
                float t_r = real[v_idx] * w_r - imag[v_idx] * w_i;
                float t_i = real[v_idx] * w_i + imag[v_idx] * w_r;

                // Combinação da borboleta
                real[v_idx] = real[u_idx] - t_r;
                imag[v_idx] = imag[u_idx] - t_i;
                real[u_idx] += t_r;
                imag[u_idx] += t_i;

                // Atualiza o fator de rotação (w = w * wlen)
                float next_w_r = w_r * wlen_r - w_i * wlen_i;
                w_i = w_r * wlen_i + w_i * wlen_r;
                w_r = next_w_r;
            }
        }
    }
}

void calcular_fft_e_frequencia(int16_t *data, float *freq_dominante)
{
    static float fft_real[BLOCK_SIZE];
    static float fft_imag[BLOCK_SIZE];

    // Aplica janela de Hann
    for (int i = 0; i < BLOCK_SIZE; i++)
    {
        float window = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (BLOCK_SIZE - 1)));
        fft_real[i] = (float)data[i] * window / 32768.0f; // Normaliza
        fft_imag[i] = 0.0f;
    }

    calcular_fft_radix2(fft_real, fft_imag, BLOCK_SIZE);

    float max_magnitude = 0.0f;
    int max_index = 0;

    // Ajuste: procure até 2000Hz para incluir 400Hz com folga
    int bin_min = (int)(60.0f * BLOCK_SIZE / SAMPLE_RATE);   // ~4
    int bin_max = (int)(2000.0f * BLOCK_SIZE / SAMPLE_RATE); // ~128

    for (int i = bin_min; i < bin_max; i++)
    {
        float magnitude = fft_real[i] * fft_real[i] + fft_imag[i] * fft_imag[i];
        if (magnitude > max_magnitude)
        {
            max_magnitude = magnitude;
            max_index = i;
        }
    }

    // INTERPOLAÇÃO PARABÓLICA para melhor precisão
    if (max_index > 0 && max_index < BLOCK_SIZE / 2 - 1)
    {
        float mag_left = fft_real[max_index - 1] * fft_real[max_index - 1] +
                         fft_imag[max_index - 1] * fft_imag[max_index - 1];
        float mag_center = max_magnitude;
        float mag_right = fft_real[max_index + 1] * fft_real[max_index + 1] +
                          fft_imag[max_index + 1] * fft_imag[max_index + 1];

        float correction = 0.5f * (mag_left - mag_right) /
                           (mag_left - 2.0f * mag_center + mag_right);

        *freq_dominante = (float)(max_index + correction) * SAMPLE_RATE / BLOCK_SIZE;
    }
    else
    {
        *freq_dominante = (float)max_index * SAMPLE_RATE / BLOCK_SIZE;
    }
}

typedef struct
{
    float *buffer;
    int size;
    unsigned int head;
    unsigned int tail;
    int count;
} circular_buffer;

circular_buffer cbuf;

void cb_init(circular_buffer *buf, int size)
{
    buf->buffer = (float *)malloc(size * sizeof(float));
    buf->head = 0;
    buf->tail = 0;
    buf->size = size;
    buf->count = 0;
}

int cb_get_avail(circular_buffer *buf)
{
    return buf->size - buf->count;
}

int cb_get_filled(circular_buffer *buf)
{
    return buf->count;
}

int cb_push(circular_buffer *buf, float data)
{
    if (cb_get_avail(buf) < 1)
    {
        return 0;
    }

    buf->buffer[buf->head] = data;
    buf->head = (buf->head + 1) % buf->size;
    buf->count++;
    return 1;
}

int cb_pop(circular_buffer *buf, float *data)
{
    if (cb_get_filled(buf) < 1)
    {
        return 0; // Buffer vazio
    }

    *data = buf->buffer[buf->tail];
    buf->tail = (buf->tail + 1) % buf->size;
    buf->count--;
    return 1;
}

float calcular_media_movel(circular_buffer *buf, float *soma_atual, float nova_amostra)
{
    if (cb_get_filled(buf) == buf->size)
    {
        float valor_antigo = 0.0f;
        cb_pop(buf, &valor_antigo);
        *soma_atual -= valor_antigo;
    }
    cb_push(buf, nova_amostra);
    *soma_atual += nova_amostra;
    int preenchidos = cb_get_filled(buf);
    if (preenchidos == 0)
        return 0.0f;

    return *soma_atual / preenchidos;
}

NotaInfo identificar_nota_musical(float freq_suavizada)
{
    NotaInfo info = {.nome = "--", .cents = 0.0f, .status = "silencio"};

    if (freq_suavizada < 20.0f)
        return info;

    int midi = (int)roundf(69.0f + 12.0f * log2f(freq_suavizada / 440.0f));
    float f_ref = 440.0f * powf(2.0f, (midi - 69) / 12.0f);
    info.cents = 1200.0f * log2f(freq_suavizada / f_ref);

    int nota_indice = midi % 12;
    int oitava = (midi / 12) - 1;
    if (nota_indice < 0)
        nota_indice += 12;

    snprintf(info.nome, sizeof(info.nome), "%s%d", nomes_notas[nota_indice], oitava);

    if (fabsf(info.cents) <= 10.0f)
    {
        info.status = "afinado";
    }
    else if (info.cents < 0.0f)
    {
        info.status = "abaixo";
    }
    else
    {
        info.status = "acima";
    }

    return info;
}

static uint32_t block_count = 0;

bool detecta_evento(float energy)
{
    static int64_t ultimo_bloco_evento = -1000;

    if (energy >= LIMIAR)
    {
        if ((block_count - ultimo_bloco_evento) > REFRACTORY_BLOCKS)
        {
            ultimo_bloco_evento = block_count;
            return true;
        }
    }

    return false;
}

void imprime_evento(bool flag_evento, float energy, float freq_dominante, float freq_suave, NotaInfo nota)
{
    if (!flag_evento)
    {
        return;
    }

    float t_s = block_count * (BLOCK_SIZE / 16000.0f); // tempo em segundos

    printf("*** EVENTO DETECTADO ***\n");
    printf("Bloco #%lu | t = %.3f s\n", block_count, t_s);
    printf("Energia = %.3f\n", energy);
    printf("Freq. dominante = %.1f Hz\n", freq_dominante);
    printf("Freq. dominante Suavizada = %.1f Hz\n", freq_suave);
    printf("Nota = %s\n", nota.nome);
    printf("Erro = %+.1f cents\n", nota.cents);
    printf("Status = %s\n", nota.status);
    printf("------------------------\n");
}

void app_main(void)
{
    esp_task_wdt_deinit();
    pp_init(&ppbuf);
    init_i2s_inmp441();

    static int16_t filtered_data[BLOCK_SIZE];
    circular_buffer freq_cbuf;
    float soma_freqs = 0.0f;

    cb_init(&freq_cbuf, WINDOW_SIZE);

    int slot = 0;

    float energia_evento = 0.0f;
    float freq_dominante = 0.0f;
    float freq_suavizada = 0.0f;
    NotaInfo nota_atual = {"--", 0.0f, "silencio"};
    bool flag_evento_pendente = false;
    float energia = 0.0f;

    while (true)
    {
        tarefa_aquisicao_i2s();

        if (tick_64ms)
        {
            tick_64ms = false;

            int block = pp_get_ready_block(&ppbuf);
            if (block != -1)
            {
                int16_t *raw_data = pp_get_block_data(&ppbuf, block);

                switch (slot)
                {
                case 0:

                    apply_fir(raw_data, filtered_data, BLOCK_SIZE);
                    energia = system_energy(filtered_data, BLOCK_SIZE);
                    if (detecta_evento(energia))
                    {
                        flag_evento_pendente = true;
                        energia_evento = energia;
                    }
                    break;

                case 1:
                    apply_fir(raw_data, filtered_data, BLOCK_SIZE);
                    energia = system_energy(filtered_data, BLOCK_SIZE);

                    if (detecta_evento(energia))
                    {
                        flag_evento_pendente = true;
                        energia_evento = energia;
                    }

                    freq_dominante = 0.0f;
                    calcular_fft_e_frequencia(filtered_data, &freq_dominante);
                    freq_suavizada = calcular_media_movel(&freq_cbuf, &soma_freqs, freq_dominante);
                    nota_atual = identificar_nota_musical(freq_suavizada);
                    break;

                case 2:
                    apply_fir(raw_data, filtered_data, BLOCK_SIZE);
                    energia = system_energy(filtered_data, BLOCK_SIZE);
                    if (detecta_evento(energia))
                    {
                        flag_evento_pendente = true;
                        energia_evento = energia;
                    }
                    break;

                case 3:
                    apply_fir(raw_data, filtered_data, BLOCK_SIZE);
                    energia = system_energy(filtered_data, BLOCK_SIZE);

                    if (detecta_evento(energia))
                    {
                        flag_evento_pendente = true;
                        energia_evento = energia;
                    }

                    freq_dominante = 0.0f;
                    calcular_fft_e_frequencia(filtered_data, &freq_dominante);
                    freq_suavizada = calcular_media_movel(&freq_cbuf, &soma_freqs, freq_dominante);
                    nota_atual = identificar_nota_musical(freq_suavizada);

                    imprime_evento(flag_evento_pendente, energia_evento, freq_dominante, freq_suavizada, nota_atual);

                    flag_evento_pendente = false;
                    break;
                }

                block_count++;

                slot++;
                if (slot >= 4)
                {
                    slot = 0;
                }

                pp_release_block(&ppbuf, block);
            }
        }
    }
}
