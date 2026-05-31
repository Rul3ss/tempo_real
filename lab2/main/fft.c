#include "fft.h"
#include <math.h>

#include "config.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
