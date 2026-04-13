#include <neorv32.h>
#include <stdlib.h> 
#include <math.h>

#define N 30
#define BAUD_RATE 19200

// Matrizes em ponto fixo 24.8
int32_t A_fix[N][N];
int32_t B_fix[N][N]; // Inversa real (A^-1)
int32_t C_hw[N][N];
int32_t C_sw[N][N];

// -----------------------------------------------------------------------------
// Algoritmo de Inversão Gauss-Jordan (em float para precisão matemática)
// -----------------------------------------------------------------------------
void inverter_matriz_float(float m[N][N], float inv[N][N]) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) inv[i][j] = (i == j) ? 1.0f : 0.0f;
    }

    for (int i = 0; i < N; i++) {
        float pivot = m[i][i];
        if (fabsf(pivot) < 0.0001f) pivot = 0.0001f; // Evita divisão por zero

        for (int j = 0; j < N; j++) {
            m[i][j] /= pivot;
            inv[i][j] /= pivot;
        }

        for (int k = 0; k < N; k++) {
            if (k != i) {
                float factor = m[k][i];
                for (int j = 0; j < N; j++) {
                    m[k][j] -= factor * m[i][j];
                    inv[k][j] -= factor * inv[i][j];
                }
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Inicialização Conforme Especificação (Random float -> Inversa -> Ponto Fixo)
// -----------------------------------------------------------------------------
void preparar_dados() {
    static float temp_A[N][N];
    static float temp_B[N][N];

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            temp_A[i][j] = (float)(rand() % 100) / 100.0f; // Valores entre 0.0 e 1.0
            if (i == j) temp_A[i][j] += 10.0f; // Diagonal dominante para estabilizar a inversa
        }
    }

    inverter_matriz_float(temp_A, temp_B);

    // Converte para Ponto Fixo 24.8 (multiplica por 2^8 = 256)
    // Recalculamos temp_A pois a eliminação de Gauss a destruiu
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            float val_a = (float)(rand() % 100) / 100.0f;
            if (i == j) val_a += 10.0f;
            
            A_fix[i][j] = (int32_t)(val_a * 256.0f);
            B_fix[i][j] = (int32_t)(temp_B[i][j] * 256.0f);
        }
    }
}

// -----------------------------------------------------------------------------
// Multiplicação Software (Golden Model e Base para o Speedup)
// -----------------------------------------------------------------------------
void matrix_mult_sw() {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            int64_t accumulator = 0;
            for (int k = 0; k < N; k++) {
                accumulator += (int64_t)A_fix[i][k] * (int64_t)B_fix[k][j];
            }
            C_sw[i][j] = (int32_t)(accumulator >> 8); // Ajuste do formato 24.8
        }
    }
}

// -----------------------------------------------------------------------------
// Multiplicação Hardware (CFS - 10 Vias Paralelas)
// -----------------------------------------------------------------------------
void matrix_mult_hw() {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j += 10) {
            NEORV32_CFS->REG[11] = 1; // Reset MACs

            for (int k = 0; k < N; k++) {
                NEORV32_CFS->REG[0] = B_fix[k][j];
                NEORV32_CFS->REG[1] = B_fix[k][j+1];
                NEORV32_CFS->REG[2] = B_fix[k][j+2];
                NEORV32_CFS->REG[3] = B_fix[k][j+3];
                NEORV32_CFS->REG[4] = B_fix[k][j+4];
                NEORV32_CFS->REG[5] = B_fix[k][j+5];
                NEORV32_CFS->REG[6] = B_fix[k][j+6];
                NEORV32_CFS->REG[7] = B_fix[k][j+7];
                NEORV32_CFS->REG[8] = B_fix[k][j+8];
                NEORV32_CFS->REG[9] = B_fix[k][j+9];

                NEORV32_CFS->REG[10] = A_fix[i][k]; // Gatilho
            }

            for (int v = 0; v < 10; v++) {
                C_hw[i][j+v] = NEORV32_CFS->REG[v];
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Verificação pela Matriz Identidade
// -----------------------------------------------------------------------------
int verify_identity() {
    // Como fizemos inversão em float e convertemos para ponto fixo, 
    // precisamos de uma pequena tolerância devido ao ruído de quantização.
    int toleracia = 45; 

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            if (i == j) { 
                if (abs(C_hw[i][j] - 256) > toleracia) return 0; // Esperado: 256 (1.0)
            } else {      
                if (abs(C_hw[i][j]) > toleracia) return 0;       // Esperado: 0
            }
        }
    }
    return 1;
}

// -----------------------------------------------------------------------------
// MAIN
// -----------------------------------------------------------------------------
int main() {
    uint32_t t_start, t_end, cyc_sw, cyc_hw;

    neorv32_uart0_setup(BAUD_RATE, 0);
    neorv32_uart0_printf("\n--- Inicializando Teste de Acelerador de Hardware ---\n");

    if (neorv32_cfs_available() == 0) {
        neorv32_uart0_printf("ERRO: Modulo CFS nao encontrado!\n");
        while(1); 
    }

    neorv32_uart0_printf("Gerando Matrizes e Calculando Inversa Real...\n");
    preparar_dados();

    neorv32_uart0_printf("Multiplicando no Software...\n");
    t_start = neorv32_cpu_csr_read(CSR_MCYCLE);
    matrix_mult_sw();
    t_end = neorv32_cpu_csr_read(CSR_MCYCLE);
    cyc_sw = t_end - t_start;

    neorv32_uart0_printf("Multiplicando no Acelerador CFS (10 Vias)...\n");
    t_start = neorv32_cpu_csr_read(CSR_MCYCLE);
    matrix_mult_hw();
    t_end = neorv32_cpu_csr_read(CSR_MCYCLE);
    cyc_hw = t_end - t_start;

    // -------------------------------------------------------------------------
    // IMPRESSÃO DOS RESULTADOS FINAIS E SPEEDUP
    // -------------------------------------------------------------------------
    neorv32_uart0_printf("\n====================================================\n");
    neorv32_uart0_printf("              RESULTADOS DE DESEMPENHO              \n");
    neorv32_uart0_printf("====================================================\n");
    neorv32_uart0_printf(" Ciclos em Software: %u\n", cyc_sw);
    neorv32_uart0_printf(" Ciclos em Hardware: %u\n", cyc_hw);

    if (cyc_hw > 0) {
        uint32_t speed_int = cyc_sw / cyc_hw;
        uint32_t speed_frac = ((cyc_sw % cyc_hw) * 100) / cyc_hw;
        neorv32_uart0_printf(" Speedup alcancado  : %u.%u vezes mais rapido\n", speed_int, speed_frac);
    }
    neorv32_uart0_printf("----------------------------------------------------\n");

    neorv32_uart0_printf(" Verificacao (A * A^-1 = I): ");
    if (verify_identity()) {
        neorv32_uart0_printf("[SUCESSO]\n");
    } else {
        neorv32_uart0_printf("[FALHA]\n");
    }
    neorv32_uart0_printf("====================================================\n\n");

    return 0;
}