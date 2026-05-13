#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h> 
#include <math.h>

#define N 30
#define NUM_THREADS 10
#define STACK_SIZE 2048 // Aumentado para acomodar o Soft-Float

// -----------------------------------------------------------------------------
// MAPEAMENTO DIRETO DO ACELERADOR HARDWARE CFS (Substitui neorv32.h)
// -----------------------------------------------------------------------------
#define NEORV32_CFS_BASE 0xFFEB0000U

typedef struct {
    volatile uint32_t REG[32];
} neorv32_cfs_t;

#define NEORV32_CFS ((neorv32_cfs_t *) NEORV32_CFS_BASE)
// -----------------------------------------------------------------------------

// Matrizes em ponto fixo 24.8
int32_t A_fix[N][N];
int32_t B_fix[N][N]; 
int32_t C_hw[N][N];
int32_t C_sw[N][N];

// Estruturas do Zephyr para as Threads
K_THREAD_STACK_ARRAY_DEFINE(thread_stacks, NUM_THREADS, STACK_SIZE);
struct k_thread thread_data[NUM_THREADS];
struct k_sem sync_sem; // Semáforo para sincronizar o fim das threads

// --- Funções de Preparação e Hardware ---
void inverter_matriz_float(float m[N][N], float inv[N][N]) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) inv[i][j] = (i == j) ? 1.0f : 0.0f;
    }
    for (int i = 0; i < N; i++) {
        float pivot = m[i][i];
        if (fabsf(pivot) < 0.0001f) pivot = 0.0001f; 
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

void preparar_dados() {
    static float temp_A[N][N];
    static float temp_B[N][N];
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            temp_A[i][j] = (float)(rand() % 100) / 100.0f;
            if (i == j) temp_A[i][j] += 10.0f; 
        }
    }
    inverter_matriz_float(temp_A, temp_B);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            float val_a = (float)(rand() % 100) / 100.0f;
            if (i == j) val_a += 10.0f;
            A_fix[i][j] = (int32_t)(val_a * 256.0f);
            B_fix[i][j] = (int32_t)(temp_B[i][j] * 256.0f);
        }
    }
}

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

void matrix_mult_sw_thread(void *arg1, void *arg2, void *arg3) {
    int start_row = (int)(intptr_t)arg1;
    int end_row = (int)(intptr_t)arg2;

    for (int i = start_row; i < end_row; i++) {
        for (int j = 0; j < N; j++) {
            int64_t accumulator = 0;
            for (int k = 0; k < N; k++) {
                accumulator += (int64_t)A_fix[i][k] * (int64_t)B_fix[k][j];
            }
            C_sw[i][j] = (int32_t)(accumulator >> 8);
        }
    }
    
    // Avisa que esta thread terminou
    k_sem_give(&sync_sem);
}

int verify_identity() {
    int toleracia = 45; 
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            if (i == j) { 
                if (abs(C_hw[i][j] - 256) > toleracia) return 0;
            } else {      
                if (abs(C_hw[i][j]) > toleracia) return 0;
            }
        }
    }
    return 1;
}

int main(void) {
    uint32_t t_start, t_end, cyc_sw, cyc_hw;

    printk("\n--- Inicializando Teste com Zephyr RTOS ---\n");

    printk("Gerando Matrizes...\n");
    preparar_dados();

    // ------------------------------------------------------------------
    // MULTIPLICAÇÃO SOFTWARE COM THREADS
    // ------------------------------------------------------------------
    printk("Multiplicando no Software (Dividido em %d Threads)...\n", NUM_THREADS);
    
    // Inicializa o semáforo em 0
    k_sem_init(&sync_sem, 0, NUM_THREADS);
    
    int rows_per_thread = N / NUM_THREADS;

    t_start = k_cycle_get_32();
    
    // Cria e dispara as threads
    for (int i = 0; i < NUM_THREADS; i++) {
        int start_r = i * rows_per_thread;
        int end_r = (i + 1) * rows_per_thread;
        
        k_thread_create(&thread_data[i], thread_stacks[i], STACK_SIZE,
                        matrix_mult_sw_thread, 
                        (void *)(intptr_t)start_r, 
                        (void *)(intptr_t)end_r, NULL,
                        5, 0, K_NO_WAIT); // Prioridade 5
    }

    // O Main aguarda (bloqueado) até que as 3 threads façam o k_sem_give()
    for (int i = 0; i < NUM_THREADS; i++) {
        k_sem_take(&sync_sem, K_FOREVER);
    }

    t_end = k_cycle_get_32();
    cyc_sw = t_end - t_start;

    // ------------------------------------------------------------------
    // MULTIPLICAÇÃO HARDWARE (CFS)
    // ------------------------------------------------------------------
    printk("Multiplicando no Acelerador CFS (10 Vias)...\n");
    t_start = k_cycle_get_32();
    matrix_mult_hw();
    t_end = k_cycle_get_32();
    cyc_hw = t_end - t_start;

    // ------------------------------------------------------------------
    // RESULTADOS
    // ------------------------------------------------------------------
    printk("\n====================================================\n");
    printk("              RESULTADOS DE DESEMPENHO              \n");
    printk("====================================================\n");
    printk(" Ciclos em Software (OS Threads): %u\n", cyc_sw);
    printk(" Ciclos em Hardware (CFS):        %u\n", cyc_hw);

    if (cyc_hw > 0) {
        uint32_t speed_int = cyc_sw / cyc_hw;
        uint32_t speed_frac = ((cyc_sw % cyc_hw) * 100) / cyc_hw;
        printk(" Speedup HW sobre OS: %u.%u vezes\n", speed_int, speed_frac);
    }
    
    printk("----------------------------------------------------\n");
    printk(" Verificacao (A * A^-1 = I): ");
    if (verify_identity()) {
        printk("[SUCESSO]\n");
    } else {
        printk("[FALHA]\n");
    }
    printk("====================================================\n\n");

    return 0;
}
