#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/bootrom.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"

#include "ff.h"
#include "diskio.h"
#include "f_util.h"
#include "hw_config.h"
#include "my_debug.h"
#include "sd_card.h"
#include "ssd1306.h"

// Definição de intervalos
#define LED_BLINK_MS 200
#define BUZZER_BEEP_MS 100

// Definição de parâmetros PWM para Buzzer
#define WRAP 1000
#define DIV_CLK 250

// Slice PWM para buzzer
static uint buzzer_slice;

// Definição dos pinos I2C para o display OLED
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define ENDERECO_DISP 0x3C    // Endereço I2C do display

// Variável para display
ssd1306_t ssd;

// Definição dos pinos I2C para o MPU6050
#define I2C_PORT i2c0                 // I2C0 usa pinos 0 e 1
#define I2C_SDA 0
#define I2C_SCL 1

// Endereço padrão do MPU6050
static int addr = 0x68;

// Pinos
const uint8_t btn_A_pin = 5;
const uint8_t btn_B_pin = 6;
const uint8_t btn_joy_pin = 22;
const uint8_t led_green_pin = 11;
const uint8_t led_blue_pin = 12;
const uint8_t led_red_pin = 13;
const uint8_t buzzer_pin = 21;

// Estados do sistema
typedef enum Sistema
{
    INICIALIZACAO,
    READY,
    CAPTURA,
    LEITURA_SD,
    ERROR,
    SD_NOT_FOUND,
    EXIT
} Sistema;
static Sistema estado_atual = INICIALIZACAO;
static Sistema estado_anterior = INICIALIZACAO;

// Timers periódicos para LED e Buzzer
struct repeating_timer led_timer;
static volatile bool led_on;
static Sistema led_blink_state;
struct repeating_timer buzzer_timer;
static volatile bool buzzer_on;
static uint8_t buzzer_num_beeps;

// Parâmetros para gravação de dados
static volatile uint curr_amostras = 0;
static const uint32_t intervalo_log = 250;
static absolute_time_t last_log_time;

// Flags acionadas pelos botões
static volatile bool gravacao_req = false;
static volatile bool leitura_req = false;
static volatile bool exit_req = false;

static volatile bool mudanca_display = false;

// Definições iniciais do arquivo CSV
static FIL file;
static char filename[20] = "mpu_data.csv";
const char *cabecalho = "time_ms,accel_x,accel_y,accel_z,giro_x,giro_y,giro_z\n";

// ------------------------------------ Protótipos ---------------------------------------

// Inicialização de LEDs e botões
static void init_leds();
static void init_buttons();
static void init_buzzer_pwm();

// Inicialização e leitura do sensor MPU6050
static void mpu6050_reset();
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]);
static void mpu6050_read_process(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);

// Leitura e escrita no cartão SD
static sd_card_t *sd_get_by_name(const char *const name);
static FATFS *sd_get_fs_by_name(const char *name);
static uint8_t run_mount();
static uint8_t run_unmount();
static void capture_mpu_data_and_save();
static void read_file(const char *filename);

// Processamento de eventos e atualização de estados dos periféricos
void gpio_irq_handler(uint gpio, uint32_t events);
static void processar_botoes();
static void set_led_state();
static char *get_state_name(Sistema estado);
static void display_upd();
static void handle_error(Sistema tipo, uint32_t duracao);

// Temporizadores
bool led_blink_callback(struct repeating_timer *t);
bool buzzer_beep_callback(struct repeating_timer *t);

// --------------------------------------------------------------------------------------

int main()
{
    stdio_init_all();

    // Inicialização dos LEDs e I2C do Display OLED
    init_leds();

    i2c_init(I2C_PORT_DISP, 400 * 1000);
    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);

    ssd1306_init(&ssd, WIDTH, HEIGHT, false, ENDERECO_DISP, I2C_PORT_DISP);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    // Exibe os estados de inicialização do Display e LED RGB
    set_led_state();
    display_upd();
    sleep_ms(5000);

    init_buzzer_pwm();

    init_buttons();

    // Inicialização da I2C do MPU6050
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Declara os pinos como I2C na Binary Info
    bi_decl(bi_2pins_with_func(I2C_SDA, I2C_SCL, GPIO_FUNC_I2C));
    mpu6050_reset();

    // Monta o cartão MicroSD
    uint8_t falha = run_mount();
    if (falha)
    {   
        printf("Cartão SD não encontrado. Por favor, tente novamente!\n");
        handle_error(SD_NOT_FOUND, 1000);
        reset_usb_boot(0, 0);
    }

    estado_atual = READY;
    while (true)
    {   
        if (estado_atual != estado_anterior)
        {   
            estado_anterior = estado_atual;
            set_led_state();
            display_upd();
        }

        processar_botoes();

        if (estado_atual == CAPTURA)
        {   
            absolute_time_t now = get_absolute_time();
            if (absolute_time_diff_us(last_log_time, now) > intervalo_log * 1000)
            {   
                last_log_time = now;
                capture_mpu_data_and_save();
                display_upd();
            }
        }
        else if (estado_atual == EXIT)
        {
            reset_usb_boot(0, 0);
        }

        sleep_ms(20);
    }
    return 0;
}

static void init_leds()
{
    gpio_init(led_red_pin);
    gpio_set_dir(led_red_pin, GPIO_OUT);
    gpio_put(led_red_pin, 1); // Para inicializar com cor AMARELA

    gpio_init(led_green_pin);
    gpio_set_dir(led_green_pin, GPIO_OUT);
    gpio_put(led_green_pin, 1); // Para inicializar com cor AMARELA

    gpio_init(led_blue_pin);
    gpio_set_dir(led_blue_pin, GPIO_OUT);
    gpio_put(led_blue_pin, 0);
}

static void init_buttons()
{
    gpio_init(btn_A_pin);
    gpio_set_dir(btn_A_pin, GPIO_IN);
    gpio_pull_up(btn_A_pin);
    gpio_set_irq_enabled_with_callback(btn_A_pin, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    gpio_init(btn_B_pin);
    gpio_set_dir(btn_B_pin, GPIO_IN);
    gpio_pull_up(btn_B_pin);
    gpio_set_irq_enabled_with_callback(btn_B_pin, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    gpio_init(btn_joy_pin);
    gpio_set_dir(btn_joy_pin, GPIO_IN);
    gpio_pull_up(btn_joy_pin);
    gpio_set_irq_enabled_with_callback(btn_joy_pin, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
}

static void init_buzzer_pwm()
{
    gpio_set_function(buzzer_pin, GPIO_FUNC_PWM);
    buzzer_slice = pwm_gpio_to_slice_num(buzzer_pin);
    pwm_set_wrap(buzzer_slice, WRAP);
    pwm_set_clkdiv(buzzer_slice, DIV_CLK);
    pwm_set_gpio_level(buzzer_pin, 0);
    pwm_set_enabled(buzzer_slice, true);
}

// Função para resetar e inicializar o MPU6050
static void mpu6050_reset()
{
    // Dois bytes para reset: primeiro o registrador, segundo o dado
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
    sleep_ms(100); // Aguarda reset e estabilização

    // Sai do modo sleep (registrador 0x6B, valor 0x00)
    buf[1] = 0x00;
    i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
    sleep_ms(10); // Aguarda estabilização após acordar
}

// Função para ler dados crus do acelerômetro, giroscópio e temperatura
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3])
{
    uint8_t buffer[6];

    // Lê aceleração a partir do registrador 0x3B (6 bytes)
    uint8_t val = 0x3B;
    i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++)
    {
        accel[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    // Lê giroscópio a partir do registrador 0x43 (6 bytes)
    val = 0x43;
    i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++)
    {
        gyro[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }
}

static void mpu6050_read_process(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{   
    // Lê valor bruto da aceleração e giroscópio do MPU6050
    int16_t raw_accel[3];
    int16_t raw_gyro[3];
    mpu6050_read_raw(raw_accel, raw_gyro);

    // Processa dados da aceleração
    const float sensibilidade_accel = 16384.0f;
    *ax = raw_accel[0] / sensibilidade_accel;
    *ay = raw_accel[1] / sensibilidade_accel;
    *az = raw_accel[2] / sensibilidade_accel;

    // Processa dados da aceleração
    const float sensibilidade_gyro = 131.0f;
    *gx = raw_gyro[0] / sensibilidade_gyro;
    *gy = raw_gyro[1] / sensibilidade_gyro;
    *gz = raw_gyro[2] / sensibilidade_gyro;

}

static sd_card_t *sd_get_by_name(const char *const name)
{
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return sd_get_by_num(i);
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}

static FATFS *sd_get_fs_by_name(const char *name)
{
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return &sd_get_by_num(i)->fatfs;
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}

static uint8_t run_mount()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return 1;
    }
    FRESULT fr = f_mount(p_fs, arg1, 1);
    if (FR_OK != fr)
    {
        printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        return 1;
    }
    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = true;
    printf("Processo de montagem do SD ( %s ) concluído\n", pSD->pcName);
    return 0;
}

static uint8_t run_unmount()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return 1;
    }
    FRESULT fr = f_unmount(arg1);
    if (FR_OK != fr)
    {
        printf("f_unmount error: %s (%d)\n", FRESULT_str(fr), fr);
        return 1;
    }
    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = false;
    pSD->m_Status |= STA_NOINIT; // in case medium is removed
    printf("SD ( %s ) desmontado\n", pSD->pcName);
    return 0;
}

// Função para capturar dados do ADC e salvar no arquivo *.txt
void capture_mpu_data_and_save()
{   
    // Lê valores do sensor e atualiza número de amostras
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    mpu6050_read_process(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
    curr_amostras++;

    // Escreve no arquivo seguindo a formatação CSV
    char buffer[100];
    sprintf(buffer, "%i,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", curr_amostras * intervalo_log, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
    UINT bw;
    FRESULT res = f_write(&file, buffer, strlen(buffer), &bw);
    if (res != FR_OK)
    {
        printf("[ERRO] Não foi possível escrever no arquivo. Monte o Cartao.\n");
        f_close(&file);
        handle_error(ERROR, 1000);
        estado_atual = READY; // Parar gravação após erro
        return;
    }

    // Faz um FLASH AZUL rápido para indicar amostragem
    gpio_put(led_red_pin, 0);
    gpio_put(led_blue_pin, 1);
    sleep_ms(20);
    gpio_put(led_blue_pin, 0);
    gpio_put(led_red_pin, 1); // Reacende VERMELHO referente ao estado de CAPTURA
}

// Função para ler o conteúdo de um arquivo e exibir no terminal
void read_file(const char *filename)
{
    FRESULT res = f_open(&file, filename, FA_READ);
    if (res != FR_OK)
    {
        printf("[ERRO] Não foi possível abrir o arquivo para leitura. Verifique se o Cartão está montado ou se o arquivo existe.\n");
        handle_error(ERROR, 1000);
        return;
    }

    char buffer[128];
    UINT br;
    printf("Conteúdo do arquivo %s:\n", filename);
    while (f_read(&file, buffer, sizeof(buffer) - 1, &br) == FR_OK && br > 0)
    {
        buffer[br] = '\0';
        printf("%s", buffer);
    }
    f_close(&file);
    printf("\nLeitura do arquivo %s concluída.\n\n", filename);
}

void gpio_irq_handler(uint gpio, uint32_t events)
{
    static absolute_time_t last_time_A;
    static absolute_time_t last_time_B;
    static absolute_time_t last_time_joy;

    absolute_time_t now = get_absolute_time();
    if (gpio == btn_A_pin)
    {
        if (absolute_time_diff_us(last_time_A, now) > 200000)
        {   
            last_time_A = now;
            gravacao_req = true;
        }
    }
    else if (gpio == btn_B_pin)
    {
        if (absolute_time_diff_us(last_time_B, now) > 200000)
        {   
            last_time_B = now;
            exit_req = true;
        }
    }
    else if (gpio == btn_joy_pin)
    {
        if (absolute_time_diff_us(last_time_joy, now) > 200000)
        {   
            last_time_joy = now;
            leitura_req = true;
        }
    }
}

static void processar_botoes()
{
    if (gravacao_req)
    {   
        gravacao_req = false;
        if (estado_atual == READY)
        {
            FRESULT res = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
            if (res != FR_OK)
            {   
                printf("[ERRO] Não foi possível criar/abrir arquivo para iniciar a gravação\n");
                handle_error(ERROR, 1000);
                return;
            }

            UINT bw;
            res = f_write(&file, cabecalho, strlen(cabecalho), &bw);
            if (res != FR_OK)
            {   
                printf("[ERRO] Não foi possível escrever o cabeçalho no arquivo para iniciar a gravação\n");
                f_close(&file);
                handle_error(ERROR, 1000);
                return;
            }

            curr_amostras = 0;
            buzzer_num_beeps = 1;
            buzzer_on = true;
            buzzer_beep_callback(NULL); // Garante primeiro beep imediato
            add_repeating_timer_ms(BUZZER_BEEP_MS, buzzer_beep_callback, NULL, &buzzer_timer);
            estado_atual = CAPTURA;
        }
        else if (estado_atual == CAPTURA)
        {   
            buzzer_num_beeps = 2;
            buzzer_on = true;
            buzzer_beep_callback(NULL); // Garante primeiro beep imediato
            add_repeating_timer_ms(BUZZER_BEEP_MS, buzzer_beep_callback, NULL, &buzzer_timer);
            f_close(&file);
            estado_atual = READY;
        }
    }

    if (leitura_req)
    {   
        leitura_req = false;
        if (estado_atual == READY)
        {   
            // Atualiza periféricos e estado momentaneamente para indicar leitura
            estado_atual = LEITURA_SD;
            set_led_state();
            display_upd();

            read_file(filename);

            // Retorna estado do programa ao anterior e atualiza buffer
            estado_atual = READY;
            estado_anterior = LEITURA_SD;
        }
    }

    if (exit_req)
    {
        exit_req = false;
        if (estado_atual == READY)
        {   
            uint8_t falha = run_unmount();
            if (!falha)
            {
                estado_atual = EXIT;
                return;
            }
            printf("[ERRO] Falha no desmonte do cartão SD. Tente novamente\n");
            handle_error(ERROR, 1000);
        }
    }
}

static void set_led_state()
{   
    // Desliga quaisquer LEDs/Rotinas ativas
    gpio_put(led_red_pin, 0);
    gpio_put(led_blue_pin, 0);
    gpio_put(led_green_pin, 0);
    cancel_repeating_timer(&led_timer);

    if (estado_atual == INICIALIZACAO) // Cor AMARELA estática
    {   
        gpio_put(led_red_pin, 1);
        gpio_put(led_green_pin, 1);
    }
    else if (estado_atual == READY) // Cor VERDE estática
    {
        gpio_put(led_green_pin, 1);
    }
    else if (estado_atual == CAPTURA) // Cor VERMELHA estática
    {
        gpio_put(led_red_pin, 1);
    }
    else if (estado_atual == LEITURA_SD || estado_atual == ERROR || estado_atual == SD_NOT_FOUND) // Cor piscando de acordo ao estado
    {
        led_on = true;
        led_blink_state = estado_atual;

        // Garante uma primeira chamada imediata para que inicie ligado
        led_blink_callback(NULL);

        add_repeating_timer_ms(LED_BLINK_MS, led_blink_callback, NULL, &led_timer);
    }
    else // Nenhum LED aceso para a saída do programa (EXIT)
    {
        return;
    }
}

static char *get_state_name(Sistema estado)
{
    switch (estado)
    {
        case INICIALIZACAO:
            return "INICIALIZANDO";
        case READY:
            return "READY";
        case CAPTURA:
            return "GRAVACAO";
        case SD_NOT_FOUND:
            return "SD_NOT_FOUND";
        case LEITURA_SD:
            return "LEITURA";
        case EXIT:
            return "SAINDO";
    }
}

static void display_upd()
{
    // Limpa o display
    ssd1306_fill(&ssd, false);

    if (estado_atual != ERROR)
    {   
        // Define estado do cartão SD em string
        char *str_sd_state;
        if (estado_atual == SD_NOT_FOUND)
        {
            str_sd_state = "SD: ERRO!";
        }
        else if (estado_atual == INICIALIZACAO)
        {
            str_sd_state = "SD: LOADING";
        }
        else
        {
            str_sd_state = "SD: OK";
        }

        // Define estado atual do programa em string
        char *str_estado = get_state_name(estado_atual);

        // Converte número de amostras em string
        char str_buffer[7];
        sprintf(str_buffer, "%i", curr_amostras);

        // Exibição no display
        ssd1306_fill(&ssd, false);                            // Limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 60, true, false);        // Desenha um retângulo
        ssd1306_line(&ssd, 3, 16, 123, 16, true);             // Desenha uma linha horizontal
        ssd1306_line(&ssd, 3, 37, 123, 37, true);             // Desenha outra linha horizontal
        ssd1306_draw_string(&ssd, "Datalogger MPU", 8, 6);   // Escreve titulo no display
        ssd1306_draw_string(&ssd, str_sd_state, 8, 18);       // Escreve status do SD no display
        ssd1306_draw_string(&ssd, str_estado, 12, 28);  // Escreve status do programa no display
        ssd1306_draw_string(&ssd, "AMOSTRAS", 30, 41);           // Escreve AMOSTRAS no display
        ssd1306_draw_string(&ssd, str_buffer, 42, 52);         // Exibe número de amostras no display
    }
    else
    {
        ssd1306_draw_string(&ssd, "ERRO!", 48, 24);
    }

    ssd1306_send_data(&ssd);
}

static void handle_error(Sistema tipo, uint32_t duracao)
{   
    // Atualiza estado e periféricos na duração especificada
    estado_atual = tipo;
    set_led_state();
    display_upd();
    sleep_ms(duracao);

    estado_atual = estado_anterior;
    estado_anterior = tipo;
}

bool led_blink_callback(struct repeating_timer *t)
{   
    if (led_blink_state == ERROR) // ROXO piscando para erro
    {
        gpio_put(led_red_pin, led_on);
        gpio_put(led_blue_pin, led_on);
    }
    else // AZUL piscando para Gravação/Leitura no SD
    {
        gpio_put(led_blue_pin, led_on);
    }
    led_on = !led_on;

    return true;
}

bool buzzer_beep_callback(struct repeating_timer *t)
{   
    if (buzzer_on)
    {
        pwm_set_gpio_level(buzzer_pin, WRAP / 2);
    }
    else
    {
        pwm_set_gpio_level(buzzer_pin, 0);
        buzzer_num_beeps--; // Desligamento indica final de um beep
    }
    buzzer_on = !buzzer_on;

    if (buzzer_num_beeps == 0)
    {
        return false;
    }
    return true;
}