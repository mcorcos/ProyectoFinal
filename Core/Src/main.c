/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ---------- MOVIMIENTO / REBOTE ----------
#define BOUNCE_DWELL_MS      120   // pausa breve al rebotar para evitar golpeteos mecánicos
#define LIMIT_CLEAR_MM       0.5f  // margen para rearmar la detección de límite (evita contar doble)

// ---------- EXTERN TIM HANDLES ----------
extern TIM_HandleTypeDef htim2;  // STEP (DM542) - TIM2 CH1
extern TIM_HandleTypeDef htim1;  // PWM H-Bridge    - TIM1 CH1/CH1N

// ---------- PINS ----------
#define DM542_DIR_Port      GPIOA
#define DM542_DIR_Pin       GPIO_PIN_1
#define DM542_EN_Port       GPIOA
#define DM542_EN_Pin        GPIO_PIN_4

#define BUTTON_Port         GPIOC     // Nucleo B1
#define BUTTON_Pin          GPIO_PIN_13

// Velocidad de homing (Hz de STEP)
#define FSTEP_HOME_HZ      200.0f   // ajustable: más lento = más suave para buscar HOME
#define FSTEP_RUN_HZ	250.0f
// ---------- CLOCK CACHE ----------
static uint32_t TIM2_Clk_Hz = 0;
static uint32_t TIM1_Clk_Hz = 0;


// ---------- CONFIGURACIÓN ENTRABLE POR CONSOLA (DUMMY por ahora) ----------
typedef struct {
  // STEP/DIR (geom)
  uint16_t steps_per_rev;       // pasos enteros por vuelta del motor p.ej. 200
  uint16_t microstepping;       // subdivisión por driver (x8, x16, etc.). Eleva la resolución y suaviza movimiento
  float    leadscrew_pitch_mm;  // avance lineal por vuelta del husillo (mm/vuelta), p.ej. 2.0
  float    traverse_width_mm;   // ancho total a cubrir del carrete. Útil para lógica de “rebotar” en extremos (auto-bounce).
  float    wire_diameter_mm;    // es tu “paso lateral” deseado por cada vuelta del carrete (si el carrete pone 1 capa por vuelta, avanzás ~el diámetro del hilo).
  // Carrete principal
  float    spool_diameter_mm;   // diámetro efectivo del carrete
  float    dc_target_rpm;       // “velocidad” deseada del carrete (dummy)
  float    hbridge_duty_0_to_1; // duty del PWM del H-bridge (dummy directo)
  // Objetivo de bobinado
  float    target_turns_total;  // vueltas objetivo de la bobina (0 = infinito / solo rebote)
  // Límites
  float    fstep_min_hz;
  float    fstep_max_hz;
  uint32_t traverse_passes_target; // 0 = infinito (rebota sin parar). Cuenta 1 cada vez que toca un extremo.
} BobbinConfig;




// Estado runtime
static volatile uint8_t g_step_dir = 0;     // 0/1
static float            g_step_freq_hz = 0; // monitoreo
#if defined(__GNUC__)
#define UNUSED_FUNC __attribute__((unused))
#else
#define UNUSED_FUNC
#endif




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Posición y límites en pasos (calculados desde cfg)

// Config por defecto para probar
static BobbinConfig cfg = {
    .steps_per_rev       = 400,
    .microstepping       = 1,
    // Medido en el banco: 1 vuelta del motor desplaza ~80 mm (correa HTD5 16T)
    .leadscrew_pitch_mm  = 8.0f,
    .traverse_width_mm   = 50.0f,
    .wire_diameter_mm    = 0.06f,
    .spool_diameter_mm   = 32.0f,
    .dc_target_rpm       = 37.25f,
    .hbridge_duty_0_to_1 = 0.6f,
    .target_turns_total  = 0.0f,  // 0 = no limitar por vueltas (usa solo rebote)
    .fstep_min_hz        = 50.0f,
    .fstep_max_hz        = 5000.0f,
    .traverse_passes_target = 0, // 0 = sin límite, rebota indefinidamente
};


static volatile int32_t g_pos_steps = 0;
static int32_t g_min_steps = 0;
static int32_t g_max_steps = 0;

static uint8_t s_backoff_done = 0;   // 0 = falta hacer el medio cm, 1 = ya hecho
static uint32_t g_traverse_passes_done = 0; // cuántas veces tocamos un extremo
static uint8_t  g_run_dir = 1;              // 1 = alejándose de HOME, 0 = yendo a HOME
static int32_t  g_limit_clear_steps = 0;    // margen en pasos para rearmar detección de límite
static uint8_t  g_homed_done = 0;           // 0 = aún no homingueó en esta sesión
static uint8_t  g_run_requested = 0;        // 1 = usuario pidió ciclo completo
static uint8_t  g_offset60_done = 0;        // 0 = aún no avanzó 60mm tras HOME


// ---------- ESTADOS DE LA MÁQUINA ----------
typedef enum {
  STATE_HOMING = 0,    // al encender vamos a HOME
  STATE_IDLE,          // quieto, esperando orden
  STATE_RUNNING        // después lo llenamos con el programa real
} MachineState_t;

static volatile MachineState_t g_state = STATE_HOMING;
static void SetState(MachineState_t new_state, const char *reason);

typedef enum {
  LIMIT_NONE = 0,
  LIMIT_MIN,
  LIMIT_MAX
} LimitHit_t;

static LimitHit_t g_last_limit_hit = LIMIT_NONE;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

static uint32_t Timer_Get_Input_Clock(TIM_HandleTypeDef *htim);
static void     Steppers_Enable(bool en_low_active);
static void     Steppers_SetDir(uint8_t dir);
static HAL_StatusTypeDef Steppers_SetStepFreq_Hz(float f_hz);

static HAL_StatusTypeDef HBridge_SetPWM_Duty(float duty_0_to_1);
static HAL_StatusTypeDef HBridge_Start(void);
static void               HBridge_Stop(void);
static inline float steps_per_mm(const BobbinConfig *c) ;
static void DC_SetSpeed_0to1(float duty);
static void DC_StartForward(float duty);
static void DC_StartReverse(float duty);
static void DC_Brake(void);
static void DC_Coast(void);
static void Console_PrintInfo(void);

// Helpers demo
static float    Compute_Stepper_Freq_From_DC(const BobbinConfig *c);
static uint8_t  Button_Read(void);
static inline uint8_t HomeSensor_IsActive(void);
static void     Config_RecomputeDerived(void);
static void     Console_PrintBanner(void);
static void     PrintScaled(const char *label, float v, uint8_t decimals);

// Consola UART simple
static void     Console_Poll(void);
static void     Console_ProcessLine(char *line);
static void     Console_PrintHelp(void);
static void     Console_PrintConfig(void);
static void     Console_PrintStatus(void);

static void Machine_Task(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


static void debug_printf(const char *fmt, ...)
{
    char buf[192]; // un poco más grande para logs con varios campos
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 20);
}

static void Console_PrintBanner(void)
{
  debug_printf("\r\n===============================\r\n");
  debug_printf(" Bobinador v1 (UART @115200)\r\n");
  debug_printf(" Comandos: HELP, CFG?, STAT?,\r\n");
  debug_printf("   WIRE x, TURNS x, RPM x, DUTY x,\r\n");
  debug_printf("   RUN, STOP, HOME\r\n");
  debug_printf(" Boton B1 o RUN: HOMING -> backoff -> bobinado\r\n");
  debug_printf("===============================\r\n\r\n");
}

static void PrintScaled(const char *label, float v, uint8_t decimals)
{
  static const int32_t pow10[] = {1, 10, 100, 1000, 10000};
  if (decimals > 4) decimals = 4;
  int32_t scale = pow10[decimals];
  int32_t scaled = (int32_t)lroundf(v * (float)scale);
  int32_t abs_scaled = (scaled < 0) ? -scaled : scaled;
  debug_printf("%s%ld.%0*d\r\n", label, scaled / scale, decimals, abs_scaled % scale);
}

static const char* StateName(MachineState_t s)
{
  switch (s) {
    case STATE_HOMING:  return "HOMING";
    case STATE_IDLE:    return "IDLE";
    case STATE_RUNNING: return "RUNNING";
    default:            return "?";
  }
}
static void SetState(MachineState_t new_state, const char *reason)
{
  MachineState_t old = g_state;
  if (old == new_state) return;
  g_state = new_state;
  if (reason) {
    debug_printf("[STATE] %s -> %s (%s)\r\n", StateName(old), StateName(new_state), reason);
  } else {
    debug_printf("[STATE] %s -> %s\r\n", StateName(old), StateName(new_state));
  }
}


// ======================== CONSOLA UART BÁSICA ========================

#define CONSOLE_RX_BUF_LEN  64
#define CONSOLE_ECHO        1
static char   s_console_buf[CONSOLE_RX_BUF_LEN];
static uint8_t s_console_idx = 0;

static void Console_Poll(void)
{
  uint8_t ch;
  if (HAL_UART_Receive(&huart2, &ch, 1, 0) == HAL_OK)
  {
    if (CONSOLE_ECHO)
    {
      if (ch == '\r' || ch == '\n') {
        const char crlf[] = "\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)crlf, 2, 10);
      } else {
        HAL_UART_Transmit(&huart2, &ch, 1, 10);
      }
    }

    if (ch == '\r' || ch == '\n')
    {
      if (s_console_idx > 0)
      {
        s_console_buf[s_console_idx] = '\0';
        Console_ProcessLine(s_console_buf);
        s_console_idx = 0;
      }
    }
    else
    {
      if (s_console_idx < (CONSOLE_RX_BUF_LEN - 1))
      {
        s_console_buf[s_console_idx++] = (char)ch;
      }
    }
  }
}

static void Console_StrToUpper(char *s)
{
  while (*s)
  {
    if (*s >= 'a' && *s <= 'z') *s = (char)(*s - 'a' + 'A');
    s++;
  }
}

static int Console_ParseFloat(const char *p, float *out)
{
  while (*p == ' ' || *p == '\t' || *p == '=') p++;
  char *endp = NULL;
  float v = strtof(p, &endp);
  if (endp == p) return 0;
  *out = v;
  return 1;
}

static void Console_ProcessLine(char *line)
{
  // Hacemos una copia para comandos en mayúsculas
  char cmd[CONSOLE_RX_BUF_LEN];
  strncpy(cmd, line, sizeof(cmd) - 1);
  cmd[sizeof(cmd) - 1] = '\0';

  // Quitar espacios iniciales
  char *p = cmd;
  while (*p == ' ' || *p == '\t') p++;

  Console_StrToUpper(p);

  if (strcmp(p, "HELP") == 0)
  {
    Console_PrintHelp();
  }
  else if (strcmp(p, "CFG?") == 0)
  {
    Console_PrintConfig();
  }
  else if (strcmp(p, "INFO?") == 0)
  {
    Console_PrintInfo();
  }
  else if (strcmp(p, "STAT?") == 0)
  {
    Console_PrintStatus();
  }
  else if (strncmp(p, "WIRE", 4) == 0)
  {
    float v;
    if (Console_ParseFloat(p + 4, &v))
    {
      if (v > 0.0f)
      {
        cfg.wire_diameter_mm = v;
        debug_printf("[CFG] wire_diameter_mm=%.3f\r\n", cfg.wire_diameter_mm);
        Config_RecomputeDerived();
      }
    }
  }
  else if (strncmp(p, "TURNS", 5) == 0)
  {
    float v;
    if (Console_ParseFloat(p + 5, &v))
    {
      if (v < 0.0f) v = 0.0f;
      cfg.target_turns_total = v;
      debug_printf("[CFG] target_turns_total=%.1f\r\n", cfg.target_turns_total);
      Config_RecomputeDerived();
    }
  }
  else if (strncmp(p, "RPM", 3) == 0)
  {
    float v;
    if (Console_ParseFloat(p + 3, &v))
    {
      if (v < 0.0f) v = 0.0f;
      cfg.dc_target_rpm = v;
      debug_printf("[CFG] dc_target_rpm=%.1f\r\n", cfg.dc_target_rpm);
    }
  }
  else if (strncmp(p, "DUTY", 4) == 0)
  {
    float v;
    if (Console_ParseFloat(p + 4, &v))
    {
      if (v < 0.0f) v = 0.0f;
      if (v > 1.0f) v = 1.0f;
      cfg.hbridge_duty_0_to_1 = v;
      debug_printf("[CFG] hbridge_duty=%.2f\r\n", cfg.hbridge_duty_0_to_1);
    }
  }
  else if (strcmp(p, "RUN") == 0)
  {
    g_run_requested = 1;
    debug_printf("[CMD] RUN request\r\n");
  }
  else if (strcmp(p, "STOP") == 0)
  {
    Steppers_SetStepFreq_Hz(0.0f);
    DC_Brake();
    g_run_requested = 0;
    g_homed_done = HomeSensor_IsActive() ? 1 : g_homed_done;
    g_offset60_done = 0;
    SetState(STATE_IDLE, "STOP");
    debug_printf("[CMD] STOP -> IDLE\r\n");
  }
  else if (strcmp(p, "HOME") == 0)
  {
    s_backoff_done = 0;
    g_homed_done = 0;
    SetState(STATE_HOMING, "cmd HOME");
    debug_printf("[CMD] HOME -> STATE_HOMING\r\n");
  }
  else
  {
    debug_printf("[CMD] Desconocido. Usa HELP\r\n");
  }
}

static void Console_PrintHelp(void)
{
  debug_printf("\r\nComandos UART:\r\n");
  debug_printf("  HELP          : esta ayuda\r\n");
  debug_printf("  CFG?          : mostrar configuracion actual\r\n");
  debug_printf("  INFO?         : resumen derivado (vueltas capa, pasos/mm, fstep)\r\n");
  debug_printf("  STAT?         : estado y posicion\r\n");
  debug_printf("  WIRE x        : diametro hilo (mm)\r\n");
  debug_printf("  TURNS x       : vueltas objetivo totales (0=infinito)\r\n");
  debug_printf("  RPM x         : rpm objetivo carrete\r\n");
  debug_printf("  DUTY x        : duty DC [0..1]\r\n");
  debug_printf("  RUN           : iniciar bobinado (desde IDLE)\r\n");
  debug_printf("  STOP          : frenar y pasar a IDLE\r\n");
  debug_printf("  HOME          : volver a hacer homing\r\n\r\n");
}

static void Console_PrintConfig(void)
{
  debug_printf("CFG: steps=%u micro=%u pitch=%.3fmm width=%.1fmm\r\n",
               cfg.steps_per_rev, cfg.microstepping,
               cfg.leadscrew_pitch_mm, cfg.traverse_width_mm);
  debug_printf("CFG: wire=%.3fmm target_turns=%.1f\r\n",
               cfg.wire_diameter_mm, cfg.target_turns_total);
  debug_printf("CFG: dc_rpm=%.1f duty=%.2f\r\n",
               cfg.dc_target_rpm, cfg.hbridge_duty_0_to_1);
  debug_printf("CFG: fstep[min,max]=[%.1f, %.1f]Hz passes_target=%lu\r\n",
               cfg.fstep_min_hz, cfg.fstep_max_hz,
               (unsigned long)cfg.traverse_passes_target);
}

static void Console_PrintStatus(void)
{
  debug_printf("STAT: state=%s pos=%ld dir=%u passes=%lu/%lu HOME=%u\r\n",
               StateName(g_state), g_pos_steps, g_run_dir,
               (unsigned long)g_traverse_passes_done,
               (unsigned long)cfg.traverse_passes_target,
               HomeSensor_IsActive());
}

static void Console_PrintInfo(void)
{
  float k_spmm = steps_per_mm(&cfg);
  float turns_per_layer = (cfg.wire_diameter_mm > 0.0f) ? (cfg.traverse_width_mm / cfg.wire_diameter_mm) : 0.0f;
  float fstep = Compute_Stepper_Freq_From_DC(&cfg);
  debug_printf("INFO: steps/mm=%.1f turns/layer=%.1f target_turns=%.1f\r\n",
               k_spmm, turns_per_layer, cfg.target_turns_total);
  debug_printf("INFO: passes_target=%lu (0=inf) fstep_calc=%.1f Hz\r\n",
               (unsigned long)cfg.traverse_passes_target, fstep);
}



static inline float steps_per_mm(const BobbinConfig *c) {
  return ((float)c->steps_per_rev * (float)c->microstepping) / c->leadscrew_pitch_mm;
}

// Recalcula valores derivados de cfg (pasadas objetivo y márgenes de límite)
static void Config_RecomputeDerived(void)
{
  float k_steps_per_mm = steps_per_mm(&cfg);
  g_limit_clear_steps = (int32_t)lroundf(LIMIT_CLEAR_MM * k_steps_per_mm);
  if (g_limit_clear_steps < 1) g_limit_clear_steps = 1;

  if (cfg.target_turns_total > 0.0f &&
      cfg.traverse_width_mm > 0.0f &&
      cfg.wire_diameter_mm > 0.0f)
  {
    float turns_per_layer = cfg.traverse_width_mm / cfg.wire_diameter_mm; // vueltas por capa
    float layers = cfg.target_turns_total / turns_per_layer;             // capas necesarias
    float passes_f = layers * 2.0f;                                      // 2 extremos por capa
    if (passes_f < 1.0f) passes_f = 1.0f;
    cfg.traverse_passes_target = (uint32_t)lroundf(passes_f);
  }
  else
  {
    cfg.traverse_passes_target = 0; // rebote infinito si no hay objetivo de vueltas
  }
}


static uint32_t Timer_Get_Input_Clock(TIM_HandleTypeDef *htim)
{
  RCC_ClkInitTypeDef clk;
  uint32_t flash;
  HAL_RCC_GetClockConfig(&clk, &flash);

  uint32_t pclk, mul = 1;
  if (htim->Instance == TIM1) {
    pclk = HAL_RCC_GetPCLK2Freq();
    if (clk.APB2CLKDivider != RCC_HCLK_DIV1) mul = 2;
  } else {
    pclk = HAL_RCC_GetPCLK1Freq();
    if (clk.APB1CLKDivider != RCC_HCLK_DIV1) mul = 2;
  }
  return pclk * mul;
}

/*********** DM542 (STEP/DIR) ***********/
static void Steppers_Enable(bool en_low_active)
{
  if (en_low_active) HAL_GPIO_WritePin(DM542_EN_Port, DM542_EN_Pin, GPIO_PIN_RESET);
  else               HAL_GPIO_WritePin(DM542_EN_Port, DM542_EN_Pin, GPIO_PIN_SET);
}

static void Steppers_SetDir(uint8_t dir)
{
  g_step_dir = (dir ? 1 : 0);
  HAL_GPIO_WritePin(DM542_DIR_Port, DM542_DIR_Pin, g_step_dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// PWM STEP a frecuencia deseada, garantizando pulso >= ~3us
static HAL_StatusTypeDef Steppers_SetStepFreq_Hz(float f_hz)
{
  if (f_hz <= 0.0f) {
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    g_step_freq_hz = 0.0f;
    return HAL_OK;
  }

  if (f_hz < cfg.fstep_min_hz) f_hz = cfg.fstep_min_hz;
  if (f_hz > cfg.fstep_max_hz) f_hz = cfg.fstep_max_hz;

  const uint32_t tim_clk = TIM2_Clk_Hz;
  uint32_t best_psc = 0, best_arr = 0;
  float err_min = 1e9f;

  for (uint32_t psc = 0; psc <= 0xFFFF; psc++) {
    float arr_f = (tim_clk / ((psc + 1.0f) * f_hz)) - 1.0f;
    if (arr_f < 1.0f || arr_f > 65534.0f) continue;
    uint32_t arr = (uint32_t)lroundf(arr_f);
    float f_real = tim_clk / ((psc + 1.0f) * (arr + 1.0f));
    float err = fabsf((f_real - f_hz) / f_hz);
    if (err < err_min) { err_min = err; best_psc = psc; best_arr = arr; if (err < 5e-4f) break; }
  }
  if (err_min >= 1e8f) return HAL_ERROR;

  uint32_t ccr = (best_arr + 1) / 2; // 50%
  // Asegurar ancho de pulso >= 3us
  float t_high = ((ccr) * (best_psc + 1.0f)) / (float)tim_clk;
  if (t_high < 3e-6f) {
    uint32_t min_ccr = (uint32_t)ceilf(3e-6f * tim_clk / (best_psc + 1.0f));
    if (min_ccr > best_arr) min_ccr = best_arr;
    ccr = min_ccr;
  }

  __HAL_TIM_DISABLE(&htim2);
  htim2.Instance->PSC  = best_psc;
  htim2.Instance->ARR  = best_arr;
  htim2.Instance->CCR1 = ccr;
  __HAL_TIM_ENABLE(&htim2);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  g_step_freq_hz = tim_clk / ((best_psc + 1.0f) * (best_arr + 1.0f));
  return HAL_OK;
}





/*********** Lógica de demo / placeholders ***********/



typedef enum { HBR_COAST = 0, HBR_BRAKE, HBR_FWD, HBR_REV } HBrMode;

static inline void HBridge_SetMode(HBrMode m)
{
  switch (m) {
    case HBR_FWD:
      HAL_GPIO_WritePin(L298_IN1_GPIO_Port, L298_IN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(L298_IN2_GPIO_Port, L298_IN2_Pin, GPIO_PIN_RESET);
      break;
    case HBR_REV:
      HAL_GPIO_WritePin(L298_IN1_GPIO_Port, L298_IN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(L298_IN2_GPIO_Port, L298_IN2_Pin, GPIO_PIN_SET);
      break;
    case HBR_BRAKE:
      HAL_GPIO_WritePin(L298_IN1_GPIO_Port, L298_IN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(L298_IN2_GPIO_Port, L298_IN2_Pin, GPIO_PIN_SET);
      break;
    default: // HBR_COAST
      HAL_GPIO_WritePin(L298_IN1_GPIO_Port, L298_IN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(L298_IN2_GPIO_Port, L298_IN2_Pin, GPIO_PIN_RESET);
      break;
  }
}

// ================== CONTROL DE MOTOR DC (CARRETE) ==================



/*********** H-Bridge PWM: código usa TIM2 CH3 (PA9); el pin L298_ENA en main.h está en PA8/TIM1 ***********/
static HAL_StatusTypeDef HBridge_SetPWM_Duty(float duty_0_to_1)
{
  if (duty_0_to_1 < 0.0f) duty_0_to_1 = 0.0f;
  if (duty_0_to_1 > 1.0f) duty_0_to_1 = 1.0f;

  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
  uint32_t ccr = (uint32_t)lroundf(duty_0_to_1 * (float)arr);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ccr);
  return HAL_OK;
}

static HAL_StatusTypeDef HBridge_Start(void)
{
  return HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}

static void HBridge_Stop(void)
{
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
}


static void DC_SetSpeed_0to1(float duty)
{
  // Limitar duty a [0, 1]
  if (duty < 0.0f) duty = 0.0f;
  if (duty > 1.0f) duty = 1.0f;

  HBridge_SetPWM_Duty(duty);
}

static void DC_StartForward(float duty)
{
  DC_SetSpeed_0to1(duty);     // fija el duty
  HBridge_SetMode(HBR_FWD);   // IN1=1, IN2=0
  HBridge_Start();            // arranca el PWM en TIM2_CH3
}

static void DC_StartReverse(float duty)
{
  DC_SetSpeed_0to1(duty);
  HBridge_SetMode(HBR_REV);   // IN1=0, IN2=1
  HBridge_Start();
}

static void DC_Brake(void)
{
  HBridge_SetMode(HBR_BRAKE); // IN1=1, IN2=1
  DC_SetSpeed_0to1(0.0f);     // sin PWM
}

static void DC_Coast(void)
{
  HBridge_SetMode(HBR_COAST); // IN1=0, IN2=0
  DC_SetSpeed_0to1(0.0f);
  HBridge_Stop();             // opcional, apaga PWM
}


// Sensor inductivo: activo en 0 (salida a 0 cuando está en HOME)

static inline uint8_t HomeSensor_IsActive(void)
{
  return (HAL_GPIO_ReadPin(HOME_GPIO_Port, HOME_Pin) == GPIO_PIN_RESET) ? 1 : 0;
}


// Calcula la frecuencia de STEP (Hz) a partir de la config de bobinado
// Idea: por cada vuelta del carrete (RPM), el carro avanza aprox 1 diámetro de hilo.
static float Compute_Stepper_Freq_From_DC(const BobbinConfig *c)
{
  // Vueltas del carrete por segundo
  float rev_per_s = c->dc_target_rpm / 60.0f;

  // Velocidad lineal del carro para que avance 1 diámetro de hilo por vuelta del carrete
  float traverse_mm_per_s = c->wire_diameter_mm * rev_per_s;

  // Pasos por mm del eje del carro
  float k_steps_mm = steps_per_mm(c);

  // Frecuencia de pasos necesaria
  float f = traverse_mm_per_s * k_steps_mm;

  // Clampeo a los límites configurados
  if (f < c->fstep_min_hz) f = c->fstep_min_hz;
  if (f > c->fstep_max_hz) f = c->fstep_max_hz;

  return f;
}


// Lee el botón de START (Nucleo B1 en PC13, activo en 0 lógico)
static uint8_t Button_Read(void)
{
  GPIO_PinState st = HAL_GPIO_ReadPin(BUTTON_Port, BUTTON_Pin);
  // En las Nucleo el botón está a 1 sin apretar y va a 0 al apretar
  return (st == GPIO_PIN_RESET) ? 1 : 0;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */



  // Guardar clocks de los timers (útil para el cálculo de frecuencia del STEP)

  TIM2_Clk_Hz = Timer_Get_Input_Clock(&htim2);
  TIM1_Clk_Hz = Timer_Get_Input_Clock(&htim1);

  // DM542: habilitar driver y fijar sentido hacia HOME
  Steppers_Enable(true);          // EN activo en bajo (ajustar si tu driver es al revés)
  Steppers_SetDir(0);             // 0 ó 1 → elegí el que vaya "hacia el sensor HOME"
  Steppers_SetStepFreq_Hz(0.0f);  // arrancamos parado, Machine_Task lo pone en marcha


  // Motor DC del carrete: arrancar hacia adelante al duty configurado
  DC_StartForward(cfg.hbridge_duty_0_to_1);


  // Calcular límites virtuales de recorrido a partir de la config de usuario
  float k_steps_per_mm = steps_per_mm(&cfg);
  g_min_steps = 0;
  g_max_steps = (int32_t)lroundf(cfg.traverse_width_mm * k_steps_per_mm);
  g_pos_steps = 0;                // después de HOME, este será nuestro origen
  g_traverse_passes_done = 0;
  g_run_dir = 1;                  // alejándose de HOME (positivo)
  g_last_limit_hit = LIMIT_NONE;
  Config_RecomputeDerived();

  // Habilitar interrupción de actualización de TIM2 para contar pasos
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);

  // Estado inicial de la máquina: arrancamos haciendo HOMING
  g_state = STATE_IDLE; // esperamos botón para iniciar homing
  g_homed_done = 0;
  g_run_requested = 0;

  Console_PrintBanner();
  debug_printf("[STATE] BOOT -> IDLE (espera RUN/B1)\r\n");



  // H-bridge del carrete: modo y duty inicial
  // Si querés el DC apagado durante homing:
  //   HBridge_SetMode(HBR_COAST);
  //   HBridge_SetPWM_Duty(0.0f);
  // Si querés que ya gire:
  HBridge_SetMode(HBR_FWD);
  HBridge_SetPWM_Duty(cfg.hbridge_duty_0_to_1);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


// Máquina de estados principal:
// - STATE_HOMING: buscar sensor HOME y fijar origen (se activa al inicio o por botón si no se homingueó)
// - STATE_IDLE: quieto esperando orden
// - STATE_RUNNING: programa de bobinado
    Machine_Task();

    // Consola UART no bloqueante
    Console_Poll();

    // Pequeño delay para no saturar la CPU ni el bus
    HAL_Delay(10);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1800;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 3600;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DM542_DIR_Pin|DM542_EN_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L298_IN1_Pin|L298_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DM542_DIR_Pin DM542_EN_Pin */
  GPIO_InitStruct.Pin = DM542_DIR_Pin|DM542_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : L298_IN1_Pin L298_IN2_Pin */
  GPIO_InitStruct.Pin = L298_IN1_Pin|L298_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : HOME_Pin */
  GPIO_InitStruct.Pin = HOME_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HOME_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Tarea principal de la máquina de estados
static void Machine_Task(void)
{
  switch (g_state) {

  case STATE_HOMING:


	// 1) Asegurar dirección hacia el sensor HOME
    //    Elegí el sentido correcto según cómo esté montado.
    Steppers_SetDir(1);                         // 1 va a home , 0 va a afuera

    // 2) Asegurar que el stepper esté moviéndose a velocidad de homing
    Steppers_SetStepFreq_Hz(FSTEP_HOME_HZ);

    // 3) Chequear sensor HOME
    if (HomeSensor_IsActive()) {
      // Llegamos al sensor: detener stepper y fijar origen
      Steppers_SetStepFreq_Hz(0.0f);
      HAL_Delay(700);
      g_pos_steps = 0;
      g_min_steps = 0;      // podés redefinir límites acá si querés
      // g_max_steps ya lo calculaste antes con traverse_width_mm
	      g_homed_done = 1;
	      s_backoff_done = 0;   // después del homing haremos el retroceso de 5 mm
	      g_offset60_done = 0;  // todavía no hicimos el avance de 60mm

      // Más adelante acá podés hacer un backoff (retroceder unos pasos)
      // si necesitás separar del sensor físico.

      debug_printf("[HOMING] Detectado HOME. pos=0\r\n");
	  SetState(STATE_IDLE, "homing completado");


    }
    break;

  case STATE_IDLE:
  {

    // Botón START siempre marca el pedido de ciclo
    if (Button_Read())
    {
      g_run_requested = 1;
      debug_printf("[IDLE] START recibido\r\n");
    }

    // Si no hay pedido de ciclo, quieto
    Steppers_SetStepFreq_Hz(0.0f);
    if (!g_run_requested) break;

    // Si el sensor está activo, damos por homing hecho
    if (HomeSensor_IsActive()) {
      g_homed_done = 1;
      g_pos_steps = 0;
    }

    // 1) Si todavía NO se hizo homing en esta sesión, ir a HOMING
    if (!g_homed_done)
    {
      SetState(STATE_HOMING, "pedido de ciclo");
      break;
    }


    // 2) Tras HOMING, avanzar primero 60mm alejándonos de HOME antes de entrar a RUNNING
    if (!g_offset60_done)
    {
      float k_spmm = steps_per_mm(&cfg);
      int32_t target_steps = (int32_t)lroundf(60.0f * k_spmm); // 60mm desde HOME
      // Evitar quedar justo en el límite para no rebotar al instante
      int32_t max_target = g_max_steps - g_limit_clear_steps;
      if (max_target < 0) max_target = 0;
      if (target_steps > max_target) target_steps = max_target;

      // Dirección alejándose de HOME (incrementa posición)
      g_run_dir = 0;
      Steppers_SetDir(g_run_dir);

      if (g_pos_steps < target_steps)
      {
        // Seguimos avanzando hacia los 60mm
        Steppers_SetStepFreq_Hz(FSTEP_HOME_HZ);
        debug_printf("[IDLE] Offset 60mm: pos=%ld steps, target=%ld\r\n",
                     (long)g_pos_steps, (long)target_steps);
        break; // aún no llegamos, no pasar a RUNNING
      }

      // Llegamos (o superamos levemente) los 60mm
      Steppers_SetStepFreq_Hz(0.0f);
      g_offset60_done = 1;
      // Re-centrar coordenadas: este punto pasa a ser pos=0/min=0
      g_pos_steps = 0;
      g_min_steps = 0;
      g_last_limit_hit = LIMIT_NONE;
      debug_printf("[IDLE] Offset 60mm completado. Nuevo origen pos=0 (steps=%ld)\r\n", (long)g_pos_steps);
      // seguimos con la lógica normal para entrar a RUNNING
    }

    // 3) Todo listo → arrancar RUNNING automáticamente
    Steppers_SetStepFreq_Hz(0.0f);
    float f = Compute_Stepper_Freq_From_DC(&cfg);

    // Reset de contadores de pasada y banderas de límites
    g_traverse_passes_done = 0;
    g_last_limit_hit = LIMIT_NONE;

    // Nos aseguramos de ir alejándonos de HOME
    g_run_dir = 0;
    Steppers_SetDir(g_run_dir);        // mismo comentario, invertí si hace falta
    Steppers_SetStepFreq_Hz(f);

    debug_printf("[IDLE] RUN request -> RUNNING\r\n");

    SetState(STATE_RUNNING, "arranque de ciclo");
    break;
  }

  case STATE_RUNNING:
  {

    // Log periódico de avance: posición actual y distancia al próximo rebote
    static uint32_t s_last_run_log_ms = 0;

    DC_StartForward(cfg.hbridge_duty_0_to_1);

    // Rearmar detección de límite solo tras haberse alejado del borde
    if (g_last_limit_hit != LIMIT_NONE) {
      if ((g_pos_steps > g_min_steps + g_limit_clear_steps) &&
          (g_pos_steps < g_max_steps - g_limit_clear_steps)) {
        g_last_limit_hit = LIMIT_NONE;
      }
    }

    // Chequeo de límites
    LimitHit_t hit = LIMIT_NONE;
    uint8_t next_dir = g_run_dir;
    if (g_pos_steps >= g_max_steps) {
      hit = LIMIT_MAX;
      next_dir = 1;                 //  1 va a home. volver hacia HOME
      g_pos_steps = g_max_steps;    // clamp
    } else if (g_pos_steps <= g_min_steps) {
      hit = LIMIT_MIN;
      next_dir = 0;                 // 0 se aleja de home .alejarse de HOME
      g_pos_steps = g_min_steps;    // clamp
    }

    if (hit != LIMIT_NONE && hit != g_last_limit_hit) {
      g_last_limit_hit = hit;
      g_traverse_passes_done++;
      g_run_dir = next_dir;

      // Pequeña pausa para amortiguar el rebote mecánico
     // Steppers_SetStepFreq_Hz(FSTEP_RUN_HZ);
      HAL_Delay(BOUNCE_DWELL_MS);

      // Log rebote una sola vez por evento de límite
      debug_printf("[RUNNING] Limite %s -> rebote dir=%u, pasada=%lu\r\n",
                   (hit == LIMIT_MAX) ? "MAX" : "MIN",
                   g_run_dir,
                   (unsigned long)g_traverse_passes_done);

      if (cfg.traverse_passes_target > 0 &&
          g_traverse_passes_done >= cfg.traverse_passes_target) {
        Steppers_SetStepFreq_Hz(0.0f);
        DC_Brake();
        g_run_requested = 0;
        g_homed_done = 0;     // queremos volver a homing
        s_backoff_done = 0;
        SetState(STATE_HOMING, "RUN completo -> volver a HOME");
        debug_printf(">> Run completo (%lu pasadas). Estado: HOMING\r\n",
                     (unsigned long)g_traverse_passes_done);
        break;
      }
    }

    // Velocidad calculada desde los parámetros de bobinado
    float f = Compute_Stepper_Freq_From_DC(&cfg);

    // Dirección de avance según el sentido actual
    Steppers_SetDir(g_run_dir);            // invertir si ves que va para el otro lado

    // Aplicar frecuencia calculada
    Steppers_SetStepFreq_Hz(f);

    // Telemetría: cuánto nos movimos y cuánto falta para rebotar
    uint32_t now_ms = HAL_GetTick();
    if (now_ms - s_last_run_log_ms >= 200) { // ~5 Hz para no saturar UART
      s_last_run_log_ms = now_ms;
      float k_spmm = steps_per_mm(&cfg);
      if (k_spmm < 1e-6f) k_spmm = 1.0f; // evitar div por cero si config inválida

      // Copias locales para evitar inconsistencias mientras se actualizan en la ISR
      int32_t pos_steps = g_pos_steps;
      int32_t min_steps = g_min_steps;
      int32_t max_steps = g_max_steps;

      // Escalado x10 para imprimir con enteros (evita %f en printf)
      int32_t pos_mm10         = (int32_t)lroundf(((float)pos_steps / k_spmm) * 10.0f);
      int32_t dist_to_min_mm10 = (int32_t)lroundf(((float)(pos_steps - min_steps) / k_spmm) * 10.0f);
      if (dist_to_min_mm10 < 0) dist_to_min_mm10 = 0;
      int32_t dist_to_max_mm10 = (int32_t)lroundf(((float)(max_steps - pos_steps) / k_spmm) * 10.0f);
      if (dist_to_max_mm10 < 0) dist_to_max_mm10 = 0;
      int32_t dist_to_next_mm10 = g_run_dir ? dist_to_min_mm10 : dist_to_max_mm10;
      int32_t f_hz10 = (int32_t)lroundf(f * 10.0f);

      debug_printf("[RUNNING] pos=%ld.%01ldmm dir=%u f=%ld.%01ldHz -> faltan %ld.%01ldmm al rebote (min=%ld.%01ld, max=%ld.%01ld) steps=%ld\r\n",
                   (long)(pos_mm10 / 10), (long)(labs(pos_mm10) % 10),
                   g_run_dir,
                   (long)(f_hz10 / 10), (long)(labs(f_hz10) % 10),
                   (long)(dist_to_next_mm10 / 10), (long)(labs(dist_to_next_mm10) % 10),
                   (long)(dist_to_min_mm10 / 10), (long)(labs(dist_to_min_mm10) % 10),
                   (long)(dist_to_max_mm10 / 10), (long)(labs(dist_to_max_mm10) % 10),
                   (long)pos_steps);
    }

    break;
  }

  default:
    g_state = STATE_HOMING;
    break;
  }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    // Si el stepper está parado, no contamos posición (evita drift por PWM del DC)
    if (g_step_freq_hz <= 0.0f) return;

    // Siempre actualizamos la posición del carro
    // g_step_dir = 1 significa que vamos hacia HOME (decrementa pos)
    if (g_step_dir)
      g_pos_steps--;
    else
      g_pos_steps++;
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
