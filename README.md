# ProyectoFinal – Control de bobinadora (STM32F303RE)

Descripción corta: firmware generado en CubeIDE que controla un eje con STEP/DIR (DM542) y un motor DC con puente H (L298). Incluye homing con sensor inductivo, backoff, rebote en extremos virtuales y conteo de pasadas configurables.

## Diagrama rápido de estados
- HOMING: avanza hacia HOME a `FSTEP_HOME_HZ` hasta que el sensor (activo en bajo) se activa; fija pos=0.
- IDLE: hace un backoff de 5 mm alejándose de HOME, queda detenido y espera botón B1 (PC13 a 0).
- RUNNING: mueve el carro según la frecuencia calculada por `Compute_Stepper_Freq_From_DC` y rebota en `g_min_steps/g_max_steps`. Cuenta pasadas y vuelve a IDLE al alcanzar `cfg.traverse_passes_target` (>0) o rebota infinito si es 0.

## Pines relevantes (ver `Core/Inc/main.h`)
- Stepper DM542: DIR `PA1`, EN `PA4`, STEP `PA0` (TIM2_CH1).
- Sensor HOME: `PB5`, activo en 0.
- Botón placa (START): `PC13` (Nucleo B1, activo en 0).
- L298: IN1 `PB10`, IN2 `PB11`, ENA `PA8` (TIM1_CH1 en el cubo). **Nota:** el PWM actual del código sale por TIM2_CH3 `PA9` (ver sección Timers).

## Timers y PWM
- TIM2_CH1: STEP del DM542 (cálculo dinámico de PSC/ARR/CCR en `Steppers_SetStepFreq_Hz`).
- TIM2_CH3: PWM que hoy gobierna el L298 en el código (`DC_*`). Advertencia: al recalcular la frecuencia de STEP se reescribe PSC/ARR/CCR1 y afecta también al PWM del DC. TIM1 está inicializado y el pin ENA está en PA8/TIM1; mover el PWM del DC a TIM1 evitaría el acople.

## Configuración clave (`Core/Src/main.c`)
Estructura `cfg` editable en build-time:
- `steps_per_rev`, `microstepping`, `leadscrew_pitch_mm`, `traverse_width_mm`, `wire_diameter_mm`.
- `dc_target_rpm`, `hbridge_duty_0_to_1`.
- Límites de frecuencia: `fstep_min_hz`, `fstep_max_hz`.
- `traverse_passes_target`: 0 = rebote infinito; >0 detiene tras esa cantidad de toques de extremo.

Constantes de movimiento:
- `BOUNCE_DWELL_MS` (pausa al rebotar) y `LIMIT_CLEAR_MM` (histeresis de detección de límite).
- `FSTEP_HOME_HZ` fija la velocidad de homing.

## Cómo arranca
1) `main()` inicializa clocks/GPIO/UART/TIM1/TIM2, calcula pasos/mm y límites virtuales, habilita interrupción de TIM2 para contar pasos.
2) Arranca en HOMING con dirección hacia HOME y DC en forward al duty configurado.
3) Tras HOME hace backoff de 5 mm, queda en IDLE y espera botón. RUNNING rebota y cuenta pasadas.

## Logs por UART
- USART2 a 115200 8N1. Mensajes tipo `[HOMING]`, `[IDLE]`, `[RUNNING]` con posición, dirección y pasadas.

## Build/flash
- Proyecto CubeIDE. Micro: STM32F303RE. Perfil FLASH en `STM32F303RETX_FLASH.ld`. Para depurar usa la launch `ProyectoFinal Debug.launch`.

## Advertencias abiertas
- El acople por compartir TIM2 entre STEP y PWM del DC puede cambiar la velocidad del carrete cuando se ajusta la frecuencia de STEP. Considera migrar el PWM del DC a TIM1_CH1 (PA8) para aislarlo.
- Revisar que las direcciones (`Steppers_SetDir`) coincidan con el montaje (HOME y avance).
