# ProyectoFinal – Control de bobinadora (STM32F303RE)

Firmware CubeIDE para un eje STEP/DIR (DM542) y un motor DC con puente H (L298). Incluye homing con sensor inductivo, backoff, rebote con pasadas contadas y logs por UART.

## Índice
- [Estados](#estados)
- [Pines](#pines)
- [Timers](#timers)
- [Configuración rápida](#configuración-rápida)
- [Flujo de uso](#flujo-de-uso)
- [Logs UART](#logs-uart)
- [Build/flash](#buildflash)
- [Advertencias / TODO](#advertencias--todo)

## Estados
```
HOMING  --(sensor HOME activo)-->  IDLE --(B1 pulsado)--> RUNNING
   ^                                 ^                        |
   |---------------------------------|<------- rebote --------|
```
- HOMING: avanza hacia HOME a `FSTEP_HOME_HZ` hasta que el sensor (activo en bajo) se activa; fija `pos=0`.
- IDLE: hace un backoff de 5 mm alejándose de HOME, queda detenido y espera botón B1 (PC13 a 0).
- RUNNING: mueve el carro con la frecuencia de `Compute_Stepper_Freq_From_DC`, rebota en `g_min_steps/g_max_steps`, cuenta pasadas y vuelve a IDLE al alcanzar `cfg.traverse_passes_target` (>0) o sigue infinito si es 0.

## Pines
| Función            | MCU pin | Notas                            |
|--------------------|---------|----------------------------------|
| STEP DM542         | PA0     | TIM2_CH1                         |
| DIR DM542          | PA1     | GPIO                             |
| EN DM542           | PA4     | GPIO                             |
| HOME sensor        | PB5     | Activo en 0                      |
| Botón START (B1)   | PC13    | Activo en 0                      |
| L298 IN1 / IN2     | PB10/11 | GPIO                             |
| L298 ENA           | PA8     | TIM1_CH1 en pines del cubo       |
| PWM DC efectivo    | PA9     | **TIM2_CH3 usado en el código**  |

## Timers
| Timer/Canal | Uso actual              | Comentario |
|-------------|-------------------------|------------|
| TIM2_CH1    | STEP DM542              | `Steppers_SetStepFreq_Hz` recalcula PSC/ARR/CCR1 |
| TIM2_CH3    | PWM L298 (DC)           | Comparte PSC/ARR con CH1 → acople de RPM |
| TIM1_CH1    | Inicializado, libre     | Pin PA8 (ENA) definido; mover PWM aquí aislaría el DC |

## Configuración rápida (`Core/Src/main.c`)
Estructura `cfg`:
| Campo                   | Qué hace                                      |
|-------------------------|-----------------------------------------------|
| `steps_per_rev`         | Pasos enteros por vuelta del motor            |
| `microstepping`         | Subdivisión del driver                        |
| `leadscrew_pitch_mm`    | mm/vuelta del husillo                         |
| `traverse_width_mm`     | Recorrido total virtual                       |
| `wire_diameter_mm`      | Avance lateral por vuelta de carrete          |
| `dc_target_rpm`         | RPM objetivo del carrete                      |
| `hbridge_duty_0_to_1`   | Duty fijo para el DC                          |
| `fstep_min_hz` / `max`  | Clampeo de frecuencia de STEP                 |
| `traverse_passes_target`| 0 = rebote infinito; >0 detiene tras N toques |

Constantes:
- `BOUNCE_DWELL_MS`: pausa al rebotar.
- `LIMIT_CLEAR_MM`: histeresis de detección de límite.
- `FSTEP_HOME_HZ`: velocidad de homing.

## Flujo de uso
1) Flash/Reset: arranca HOMING dirigiéndose al sensor HOME.
2) Al activar HOME: fija `pos=0`, pasa a IDLE y ejecuta backoff de 5 mm alejándose del sensor.
3) En IDLE: espera botón B1. Al pulsar: resetea pasadas, calcula frecuencia de STEP desde `cfg` y arranca RUNNING alejándose de HOME.
4) RUNNING: rebota en límites, cuenta pasadas y frena en IDLE si `traverse_passes_target` > 0.

## Logs UART
- USART2 115200 8N1. Mensajes: `[HOMING]`, `[IDLE]`, `[RUNNING] pos=... dir=... pasa=...`.

## Build/flash
- Proyecto CubeIDE (STM32F303RE). Linker: `STM32F303RETX_FLASH.ld`. Launch: `ProyectoFinal Debug.launch`.

## Advertencias / TODO
- El PWM del DC comparte TIM2 con el STEP: cambiar la frecuencia del stepper cambia PSC/ARR y puede alterar RPM del carrete. Migrar PWM a TIM1_CH1 (PA8) eliminaría el acople.
- Verificar sentido de `Steppers_SetDir` según montaje (HOME y avance).
