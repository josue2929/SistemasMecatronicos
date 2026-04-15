# Referencia Completa de Configuración de Sensores

## 1. Mapa de Pines GPIO

| Sensor | Tipo | Pin ESP32 | Descripción |
|--------|------|-----------|-------------|
| S1 (Tanque 1) | Digital | GPIO32 | Nivel/proximidad |
| S2 (Tanque 2) | Digital | GPIO33 | Nivel/proximidad |
| S3 (Tanque 3) | Digital | GPIO34 | Nivel/proximidad |
| Presión (T2→T3) | ADC | GPIO36 (ADC1_CH0) | Sensor analógico |
| Temperatura (T3) | ADC + NTC | GPIO39 (ADC1_CH3) | NTC 10K B3950 con divisor |
| Caudalímetro | Digital (pulsos) | GPIO27 | ZJ-S201 con contador ISR |
| Vibración | Digital | GPIO21 | Sensor digital de vibración |

## 2. Conexiones por Sensor

### 2.1 Sensores Digitales de Nivel (S1, S2, S3)

**Conexión:**
- Cable del sensor → GPIO correspondiente
- GND → GND del ESP32

**Configuración:**
```c
#define APP_LEVEL_SENSORS_ACTIVE_HIGH     true
#define APP_LEVEL_SENSORS_ENABLE_PULLUP   false
#define APP_LEVEL_SENSORS_ENABLE_PULLDOWN false
```

**Lectura esperada:**
- level = 0 (bajo/vacío)
- level = 1 (alto/lleno)

---

### 2.2 Sensor de Presión

**Especificación:**
- 3 cables: Rojo (3.3V), Negro (GND), Amarillo (salida)

**Conexión:**
- Rojo → 3.3V del ESP32
- Negro → GND del ESP32
- Amarillo → GPIO36 (ADC1_CH0)

**Configuración ADC:**
```c
#define APP_ADC_ATTENUATION               ADC_ATTEN_DB_12
#define APP_ADC_BITWIDTH                  ADC_BITWIDTH_DEFAULT
```

**Calibración:**
```c
#define CAL_ADC_RAW_MAX                  4095.0f
#define CAL_ADC_REFERENCE_VOLTAGE_V      3.300f
#define CAL_ADC_FRONTEND_GAIN            1.000f
#define CAL_ADC_FRONTEND_OFFSET_V        0.000f
#define CAL_PRESSURE_KPA_PER_V           100.000f
#define CAL_PRESSURE_OFFSET_KPA          0.000f
```

**Fórmula:**
```
voltage_v = (raw_adc / 4095.0) * 3.3
pressure_kpa = voltage_v * 100.0 + 0.0
```

**Rango típico:**
- Raw ADC: 0–4095
- Voltaje: 0.0–3.3 V
- Presión: 0–330 kPa

---

### 2.3 Sensor de Temperatura (NTC 10K B3950)

**Especificación:**
- 2 cables (resistencia pasiva)
- Modelo: NTC 10K B3950
- R @ 25°C = 10 kΩ
- Beta = 3950

**Conexión (Divisor de Tensión):**
```
3.3V ──[ 1kΩ ]───┐
                  │
                GPIO39 (ADC1_CH3)
                  │
                 NTC
                  │
                 GND
```

**Componentes requeridos:**
- Resistor de 1 kΩ (divisor de tensión)

**Configuración:**
```c
#define CAL_TEMPERATURE_NTC_BETA         3950.0f
#define CAL_TEMPERATURE_NTC_R25          10000.0f
#define CAL_TEMPERATURE_DIVIDER_R        1000.0f
#define CAL_TEMPERATURE_K0               298.15f
#define CAL_TEMPERATURE_C0              -273.15f
```

**Fórmula Steinhart-Hart simplificada:**
```
voltage_v = (raw_adc / 4095.0) * 3.3

r_ntc = (R_div * Vmeasured) / (Vcc - Vmeasured)
r_ntc = (1000.0 * voltage_v) / (3.3 - voltage_v)

inv_T = (1/T0) + (1/B) * ln(R_ntc / R25)
inv_T = (1/298.15) + (1/3950) * ln(r_ntc / 10000)

temperature_kelvin = 1 / inv_T
temperature_celsius = temperature_kelvin - 273.15
```

**Rango típico:**
- Voltaje a 25°C: ~1.65 V (punto medio)
- Voltaje a 0°C: más alto (~2.2 V)
- Voltaje a 50°C: más bajo (~1.2 V)
- Temperatura: -10°C a +80°C (rango operativo)

**Calibración (si es necesaria):**
- Tomar lecturas a temperaturas conocidas
- Ajustar `CAL_TEMPERATURE_DIVIDER_R` si usas otro resistor

---

### 2.4 Caudalímetro (ZJ-S201)

**Especificación:**
- Modelo: ZJ-S201 Water Flow Sensor
- Salida: pulsos digitales compatible 3.3V
- Típico: 450 pulsos/litro
- 3 cables: Rojo (alimentación), Negro (GND), Amarillo (pulsos)

**Conexión:**
- Rojo → 3.3V o 5V (verifica datasheet del sensor específico)
- Negro → GND del ESP32
- Amarillo → GPIO27

**Con pull-up interno:**
```c
#define APP_FLOWMETER_ENABLE_PULLUP       true
#define APP_FLOWMETER_ENABLE_PULLDOWN     false
```

**Con pull-up externo (si no usas interno):**
- Resistor 10 kΩ entre GPIO27 y 3.3V

**Calibración:**
```c
#define CAL_FLOWMETER_PULSES_PER_LITER   450.000f
#define CAL_FLOWMETER_FLOW_OFFSET_L_MIN  0.000f
```

**Fórmula:**
```
flow_l_min = (pulses_per_second * 60) / 450
```

**Calibración real:**
1. Medir 1 litro de agua en recipiente calibrado
2. Contar pulsos observados (pulse_count)
3. Calcular: `pulses_per_liter = pulse_count_total / volumen_litros`
4. Actualizar `CAL_FLOWMETER_PULSES_PER_LITER`

---

### 2.5 Sensor de Vibración

**Conexión:**
- Señal digital del sensor → GPIO21
- VCC del sensor → 3.3V
- GND del sensor → GND del ESP32

**Configuración:**
```c
#define APP_VIBRATION_SENSOR_ACTIVE_HIGH     true
#define APP_VIBRATION_SENSOR_ENABLE_PULLUP   false
#define APP_VIBRATION_SENSOR_ENABLE_PULLDOWN false
```

**Lectura esperada:**
- vibration_level = 0 (sin vibración)
- vibration_level = 1 (vibración detectada)
- vibration_edges incrementa cuando cambia el estado

---

## 3. Configuración Global (app_config.h)

```c
#define APP_SELECTED_SENSOR_TEST TEST_ALL_SENSORS

#define APP_DIGITAL_SAMPLE_PERIOD_MS      500U
#define APP_ADC_SAMPLE_PERIOD_MS          1000U
#define APP_FLOW_SAMPLE_PERIOD_MS         1000U
#define APP_ALL_SENSORS_SAMPLE_PERIOD_MS  1000U

#define APP_LEVEL_SENSORS_ACTIVE_HIGH     true
#define APP_LEVEL_SENSORS_ENABLE_PULLUP   false
#define APP_LEVEL_SENSORS_ENABLE_PULLDOWN false

#define APP_FLOWMETER_ENABLE_PULLUP       true
#define APP_FLOWMETER_ENABLE_PULLDOWN     false

#define APP_VIBRATION_SENSOR_ACTIVE_HIGH     true
#define APP_VIBRATION_SENSOR_ENABLE_PULLUP   false
#define APP_VIBRATION_SENSOR_ENABLE_PULLDOWN false

#define APP_ADC_ATTENUATION               ADC_ATTEN_DB_12
#define APP_ADC_BITWIDTH                  ADC_BITWIDTH_DEFAULT

#define APP_ENABLE_STATUS_LED             false
```

---

## 4. Salidas del Monitor Serial

### 4.1 TEST_S1_LEVEL (y S2, S3)
```
timestamp_ms,gpio_pin,level,edge_type,edge_count
259552,32,1,NONE,1
260052,32,1,NONE,1
```

### 4.2 TEST_PRESSURE
```
timestamp_ms,raw_adc,voltage_v,pressure_kpa
24052,187,0.151,15.07
```

### 4.3 TEST_TEMPERATURE
```
timestamp_ms,raw_adc,voltage_v,temperature_c
10052,2048,1.650,25.00
```

### 4.4 TEST_FLOWMETER
```
timestamp_ms,pulse_count,pulses_per_second,estimated_flow_l_min
10052,22,22.00,2.933
11052,159,137.00,18.267
```

### 4.5 TEST_ALL_SENSORS
```
S1_LEVEL,S2_LEVEL,S3_LEVEL: timestamp_ms,gpio,level,edge_type,edge_count
PRESSURE: timestamp_ms,raw_adc,voltage_v,pressure_kpa
TEMPERATURE: timestamp_ms,raw_adc,voltage_v,temperature_c
FLOWMETER: timestamp_ms,pulse_count,pulses_per_second,estimated_flow_l_min
VIBRATION: timestamp_ms,gpio,level,edge_type,edge_count
```

---

## 5. Procedimiento de Testing Paso a Paso

### 5.1 Test Individual de S1
1. Selecciona `TEST_S1_LEVEL` en `app_config.h`
2. Conecta sensor S1 a GPIO32
3. Compila: `idf.py build`
4. Flashea: `idf.py -p COM6 flash monitor`
5. Activa/desactiva sensor manualmente
6. Verifica `level` cambio 0↔1 y `edge_count` incremente

### 5.2 Test Individual de Presión
1. Selecciona `TEST_PRESSURE` en `app_config.h`
2. Conecta sensor presión: Rojo→3.3V, Negro→GND, Amarillo→GPIO36
3. Flashea y abre monitor
4. Verifica `raw_adc` aumente al aumentar presión
5. Verifica `voltage_v` entre 0.0–3.3 V
6. Anota puntos de calibración

### 5.3 Test Individual de Temperatura
1. Selecciona `TEST_TEMPERATURE` en `app_config.h`
2. Conecta NTC: cable1→GND, cable2→GPIO39
3. Conecta resistor 1kΩ: 3.3V→GPIO39
4. Flashea y abre monitor
5. Verifica `temperature_c` cambie al cambiar temperatura
6. Compara con termómetro de referencia

### 5.4 Test Individual de Caudalímetro
1. Selecciona `TEST_FLOWMETER` en `app_config.h`
2. Conecta sensor: Rojo→3.3V, Negro→GND, Amarillo→GPIO27
3. Activa pull-up en `app_config.h` si es necesario
4. Flashea y abre monitor
5. Genera flujo o pulsos
6. Verifica `pulse_count` incremente

### 5.5 Test Todos los Sensores
1. Selecciona `TEST_ALL_SENSORS` en `app_config.h`
2. Conecta todos los sensores según especificaciones
3. Flashea y abre monitor
4. Cambia estados sin esperar bloqueos
5. Verifica que todas las lecturas aparezcan periódicamente

---

## 6. Diagnóstico de Problemas

### Sensor de Nivel lee 0 siempre
- Verifica conexión a GPIO
- Revisa que no esté flotante (si es necesario, agrega pull-up)

### Presión lee 0V siempre
- Verifica alimentación 3.3V
- Revisa conexión del ADC
- Mide voltaje con multímetro en GPIO36

### Presión saturada (3.3V siempre)
- El divisor/acondicionamiento está mal
- Revisa acoplamiento

### Temperatura lee -273.15°C
- Voltaje en GPIO39 es 0V
- Verifica divisor: resistor 1kΩ entre 3.3V y GPIO39
- Verifica conexión NTC a GPIO39 y GND

### Temperatura saturada (>150°C)
- Voltaje en GPIO39 es 3.3V
- Revisa que NTC esté entre GPIO39 y GND

### Caudalímetro no cuenta pulsos
- Verifica conexión GPIO27
- Revisa pull-up (interno o externo)
- Confirma que sensor entrega pulsos compatibles 3.3V
- Revisa si necesita 5V (entonces usa regulador)

---

## 7. Calibración Final

Para cada sensor, después de confirmar que funciona:

1. **Presión:** Toma 2-3 puntos conocidos (presión mínima, media, máxima)
   - Anota raw_adc y presión real
   - Calcula pendiente y offset
   - Actualiza `CAL_PRESSURE_KPA_PER_V` y `CAL_PRESSURE_OFFSET_KPA`

2. **Temperatura:** Toma puntos a 0°C, 25°C, 50°C
   - Verifica que la curva Steinhart-Hart se ajuste
   - Si desviación sistemática, ajusta `CAL_TEMPERATURE_NTC_BETA` o divisor

3. **Caudalímetro:** Mide flujo real (ej: 1 litro en X segundos)
   - Calcula pulsos/litro real
   - Actualiza `CAL_FLOWMETER_PULSES_PER_LITER`

---

## 8. Archivo de Configuración Header (io_map.h)

```c
#define IO_GPIO_S1_LEVEL              GPIO_NUM_32
#define IO_GPIO_S2_LEVEL              GPIO_NUM_33
#define IO_GPIO_S3_LEVEL              GPIO_NUM_34

#define IO_GPIO_PRESSURE_ADC          GPIO_NUM_36
#define IO_ADC_UNIT_PRESSURE          ADC_UNIT_1
#define IO_ADC_CHANNEL_PRESSURE       ADC_CHANNEL_0

#define IO_GPIO_TEMPERATURE_T3_ADC    GPIO_NUM_39
#define IO_ADC_UNIT_TEMPERATURE_T3    ADC_UNIT_1
#define IO_ADC_CHANNEL_TEMPERATURE_T3 ADC_CHANNEL_3

#define IO_GPIO_FLOWMETER             GPIO_NUM_27

#define IO_GPIO_VIBRATION             GPIO_NUM_21
```

---

## 9. Notas Importantes

- **Masa común**: Siempre conecta el GND de todos los sensores al mismo GND del ESP32
- **Alimentación diferenciada**: Si algunos sensores necesitan 5V, usa regulador 5V→3.3V para salidas
- **Pull-up/Pull-down**: Revisa datasheet de cada sensor para saber si necesita
- **Ruido**: Si hay lecturas ruidosas, agrega capacitor de 100nF entre GPIO y GND
- **Timeouts**: Los tiempos de muestreo pueden ajustarse en `app_config.h`

