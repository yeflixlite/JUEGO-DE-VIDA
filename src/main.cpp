#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// ======= CONFIGURACIÓN =======
// Hardware
const uint8_t PIN_NEOPIXEL = 22;           // GPIO para datos
const uint8_t MATRIX_W = 16;               // ancho físico
const uint8_t MATRIX_H = 16;               // alto físico
const uint16_t NUM_PIXELS = MATRIX_W * MATRIX_H;

// Parámetros de funcionamiento
const uint8_t BRIGHTNESS = 128;            // 0-255
const uint16_t STEP_DELAY_MS = 150;        // velocidad entre generaciones
const bool WRAP_EDGES = true;              // si true, mundo toroide

// Detección de ciclos
const uint8_t HISTORY = 8;                 // cuantas generaciones recordar

Adafruit_NeoPixel strip(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// Estados: usamos dos buffers para no corromper la generación en curso
static uint8_t state[2][NUM_PIXELS];       // 0 = muerto, 1 = vivo
static uint8_t cur = 0;                    // índice del buffer actual

// Historial simple de checksums para detectar repeticiones/estabilidad
static uint32_t historySums[HISTORY];
static uint8_t historyPos = 0;

// ======= DECLARACIONES DE FUNCIONES =======
// Convierte coordenadas (x,y) a el índice físico del LED en la tira
inline uint16_t xyToIndex(int x, int y);

// Cuenta vecinos vivos alrededor de (x,y)
uint8_t countNeighbors(int x, int y);

// Calcula la siguiente generación en el buffer opuesto
void stepGeneration();

// Dibuja la matriz actual en los NeoPixels
void drawMatrix();

// Inicializa aleatoriamente el estado
void randomizeState();

// Calcula checksum simple de la generación actual
uint32_t checksumCurrent();

// Reinicia la simulación (randomize + clear history)
void restartSimulation();

// ======= IMPLEMENTACIÓN =======

// Conversión (x,y) -> índice con soporte para cableado en zig-zag (serpentina).
// Se asume que la tira está conectada en la esquina (0,0) y se recorren filas.
// Filas pares se mapean izquierda->derecha, filas impares derecha->izquierda.
inline uint16_t xyToIndex(int x, int y) {
  // manejar envoltura si se pide
  if (WRAP_EDGES) {
    x = (x % MATRIX_W + MATRIX_W) % MATRIX_W;
    y = (y % MATRIX_H + MATRIX_H) % MATRIX_H;
  } else {
    if (x < 0 || x >= MATRIX_W || y < 0 || y >= MATRIX_H) return 0xFFFF;
  }
  // en serpentina: cada fila alterna dirección
  if (y % 2 == 0) {
    return (uint16_t)(y * MATRIX_W + x);
  } else {
    return (uint16_t)(y * MATRIX_W + (MATRIX_W - 1 - x));
  }
}

// Cuenta vecinos vivos alrededor con posibilidad de envolver bordes.
uint8_t countNeighbors(int x, int y) {
  uint8_t cnt = 0;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) continue;
      int nx = x + dx;
      int ny = y + dy;
      if (!WRAP_EDGES) {
        if (nx < 0 || nx >= MATRIX_W || ny < 0 || ny >= MATRIX_H) continue;
      }
      uint16_t idx = xyToIndex(nx, ny);
      if (idx == 0xFFFF) continue;
      if (state[cur][idx]) ++cnt;
    }
  }
  return cnt;
}

// Actualiza la generación: aplica reglas de Conway y escribe en buffer opuesto.
void stepGeneration() {
  uint8_t next = 1 - cur;
  // calcular nueva generación
  for (int y = 0; y < MATRIX_H; ++y) {
    for (int x = 0; x < MATRIX_W; ++x) {
      uint16_t i = xyToIndex(x, y);
      uint8_t alive = state[cur][i];
      uint8_t n = countNeighbors(x, y);
      uint8_t newState = 0;
      if (alive) {
        // Sobrevivencia con 2 o 3 vecinos
        if (n == 2 || n == 3) newState = 1;
      } else {
        // Nace con exactamente 3 vecinos
        if (n == 3) newState = 1;
      }
      state[next][i] = newState;
    }
  }
  cur = next;
}

// Dibuja usando verde para vivos, apagado para muertos.
void drawMatrix() {
  strip.setBrightness(BRIGHTNESS);
  for (uint16_t i = 0; i < NUM_PIXELS; ++i) {
    if (state[cur][i]) {
      strip.setPixelColor(i, strip.Color(0, 200, 0)); // verde
    } else {
      strip.setPixelColor(i, 0);
    }
  }
  strip.show();
}

// Inicializa el estado con aleatorio usando ruido simple.
void randomizeState() {
  // llenar con probabilidad ~0.3 de vida
  for (uint16_t i = 0; i < NUM_PIXELS; ++i) {
    state[cur][i] = (random(100) < 30) ? 1 : 0;
  }
}

// Checksum simple sumando bits (rápido)
uint32_t checksumCurrent() {
  uint32_t s = 0;
  for (uint16_t i = 0; i < NUM_PIXELS; ++i) s += state[cur][i];
  // mezclar un poco
  s = (s << 5) ^ (s >> 3) ^ 0x9e3779b9;
  return s;
}

void restartSimulation() {
  // borrar buffers
  memset(state, 0, sizeof(state));
  randomizeState();
  // limpiar historial
  for (uint8_t i = 0; i < HISTORY; ++i) historySums[i] = 0;
  historyPos = 0;
}

// ======= SETUP Y LOOP =======
void setup() {
  // Inicializaciones
  strip.begin();
  strip.show();
  strip.setBrightness(BRIGHTNESS);
  // semilla aleatoria usando pin analógico no conectado
  randomSeed(analogRead(0));
  restartSimulation();
}

void loop() {
  // Dibujar, avanzar y gestionar reinicios automáticos
  drawMatrix();
  delay(STEP_DELAY_MS);

  // avanzar una generación
  stepGeneration();

  // Checksum para detección de repetición o extinción
  uint32_t s = checksumCurrent();
  bool found = false;
  for (uint8_t i = 0; i < HISTORY; ++i) {
    if (historySums[i] == s) { found = true; break; }
  }
  // almacenar
  historySums[historyPos++] = s;
  if (historyPos >= HISTORY) historyPos = 0;

  // condición de reinicio: población 0 o estado repetido
  uint32_t pop = 0;
  for (uint16_t i = 0; i < NUM_PIXELS; ++i) pop += state[cur][i];
  if (pop == 0 || found) {
    // pequeño efecto de apagado
    for (int b = BRIGHTNESS; b > 0; b -= 20) {
      strip.setBrightness(b);
      drawMatrix();
      delay(30);
    }
    restartSimulation();
  }
}

/*
  ======= ADICIONAL (INFORMACIÓN DEL PROYECTO) =======
  A continuación se incluyen como comentarios la configuración de platformio.ini,
  explicación del algoritmo, diagrama de conexión, recomendaciones de fuente y
  posibles mejoras. Esto se anexa en este archivo para entregar todo en un único
  fichero.

  1) platformio.ini

  [env:esp32doit-devkit-v1]
  platform = espressif32
  board = esp32doit-devkit-v1
  framework = arduino
  lib_deps = adafruit/Adafruit NeoPixel@^1.10.6

  2) Explicación del algoritmo
  - Usamos una matriz 16x16 almacenada en una tira de 256 LEDs en cableado serpentina.
  - Dos buffers "state[2]" garantizan que la generación actual no sea modificada mientras
    se calcula la siguiente.
  - Para cada celda contamos los 8 vecinos aplicando envoltura (WRAP_EDGES=true)
    para simular un mundo toroide. Las reglas aplicadas son las de Conway:
      * Célula viva con <2 vecinos muere (soledad).
      * Célula viva con 2 o 3 vecinos sobrevive.
      * Célula viva con >3 vecinos muere (sobrepoblación).
      * Célula muerta con exactamente 3 vecinos nace.
  - Guardamos checksums de las últimas generaciones para detectar repeticiones o estados
    estables y reiniciar automáticamente si se detecta un ciclo corto.

  3) Diagrama de conexión
  - NeoPixel DIN  -> GPIO22 (PIN_NEOPIXEL)
  - NeoPixel VCC  -> 5V (fuente externa)
  - NeoPixel GND  -> GND (común con ESP32 GND)
  - ESP32 VIN/5V -> no conectar a menos que use la misma fuente; preferible alimentar
    los LEDs desde una fuente 5V separada y compartir GND.

  4) Recomendaciones de fuente de alimentación
  - Un LED WS2812B consume hasta ~60 mA a blanco máximo (3 canales a 20 mA).
    Para 256 LEDs: 256 * 0.06 A = 15.36 A aproximado.
  - Recomendable fuente 5V con al menos 20 A para margen (o segmentar la tira
    y asegurar decoupling capacitors y cableado de baja resistencia).
  - Añadir un condensador electrolítico grande (1000 uF, 6.3V o más) en la VCC al inicio
    de la tira y una resistencia en serie pequeña (22-100 ohm) en la línea de datos.

  5) Mejoras futuras
  - Patrones predefinidos y selección de modos con botones físicos (start/stop/random).
  - Implementar botones para cambiar velocidad y brillo en tiempo real.
  - Añadir patrones iniciales clásicos (glider, pulsar, gosper gun adaptada) y un menu.
  - Optimización: usar bits en lugar de bytes para ahorrar memoria, cálculos por bloques,
    o usar SPI/I2S para salida de LEDs para mayor rendimiento.

*/