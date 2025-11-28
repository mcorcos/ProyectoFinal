/*
 * ESP32 WiFi <-> UART bridge para consola del STM32.
 * - Conecta a WiFi (STA), y si falla levanta un AP de respaldo.
 * - Expone un servidor TCP (puerto 2323) tipo telnet.
 * - Lo que llega por TCP se envía por UART2 al STM32 (USART2).
 * - Lo que llega por UART2 se reenvía a todos los clientes TCP.
 *
 * Ajusta:
 *   - WIFI_SSID / WIFI_PASS
 *   - Pines RX/TX del ESP32 para Serial2 (por defecto RX2=16, TX2=17).
 *   - BAUD si cambiaste la consola del STM32.
 *
 * Uso:
 *   1) Flashear en el ESP32.
 *   2) Conectar: ESP32 TX2 -> STM32 RX (PA3), ESP32 RX2 -> STM32 TX (PA2), GND común.
 *   3) Conectar por telnet: telnet <ip> 2323. Verás la consola (HELP, RUN, etc).
 */

#include <WiFi.h>

// ---------- CONFIGURACIÓN ----------
static const char *WIFI_SSID = "pone_tu_ssid";
static const char *WIFI_PASS = "pone_tu_pass";

static const int   UART_BAUD = 115200;
static const int   UART_RX_PIN = 16; // ESP32 RX2
static const int   UART_TX_PIN = 17; // ESP32 TX2

static const uint16_t TCP_PORT = 2323;
static const uint8_t  MAX_CLIENTS = 2;

// ---------- ESTADO ----------
WiFiServer server(TCP_PORT);
WiFiClient clients[MAX_CLIENTS];

// ---------- HELPERS ----------
static void addClient(WiFiClient c) {
  for (uint8_t i = 0; i < MAX_CLIENTS; i++) {
    if (!clients[i] || !clients[i].connected()) {
      clients[i].stop();
      clients[i] = c;
      clients[i].print("== STM32 console bridge ==\r\n");
      clients[i].print("Comandos: HELP, CFG?, STAT?, WIRE x, TURNS x, RPM x, DUTY x, RUN, STOP, HOME\r\n\r\n");
      return;
    }
  }
  // Sin lugar
  c.println("Server full");
  c.stop();
}

static void broadcast(const uint8_t *buf, size_t len) {
  for (uint8_t i = 0; i < MAX_CLIENTS; i++) {
    if (clients[i] && clients[i].connected()) {
      clients[i].write(buf, len);
    }
  }
}

static void handleNewClients() {
  WiFiClient c = server.accept();
  if (c) addClient(c);
}

static void handleClientToUart() {
  uint8_t buf[64];
  for (uint8_t i = 0; i < MAX_CLIENTS; i++) {
    if (clients[i] && clients[i].connected()) {
      int n = clients[i].available();
      if (n > 0) {
        if (n > (int)sizeof(buf)) n = sizeof(buf);
        n = clients[i].read(buf, n);
        if (n > 0) Serial2.write(buf, n);
      }
    }
  }
}

static void handleUartToClient() {
  uint8_t buf[64];
  int n = Serial2.available();
  if (n > 0) {
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    n = Serial2.readBytes(buf, n);
    if (n > 0) broadcast(buf, n);
  }
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  Serial.println("\r\n[bridge] Boot");

  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint8_t retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 50) {
    delay(200);
    Serial.print(".");
    retries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\r\n[bridge] STA IP: %s\r\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\r\n[bridge] STA failed. Starting AP...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("STM32-Bridge", "stm32bridge");
    Serial.printf("[bridge] AP IP: %s\r\n", WiFi.softAPIP().toString().c_str());
  }

  server.begin();
  server.setNoDelay(true);
  Serial.printf("[bridge] TCP server on %u\r\n", TCP_PORT);
}

// ---------- LOOP ----------
void loop() {
  handleNewClients();
  handleClientToUart();
  handleUartToClient();
}
