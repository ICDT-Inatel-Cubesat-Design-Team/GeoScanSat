#include <SPI.h>
#include <LoRa.h>
#include <string.h>

// ===== Ligações do RFM95/RFM96 ao ESP32 DevKit =====
// NSS/CS  -> GPIO5
// MOSI    -> GPIO23
// MISO    -> GPIO19
// SCK     -> GPIO18
// RESET   -> GPIO16
// DIO0    -> GPIO4

#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23

#define LORA_SS    5
#define LORA_RST   16
#define LORA_DIO0  4

// ===== Cabeçalho do protocolo GS =====
#define GS_MAGIC0  'G'
#define GS_MAGIC1  'S'
#define GS_VERSION 1

// ===== Flags =====
#define GS_FLAG_SENSOR  0x01
#define GS_FLAG_MODULE  0x02
#define GS_FLAG_PAYLOAD 0x04
#define GS_FLAG_WATER   0x08

// ===== Tamanhos =====
#define GS_HEADER_LEN       8
#define GS_SENSOR_BLOCK_LEN 56
#define GS_OPTIONAL_LEN     16
#define GS_WATER_BLOCK_LEN  24
#define GS_MIN_LEN          64
#define GS_MAX_LEN          120

uint32_t readU32(const uint8_t *buffer, size_t offset)
{
  uint32_t value = 0;
  memcpy(&value, buffer + offset, sizeof(value));
  return value;
}

float readFloat(const uint8_t *buffer, size_t offset)
{
  float value = 0.0f;
  memcpy(&value, buffer + offset, sizeof(value));
  return value;
}

void printHex(const uint8_t *buffer, size_t length)
{
  Serial.println(F("HEX:"));

  for (size_t i = 0; i < length; i++) {
    if (buffer[i] < 0x10) Serial.print('0');
    Serial.print(buffer[i], HEX);
    Serial.print(' ');

    if ((i + 1) % 16 == 0) Serial.println();
  }

  if (length % 16 != 0) Serial.println();
}

bool blockAvailable(size_t received, size_t offset, size_t blockLength)
{
  return received >= (offset + blockLength);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("Inicializando receptor LoRa no ESP32..."));

  // No ESP32, os pinos SPI podem ser definidos explicitamente.
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

  // Define NSS, RESET e DIO0 do módulo LoRa.
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  // Frequência SPI reduzida para facilitar testes de bancada.
  LoRa.setSPIFrequency(1000000);

  if (!LoRa.begin(915E6)) {
    Serial.println(F("Erro ao inicializar o LoRa."));
    Serial.println(F("Verifique alimentacao 3.3 V, SPI e pino NSS."));

    while (true) {
      delay(1000);
    }
  }

  // Deve ser igual ao transmissor.
  LoRa.setSignalBandwidth(125E3);
  LoRa.setSpreadingFactor(7);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x4C);
  LoRa.disableCrc();  // deve ser igual ao transmissor
  LoRa.receive();

  Serial.println(F("Receptor LoRa pronto."));
}

void loop()
{
  const int packetSize = LoRa.parsePacket();

  if (packetSize <= 0) return;

  // O SX127x suporta pacotes de até 255 bytes; o maior pacote GS atual tem 120 bytes.
  uint8_t buffer[255];
  size_t received = 0;
  bool overflow = false;

  while (LoRa.available() && received < (size_t)packetSize) {
    const int value = LoRa.read();

    if (value < 0) break;

    if (received < sizeof(buffer)) {
      buffer[received++] = (uint8_t)value;
    } else {
      overflow = true;
      break;
    }
  }

  // Descarta qualquer byte excedente ainda presente no FIFO.
  while (LoRa.available()) {
    LoRa.read();
    overflow = true;
  }

  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("PACOTE GS RECEBIDO"));
  Serial.println(F("========================================"));

  Serial.print(F("Tamanho informado: "));
  Serial.print(packetSize);
  Serial.println(F(" bytes"));

  Serial.print(F("Bytes armazenados: "));
  Serial.print(received);
  Serial.println(F(" bytes"));

  Serial.print(F("RSSI: "));
  Serial.print(LoRa.packetRssi());
  Serial.println(F(" dBm"));

  Serial.print(F("SNR: "));
  Serial.print(LoRa.packetSnr());
  Serial.println(F(" dB"));

  if (overflow) {
    Serial.println(F("Erro: pacote maior que o buffer do ESP32 ou bytes excedentes no FIFO."));
    LoRa.receive();
    return;
  }

  if (received != (size_t)packetSize) {
    Serial.print(F("Aviso: tamanho informado diferente do tamanho lido. Informado="));
    Serial.print(packetSize);
    Serial.print(F(", lido="));
    Serial.println(received);
  }

  if (received < GS_HEADER_LEN) {
    Serial.println(F("Erro: pacote menor que o cabecalho GS."));
    printHex(buffer, received);
    LoRa.receive();
    return;
  }

  if (buffer[0] != GS_MAGIC0 || buffer[1] != GS_MAGIC1) {
    Serial.println(F("Pacote generico recebido: nao comeca com GS."));
    printHex(buffer, received);

    Serial.println(F("ASCII:"));
    for (size_t i = 0; i < received; i++) {
      const uint8_t c = buffer[i];
      Serial.write((c >= 32 && c <= 126) ? c : '.');
    }
    Serial.println();

    LoRa.receive();
    return;
  }

  const uint8_t version = buffer[2];
  const uint8_t flags = buffer[3];
  const uint32_t txTimestamp = readU32(buffer, 4);

  if (version != GS_VERSION) {
    Serial.print(F("Erro: versao nao suportada: "));
    Serial.println(version);
    LoRa.receive();
    return;
  }

  Serial.println();
  Serial.println(F("----- CABECALHO -----"));
  Serial.print(F("Magic: "));
  Serial.print((char)buffer[0]);
  Serial.println((char)buffer[1]);

  Serial.print(F("Versao: "));
  Serial.println(version);

  Serial.print(F("Flags: 0x"));
  if (flags < 0x10) Serial.print('0');
  Serial.println(flags, HEX);

  Serial.print(F("Timestamp TX: "));
  Serial.print(txTimestamp);
  Serial.println(F(" ms"));

  Serial.print(F("Sensores: "));
  Serial.println((flags & GS_FLAG_SENSOR) ? F("sim") : F("nao"));
  Serial.print(F("Modulo: "));
  Serial.println((flags & GS_FLAG_MODULE) ? F("sim") : F("nao"));
  Serial.print(F("Payload: "));
  Serial.println((flags & GS_FLAG_PAYLOAD) ? F("sim") : F("nao"));
  Serial.print(F("Agua: "));
  Serial.println((flags & GS_FLAG_WATER) ? F("sim") : F("nao"));

  size_t offset = GS_HEADER_LEN;

  // ===== Bloco obrigatório de sensores =====
  if (!(flags & GS_FLAG_SENSOR)) {
    Serial.println(F("Erro: pacote sem o bloco de sensores."));
    LoRa.receive();
    return;
  }

  if (!blockAvailable(received, offset, GS_SENSOR_BLOCK_LEN)) {
    Serial.println(F("Erro: bloco de sensores incompleto."));
    printHex(buffer, received);
    LoRa.receive();
    return;
  }

  const uint32_t sensorTimestamp = readU32(buffer, offset); offset += 4;
  const float accelX = readFloat(buffer, offset); offset += 4;
  const float accelY = readFloat(buffer, offset); offset += 4;
  const float accelZ = readFloat(buffer, offset); offset += 4;
  const float gyroX = readFloat(buffer, offset); offset += 4;
  const float gyroY = readFloat(buffer, offset); offset += 4;
  const float gyroZ = readFloat(buffer, offset); offset += 4;
  const float magX = readFloat(buffer, offset); offset += 4;
  const float magY = readFloat(buffer, offset); offset += 4;
  const float magZ = readFloat(buffer, offset); offset += 4;
  const float temperature = readFloat(buffer, offset); offset += 4;
  const float pressure = readFloat(buffer, offset); offset += 4;
  const float altitude = readFloat(buffer, offset); offset += 4;
  const float battery = readFloat(buffer, offset); offset += 4;

  Serial.println();
  Serial.println(F("----- SENSORES -----"));
  Serial.print(F("Timestamp: ")); Serial.print(sensorTimestamp); Serial.println(F(" ms"));

  Serial.print(F("Accel X: ")); Serial.println(accelX, 4);
  Serial.print(F("Accel Y: ")); Serial.println(accelY, 4);
  Serial.print(F("Accel Z: ")); Serial.println(accelZ, 4);

  Serial.print(F("Gyro X: ")); Serial.println(gyroX, 4);
  Serial.print(F("Gyro Y: ")); Serial.println(gyroY, 4);
  Serial.print(F("Gyro Z: ")); Serial.println(gyroZ, 4);

  Serial.print(F("Mag X: ")); Serial.println(magX, 4);
  Serial.print(F("Mag Y: ")); Serial.println(magY, 4);
  Serial.print(F("Mag Z: ")); Serial.println(magZ, 4);

  Serial.print(F("Temperatura: ")); Serial.print(temperature, 2); Serial.println(F(" C"));
  Serial.print(F("Pressao: ")); Serial.print(pressure, 2); Serial.println(F(" Pa"));
  Serial.print(F("Altitude: ")); Serial.print(altitude, 2); Serial.println(F(" m"));
  Serial.print(F("Bateria: ")); Serial.print(battery, 2); Serial.println(F(" V"));

  // ===== Bloco opcional do módulo =====
  if (flags & GS_FLAG_MODULE) {
    if (!blockAvailable(received, offset, GS_OPTIONAL_LEN)) {
      Serial.println(F("Erro: bloco do modulo incompleto."));
      LoRa.receive();
      return;
    }

    const uint32_t timestamp = readU32(buffer, offset); offset += 4;
    const float latitude = readFloat(buffer, offset); offset += 4;
    const float longitude = readFloat(buffer, offset); offset += 4;
    const uint32_t status = readU32(buffer, offset); offset += 4;

    Serial.println();
    Serial.println(F("----- MODULO LORA -----"));
    Serial.print(F("Timestamp: ")); Serial.print(timestamp); Serial.println(F(" ms"));
    Serial.print(F("Latitude: ")); Serial.println(latitude, 6);
    Serial.print(F("Longitude: ")); Serial.println(longitude, 6);
    Serial.print(F("Status: ")); Serial.println(status ? F("valido") : F("invalido"));
  }

  // ===== Bloco opcional do payload =====
  if (flags & GS_FLAG_PAYLOAD) {
    if (!blockAvailable(received, offset, GS_OPTIONAL_LEN)) {
      Serial.println(F("Erro: bloco do payload incompleto."));
      LoRa.receive();
      return;
    }

    const uint32_t timestamp = readU32(buffer, offset); offset += 4;
    const float latitude = readFloat(buffer, offset); offset += 4;
    const float longitude = readFloat(buffer, offset); offset += 4;
    const uint32_t found = readU32(buffer, offset); offset += 4;

    Serial.println();
    Serial.println(F("----- PAYLOAD -----"));
    Serial.print(F("Timestamp: ")); Serial.print(timestamp); Serial.println(F(" ms"));
    Serial.print(F("Latitude: ")); Serial.println(latitude, 6);
    Serial.print(F("Longitude: ")); Serial.println(longitude, 6);
    Serial.print(F("Encontrado: ")); Serial.println(found ? F("sim") : F("nao"));
  }

  // ===== Bloco opcional de qualidade da água =====
  if (flags & GS_FLAG_WATER) {
    if (!blockAvailable(received, offset, GS_WATER_BLOCK_LEN)) {
      Serial.println(F("Erro: bloco de agua incompleto."));
      LoRa.receive();
      return;
    }

    const uint32_t timestamp = readU32(buffer, offset); offset += 4;
    const float conductivity = readFloat(buffer, offset); offset += 4;
    const float turbidity = readFloat(buffer, offset); offset += 4;
    const float ph = readFloat(buffer, offset); offset += 4;
    const float waterTemperature = readFloat(buffer, offset); offset += 4;
    const uint32_t status = readU32(buffer, offset); offset += 4;

    Serial.println();
    Serial.println(F("----- QUALIDADE DA AGUA -----"));
    Serial.print(F("Timestamp: ")); Serial.print(timestamp); Serial.println(F(" ms"));
    Serial.print(F("Condutividade: ")); Serial.print(conductivity, 2); Serial.println(F(" uS/cm"));
    Serial.print(F("Turbidez: ")); Serial.print(turbidity, 2); Serial.println(F(" NTU"));
    Serial.print(F("pH: ")); Serial.println(ph, 2);
    Serial.print(F("Temperatura da agua: ")); Serial.print(waterTemperature, 2); Serial.println(F(" C"));
    Serial.print(F("Status: ")); Serial.println(status ? F("valido") : F("invalido"));
  }

  Serial.println();
  Serial.print(F("Bytes decodificados: "));
  Serial.println(offset);

  if (offset < received) {
    Serial.print(F("Aviso: sobraram "));
    Serial.print(received - offset);
    Serial.println(F(" bytes."));
    printHex(buffer + offset, received - offset);
  } else if (offset == received) {
    Serial.println(F("Pacote completamente decodificado."));
  } else {
    Serial.println(F("Erro: offset ultrapassou o pacote."));
  }

  Serial.println(F("========================================"));
  LoRa.receive();
}