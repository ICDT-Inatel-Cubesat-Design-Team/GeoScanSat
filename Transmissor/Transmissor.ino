#include <SPI.h>
#include <LoRa.h>

#define LORA_SS    5
#define LORA_RST   16
#define LORA_DIO0  4

void setup()
{
  Serial.begin(115200);

  LoRa.setPins(
    LORA_SS,
    LORA_RST,
    LORA_DIO0
  );

  if (!LoRa.begin(915E6))
  {
    Serial.println("Erro ao iniciar LoRa");

    while (true)
    {
      delay(1000);
    }
  }

  LoRa.setSignalBandwidth(125E3);
  LoRa.setSpreadingFactor(7);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x4C);

  // Precisa ser igual no receptor
  LoRa.disableCrc();

  Serial.println("Transmissor pronto");
}

void loop()
{
  // Substituir posteriormente pelas leituras reais
  const float condutividade = 1250.50f;
  const float turbidez      = 18.75f;
  const float ph            = 7.25f;
  const float temperatura   = 24.80f;
  const int status          = 1;

  char mensagem[150];

  snprintf(
    mensagem,
    sizeof(mensagem),
    "{\"cond\":%.2f,\"turb\":%.2f,"
    "\"ph\":%.2f,\"temp\":%.2f,\"s\":%d}",
    condutividade,
    turbidez,
    ph,
    temperatura,
    status
  );

  LoRa.beginPacket();
  LoRa.print(mensagem);

  const int resultado = LoRa.endPacket();

  Serial.print("Enviado: ");
  Serial.println(mensagem);

  if (resultado != 1)
  {
    Serial.println("Falha no envio");
  }

  delay(10000);
}