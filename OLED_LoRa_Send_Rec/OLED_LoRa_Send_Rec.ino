/*
  This is a simple example show the LoRa sended data in OLED.

  The onboard OLED display is SSD1306 driver and I2C interface. In order to make the
  OLED correctly operation, you should output a high-low-high(1-0-1) signal by soft-
  ware to OLED's reset pin, the low-level signal at least 5ms.

  OLED pins to ESP32 GPIOs via this connecthin:
  OLED_SDA -- GPIO4
  OLED_SCL -- GPIO15
  OLED_RST -- GPIO16
  
  by Aaron.Lee from HelTec AutoMation, ChengDu, China
  成都惠利特自动化科技有限公司
  www.heltec.cn
  
  this project also realess in GitHub:
  https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
*/

#include <SPI.h> //responsável pela comunicação serial
#include <LoRa.h> //responsável pela comunicação com o WIFI Lora
#include <Wire.h>  //responsável pela comunicação i2c
#include "SSD1306.h" //responsável pela comunicação com o display
//#include "images.h" //contém o logo para usarmos ao iniciar o display

// Definição dos pinos 
#define SCK     5    // GPIO5  -- SX127x's SCK
#define MISO    19   // GPIO19 -- SX127x's MISO
#define MOSI    27   // GPIO27 -- SX127x's MOSI
#define SS      18   // GPIO18 -- SX127x's CS
#define RST     14   // GPIO14 -- SX127x's RESET
#define DI00    26   // GPIO26 -- SX127x's IRQ(Interrupt Request)

#define BAND    915E6  //Frequencia do radio - podemos utilizar ainda : 433E6, 868E6, 915E6
//#define PABOOST true //nao é mais utilizada

//variável responsável por armazenar o valor do contador (enviaremos esse valor para o outro Lora)
unsigned int counter = 0;

//parametros: address,SDA,SCL 
SSD1306 display(0x3c, 4, 15); //construtor do objeto que controlaremos o display

String rssi = "RSSI --";
String packSize = "--";
String packet ;

//Entradas e Saídas Utilizadas---------------------------------------------------------------------------
const int chave1_pin = 36; //GPIO36-esp32 
const int chave2_pin = 37; //GPIO37-esp32 
const int led1_pin = 12; //GPIO38-esp32 
//-------------------------------------------------------------------------------------------------------

long lastSendTime = 0;        // last send time
int interval = 500;          // interval between sends


void setup()
{
  //Configura pinos para chave e led
  pinMode(chave1_pin, INPUT);
  pinMode(chave2_pin, INPUT);
  pinMode(led1_pin, OUTPUT);

  //configura os pinos como saida
  pinMode(16,OUTPUT); //RST do oled
  pinMode(25,OUTPUT);
  
  digitalWrite(16, LOW);    // reseta o OLED
  delay(50); 
  digitalWrite(16, HIGH); // enquanto o OLED estiver ligado, GPIO16 deve estar HIGH

  display.init(); //inicializa o display
  display.flipScreenVertically(); 
  display.setFont(ArialMT_Plain_10); //configura a fonte para um tamanho maior

  //imprime o logo na tela
//  logo();

  delay(1500);
  display.clear(); //apaga todo o conteúdo da tela do display
  
  SPI.begin(SCK,MISO,MOSI,SS); //inicia a comunicação serial com o Lora
  LoRa.setPins(SS,RST,DI00); //configura os pinos que serão utlizados pela biblioteca (deve ser chamado antes do LoRa.begin)
  
  //inicializa o Lora com a frequencia específica.
  //if (!LoRa.begin(BAND,PABOOST)) //foi modificada para o codigo abaixo:
  if (!LoRa.begin(BAND))
  {
    display.drawString(0, 0, "Starting LoRa failed!");
    display.display(); //mostra o conteúdo na tela
    while (1);
  }
  //indica no display que inicilizou corretamente.
  display.drawString(0, 0, "LoRa Initial success!");
  display.display(); //mostra o conteúdo na tela
  delay(1000);

  
} //fim do setup

void loop()
{
  //apaga o conteúdo do display
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  
  display.drawString(0, 0, "Sending packet: ");
  display.drawString(40, 26, String(counter));
  display.display(); //mostra o conteúdo na tela

  if (millis() - lastSendTime > interval) {
    sendMessage();
    lastSendTime = millis();            // timestamp the message
    interval = random(500) + 500;    // 1-2 seconds 
  }
  
  onReceive(LoRa.parsePacket());
}

void sendMessage(){
   //beginPacket : abre um pacote para adicionarmos os dados para envio
  LoRa.beginPacket();
  //print: adiciona os dados no pacote
  LoRa.print( digitalRead(chave1_pin) );
  LoRa.print("|");
  LoRa.print( digitalRead(chave2_pin) );
  //LoRa.print(counter);
  LoRa.print("|");
  LoRa.print( digitalRead(led1_pin) );
  LoRa.print("|");
  LoRa.print(counter);
  LoRa.print("|");
  //endPacket : fecha o pacote e envia
  LoRa.endPacket(); //retorno= 1:sucesso | 0: falha

  //incrementa uma unidade no contador
  counter++;
}


void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int comandoLed1 = LoRa.read();          
  digitalWrite(led1_pin, comandoLed1);  
}

//essa função apenas imprime o logo na tela do display
//void logo()
//{
  //apaga o conteúdo do display
//  display.clear();
  //imprime o logo presente na biblioteca "images.h"
//  display.drawXbm(0,5,logo_width,logo_height,logo_bits);
//  display.display();
//}
