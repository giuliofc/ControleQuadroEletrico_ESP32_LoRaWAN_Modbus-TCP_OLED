/*
  This is a simple example show the LoRa recived data in OLED.

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

//Para Lora e Display------------------------------------------------------------------------------------------------------------
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
//#define PABOOST true

//parametros: address,SDA,SCL 
SSD1306 display(0x3c, 4, 15); //construtor do objeto que controlaremos o display

String rssi = "RSSI --";  //indicador de sinal recebido 
String packSize = "--";   
String packet ;


//TESTE*********************************
String pacote[4];
String pacote0;
int posicaoDaBarra0=0;
int posicaoDaBarra[4];
int posicaoDaBarraAnt=0;
//**************************************

unsigned long tempoAux=millis(); //para subistituir o delay no loop do Lora
unsigned long tempoAux2=millis(); 

long lastSendTime = 0;        // last send time
int interval = 500;          // interval between sends
//---------------------------------------------------------------------------------------------------------------------------------

//Para Modbus----------------------------------------------------------------------------------------------------------------------
#include <Modbus.h>
#include <ModbusIP_ESP32.h>

IPAddress ip;   
String sIP="";

//ModbusIP object
ModbusIP mb;
//---------------------------------------------------------------------------------------------------------------------------------

//Entradas e Saídas Utilizadas-----------------------------------------------------------------------------------------------------
const int chave1Pin_ETA = 36; //GPIO36-esp32 
const int chave2Pin_ETA = 37; //GPIO37-esp32 
const int led1Pin_ETA = 12; //GPIO38-esp32 

//ETA - Modbus Registers Offsets (0-9999)
const int TEST_HREG = 100; 
const int CHAVE1_ISTS_ETA= 101;  //mb=modbus
const int CHAVE2_ISTS_ETA = 102;
const int LED1_COIL_ETA = 103;

//POÇO - Modbus Registers Offsets
const int CHAVE1_ISTS_POCO= 104;  
const int CHAVE2_ISTS_POCO = 105;
const int LED1_COIL_POCO = 106;
const int LED1_ISTS_POCO = 107; 

//Variavel auxiliar comando Poco 
int comando_Led1_Poco;
//---------------------------------------------------------------------------------------------------------------------------------


void setup() {

//Para Modbus-----------------------------------------------------------------------------------------------------------------------
  Serial.begin(115200);

  //Conf. Roteador
  mb.config("TP-LINK_D9E4D6", "38065965");

  //Conf. Celular
 // mb.config("cel_marcia", "senhamarcia");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  ip = WiFi.localIP();
  for (int i=0; i<4; i++) {
    sIP += i  ? "." + String(ip[i]) : String(ip[i]);
  }
  
  //mb.addHreg(TEST_HREG, 0xABCD);
  mb.addHreg(TEST_HREG);

  //ETA - Config pinos e registros 
  pinMode(chave1Pin_ETA, INPUT);
  mb.addIsts(CHAVE1_ISTS_ETA);
  
  pinMode(chave2Pin_ETA, INPUT);
  mb.addIsts(CHAVE2_ISTS_ETA);

  pinMode(led1Pin_ETA, OUTPUT);
  mb.addCoil(LED1_COIL_ETA);

  //POCO - Config pinos e registros 
   mb.addIsts(CHAVE1_ISTS_POCO);
   mb.addIsts(CHAVE2_ISTS_POCO);
   mb.addCoil(LED1_COIL_POCO);
   mb.addIsts(LED1_ISTS_POCO);
   
 //---------------------------------------------------------------------------------------------------------------------------------
 

//Para Lora e Display ----------------------------------------------------------------------------------------------------------
 //configura os pinos como saida
  pinMode(16,OUTPUT); //RST do oled
  digitalWrite(16, LOW);    // reseta o OLED
  delay(50); 
  digitalWrite(16, HIGH); // enquanto o OLED estiver ligado, GPIO16 deve estar HIGH
  display.init();
  display.flipScreenVertically();  
  display.setFont(ArialMT_Plain_10);
//  logo();
  delay(1500);
  display.clear();

  SPI.begin(SCK,MISO,MOSI,SS); //inicia a comunicação serial com o Lora
  LoRa.setPins(SS,RST,DI00); //configura os pinos que serão utlizados pela biblioteca (deve ser chamado antes do LoRa.begin)
  
  //inicializa o Lora com a frequencia específica.
  if (!LoRa.begin(BAND)) {
    display.drawString(0, 0, "Starting LoRa failed!");
    display.display();
    while (1);
  }

  //indica no display que inicilizou corretamente.
  display.drawString(0, 0, "LoRa Initial success!");
  display.drawString(0, 10, "Wait for incomm data...");
  display.display();
  delay(1000);

  //LoRa.onReceive(cbk);
  LoRa.receive(); //habilita o Lora para receber dados
//----------------------------------------------------------------------------------------------------------------------------------
 
} //fim do void setup 


void loop() {

  mb.task();
   
  onReceive();

  leituraEscritaModbus();

  if (millis() - lastSendTime > interval) {
    sendMessage();
    lastSendTime = millis();         // timestamp the message
    interval = random(500) + 500;    // 0,5 a 1 seconds 
  }
  

} //fim do void loop



void onReceive(){  
  //Para Lora e Display----------------------------------------------------------------------------------------------------------------
  if( tempoAux<=millis() ){  //para substituir o delay 
      tempoAux=millis()+10;
      //parsePacket: checa se um pacote foi recebido
      //retorno: tamanho do pacote em bytes. Se retornar 0 (ZERO) nenhum pacote foi recebido
      int packetSize = LoRa.parsePacket();
      //caso tenha recebido pacote chama a função para configurar os dados que serão mostrados em tela
      if (packetSize) { 
        cbk(packetSize);  
      }
   }  
}


void sendMessage(){
  LoRa.beginPacket();                   // start packet
  LoRa.write(comando_Led1_Poco);        
  LoRa.endPacket();                     // finish packet and send it
}


void leituraEscritaModbus(){
    if( tempoAux2<=millis() ){  //para substituir o delay 
       tempoAux2=millis()+30;
       //mb.Hreg(TEST_HREG, 12345);
  
       //Leitura escrita ETA
       mb.Ists(CHAVE1_ISTS_ETA, digitalRead(chave1Pin_ETA)); //le o pino e escreve no modbus
       mb.Ists(CHAVE2_ISTS_ETA, digitalRead(chave2Pin_ETA));
       digitalWrite(led1Pin_ETA, mb.Coil(LED1_COIL_ETA));
    
       //Leitura escrita POCO
       mb.Ists(CHAVE1_ISTS_POCO, pacote0.toInt() );
       mb.Ists(CHAVE2_ISTS_POCO, pacote[1].toInt() );
       mb.Ists(LED1_ISTS_POCO, pacote[2].toInt() );
       comando_Led1_Poco = mb.Coil(LED1_COIL_POCO);
    }  
}


//função responsável por recuperar o conteúdo do pacote recebido
//parametro: tamanho do pacote (bytes)
void cbk(int packetSize) {
  packet ="";
  packSize = String(packetSize,DEC); //transforma o tamanho do pacote em String para imprimirmos
  for (int i = 0; i < packetSize; i++) { 
    packet += (char) LoRa.read(); //recupera o dado recebido e concatena na variável "packet"
  }
  rssi = "RSSI=  " + String(LoRa.packetRssi(), DEC)+ "dB"; //configura a String de Intensidade de Sinal (RSSI)
  //mostrar dados em tela


  //TESTE**********************************************************
  
  posicaoDaBarra0=packet.indexOf("|");
  pacote0 = packet.substring(0 , posicaoDaBarra0);
  posicaoDaBarraAnt=posicaoDaBarra0;
  
  for(int i=1; i<4; i++){
  posicaoDaBarra[i]=packet.indexOf("|", posicaoDaBarraAnt+1); 
  pacote[i] = packet.substring(posicaoDaBarraAnt+1, posicaoDaBarra[i]);
  posicaoDaBarraAnt=posicaoDaBarra[i];
  }
  
  Serial.print(pacote0); 
  Serial.print(" - ");
  Serial.print(pacote[1]);
  Serial.print(" - ");
  Serial.print(pacote[2]);
  Serial.print(" - ");
  Serial.println(pacote[3]);
 
  loraData();
}

//função responsável por configurar os dadosque serão exibidos em tela.
//RSSI : primeira linha
//RX packSize : segunda linha
//packet : terceira linha
void loraData(){
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
 // display.drawString(0 , 18 , "Rx "+ packSize + " bytes");
  display.drawString(0, 18, sIP);
 
  display.drawStringMaxWidth(0 , 39 , 128, packet);
  display.drawString(0, 0, rssi);  
  display.display();
}

//Para Lora e Display----------------------------------------------------------------------------------------------------------------
//essa função apenas imprime o logo na tela do display
/*void logo()
{
  //apaga o conteúdo do display
  display.clear();
  //imprime o logo presente na biblioteca "images.h"
  display.drawXbm(0,5,logo_width,logo_height,logo_bits);
  display.display();
} */
