#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <TinyGPS.h>

// Configuración del pinES
#define MQ135_PIN 34
Adafruit_BME280 bme;
BH1750 lightMeter;
TinyGPS gps;

HardwareSerial GPS_Serial(1); // Usamos UART1


#define LED1 15  // Definimos el pin del LED1
#define LED2 2   // Definimos el pin del LED2
#define LED3 4   // Definimos el pin del LED3
#define LED4 5   // Definimos el pin del LED4


// Configuración de WiFi   TEC IOT*****************************************
const char *red = "Tec-IoT";
const char *password = "spotless.magnetic.bridge";

// Configuración de WiFi   CASA*****************************************
// const char *red = "INFINITUM9487_2.4";
// const char *password = "7149946209";


//*********************************************************  TEC IOT *********************************************************************
//URL base del servidor (sin los parámetros dinámicos)
String urlBaseco2= "http://10.48.101.19/prueba/co2tomysql.php?idSensor=12345MQ135";
String urlBasetemp= "http://10.48.101.19/prueba/temptomysql.php?idSensor=12345BME280";
String urlBasehum= "http://10.48.101.19/prueba/humedadtomysql.php?idSensor=54321BME280";
String urlBasepresion= "http://10.48.101.19/prueba/presiontomysql.php?idSensor=67890BME280";
String urlBaseluz= "http://10.48.101.19/prueba/intensidadLuztomysql.php?idSensor=12345BH1750";
String urlBaselatitud= "http://10.48.101.19/prueba/latitudtomysql.php?idSensor=12345GPSNEO6M";
String urlBaselongitud= "http://10.48.101.19/prueba/longitudtomysql.php?idSensor=67890GPSNEO6M";
String urlBasealerta= "http://10.48.101.19/prueba/alertatomysql.php";



//*********************************************************  CASA *********************************************************************
// String urlBaseco2= "http://192.168.1.227/prueba/co2tomysql.php?idSensor=12345MQ135";
// String urlBasetemp= "http://192.168.1.227/prueba/temptomysql.php?idSensor=12345BME280";
// String urlBasehum= "http://192.168.1.227/prueba/humedadtomysql.php?idSensor=54321BME280";
// String urlBasepresion= "http://192.168.1.227/prueba/presiontomysql.php?idSensor=67890BME280";
// String urlBaseluz= "http://192.168.1.227/prueba/intensidadLuztomysql.php?idSensor=12345BH1750";
// String urlBaselatitud= "http://192.168.1.227/prueba/latitudtomysql.php?idSensor=12345GPSNEO6M";
// String urlBaselongitud= "http://192.168.1.227/prueba/longitudtomysql.php?idSensor=67890GPSNEO6M";
// String urlBasealerta= "http://192.168.1.227/prueba/alertatomysql.php";



HTTPClient http;
WiFiClient clienteWiFi;

void setup() {
  abrirSerial();
  conectarRed();
  Wire.begin(21, 22);  // SDA = 21, SCL = 22
  pinMode(MQ135_PIN, INPUT);
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17); // Configura UART1 en pines 13 (RX) y 14 (TX)

  // Iniciar el sensor BME280
  if (!bme.begin(0x76)) { 
    Serial.println("No se encontró el sensor BME280");
    while (1);
  }

  // Iniciar el sensor GY-302 (BH1750)
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("No se encontró el sensor GY-302");
    while (1);
  }
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
}

void loop() {
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  // Leer valores de los sensores
  // Leer valores del BME280
  float temperatura = bme.readTemperature();
  float humedad = bme.readHumidity();
  float presion = bme.readPressure() / 100.0F; 

  // Leer la intensidad lumínica del GY-302
  float lux = lightMeter.readLightLevel();

  // LECTURAS DEL MQ135
  int CO2 = analogRead(MQ135_PIN);

  // Variables para almacenar datos GPS
  float flat, flon;
  unsigned long age;

  // Procesar los datos del GPS
  while (GPS_Serial.available() > 0) {
    int c = GPS_Serial.read();
    if (gps.encode(c)) {  // Procesar carácter por carácter
      gps.f_get_position(&flat, &flon, &age);
    }
  }

  // Verificar si los valores son válidos
  if (isnan(temperatura) || isnan(humedad) || isnan(presion) || 
      isnan(lux) || isnan(flat) || isnan(flon)) {
    Serial.println("Error al leer uno o más sensores");
    delay(5000);
    return;
  }

  // Construir la URL con los valores dinámicos de los sensores
  String urlco2 = urlBaseco2 + "&valor=" + String(CO2) + "&unidad=CO2";

  String urltemp = urlBasetemp + "&valor=" + String(temperatura) + "&unidad=°C";
  String urlhum = urlBasehum + "&valor=" + String(humedad) + "&unidad=%";
  String urlpresion = urlBasepresion + "&valor=" + String(presion) + "&unidad=hPa";
  String urlluz = urlBaseluz + "&valor=" + String(lux) + "&unidad=lux";
  String urllat = urlBaselatitud + "&valor=" + String(19.595346) + "&unidad=°";
  String urllong = urlBaselongitud + "&valor=" + String(-99.2276) + "&unidad=°";



  //SERIE DE URL'S GENERADAS A PHP
  Serial.println("URL generada CO2: " + urlco2);
  Serial.println("URL generada  TEMPERATURA: " + urltemp);
  Serial.println("URL generada  HUMEDAD: " + urlhum);
  Serial.println("URL generada  PRESION: " + urlpresion);
  Serial.println("URL generada  LUZ: " + urlluz);
  Serial.println("URL generada  LATITUD: " + urllat);
  Serial.println("URL generada  LONG: " + urllong);


  // Realizar la petición HTTP A CO2
  // if (http.begin(clienteWiFi, urlco2)) { 
  //   int codigo = http.GET();  // Hace la petición y obtiene el código de respuesta
  //   Serial.printf("Código HTTP: %d\n", codigo);
  //   if (codigo > 0) {
  //     Serial.println("Respuesta del servidor: " + http.getString());
  //   } else {
  //     Serial.println("Error al realizar la petición co2.");
  //   }
  // }

  // Realizar la petición HTTP A TEMPERATURA
  // if (http.begin(clienteWiFi, urltemp)) { 
  //   int codigo = http.GET();  // Hace la petición y obtiene el código de respuesta
  //   Serial.printf("Código HTTP: %d\n", codigo);
  //   if (codigo > 0) {
  //     Serial.println("Respuesta del servidor: " + http.getString());
  //   } else {
  //     Serial.println("Error al realizar la petición temperatura.");
  //   }
  // }

  // Realizar la petición HTTP A HUMEDAD
  // if (http.begin(clienteWiFi, urlhum)) { 
  //   int codigo = http.GET();  // Hace la petición y obtiene el código de respuesta
  //   Serial.printf("Código HTTP: %d\n", codigo);
  //   if (codigo > 0) {
  //     Serial.println("Respuesta del servidor: " + http.getString());
  //   } else {
  //     Serial.println("Error al realizar la petición humedad.");
  //   }
  // }

  // Realizar la petición HTTP A PRESION
  // if (http.begin(clienteWiFi, urlpresion)) { 
  //   int codigo = http.GET();  // Hace la petición y obtiene el código de respuesta
  //   Serial.printf("Código HTTP: %d\n", codigo);
  //   if (codigo > 0) {
  //     Serial.println("Respuesta del servidor: " + http.getString());
  //   } else {
  //     Serial.println("Error al realizar la petición presion.");
  //   }
  // }

  // Realizar la petición HTTP A INTENSIDADLUZ
  // if (http.begin(clienteWiFi, urlluz)) { 
  //   int codigo = http.GET();  // Hace la petición y obtiene el código de respuesta
  //   Serial.printf("Código HTTP: %d\n", codigo);
  //   if (codigo > 0) {
  //     Serial.println("Respuesta del servidor: " + http.getString());
  //   } else {
  //     Serial.println("Error al realizar la petición luz.");
  //   }
  // }


  // CASOS DE ALERTAS PARA CADA SENSOR ***********************************************************************************************************************************

// PARA CO2
  if (CO2 >= 1900){
    String urlalerta = urlBasealerta + "?idSensor=12345MQ135" + "&valor=" + String(CO2) + "&unidad=CO2";
    if (http.begin(clienteWiFi, urlalerta)) { 
      int codigo = http.GET();  // Hace la petición y obtiene el código de respuesta
      Serial.printf("Código HTTP: %d\n", codigo);
      if (codigo > 0) {
        Serial.println("Respuesta del servidor: " + http.getString());
    } else {
      Serial.println("Error al realizar la petición alerta co2.");
    }
  }
  } else{

    if (http.begin(clienteWiFi, urlco2)) {
      int codigo = http.GET();  // Hace la petición y obtiene el código de respuesta
      Serial.printf("Código HTTP: %d\n", codigo);
      if (codigo > 0) {
        Serial.println("Respuesta del servidor: " + http.getString());
    } else {
      Serial.println("Error al realizar la petición co2.");
    }
  }
  }





// PARA TEMPERATURA
  if (temperatura >= 30){
    String urlalerta = urlBasealerta + "?idSensor=12345BME280" + "&valor=" + String(temperatura) + "&unidad=°C";
    if (http.begin(clienteWiFi, urlalerta)) { 
      int codigo = http.GET();  // Hace la petición y obtiene el código de respuesta
      Serial.printf("Código HTTP: %d\n", codigo);
      if (codigo > 0) {
        Serial.println("Respuesta del servidor: " + http.getString());
    } else {
      Serial.println("Error al realizar la petición alerta temperatura.");
    }
  }
  } else{
    if (http.begin(clienteWiFi, urltemp)) {
      int codigo = http.GET();  // Hace la petición y obtiene el código de respuesta
      Serial.printf("Código HTTP: %d\n", codigo);
      if (codigo > 0) {
        Serial.println("Respuesta del servidor: " + http.getString());
    } else {
      Serial.println("Error al realizar la petición temperatura.");
    }
  }
  }








// PARA HUMEDAD 
  if (humedad >= 45){
    String urlalerta = urlBasealerta + "?idSensor=54321BME280" + "&valor=" + String(humedad) + "&unidad=%";
    if (http.begin(clienteWiFi, urlalerta)) { 
      int codigo = http.GET();  // Hace la petición y obtiene el código de respuesta
      Serial.printf("Código HTTP: %d\n", codigo);
      if (codigo > 0) {
        Serial.println("Respuesta del servidor: " + http.getString());
    } else {
      Serial.println("Error al realizar la petición alerta humedad.");
    }
  }
  } else{
    if (http.begin(clienteWiFi, urlhum)) {
      int codigo = http.GET();  // Hace la petición y obtiene el código de respuesta
      Serial.printf("Código HTTP: %d\n", codigo);
      if (codigo > 0) {
        Serial.println("Respuesta del servidor: " + http.getString());
    } else {
      Serial.println("Error al realizar la petición humedad.");
    }
  }
  }









// PARA PRESION 
  if (presion >= 800){
    String urlalerta = urlBasealerta + "?idSensor=67890BME280" + "&valor=" + String(presion) + "&unidad=hPa";
    if (http.begin(clienteWiFi, urlalerta)) { 
      int codigo = http.GET();  // Hace la petición y obtiene el código de respuesta
      Serial.printf("Código HTTP: %d\n", codigo);
      if (codigo > 0) {
        Serial.println("Respuesta del servidor: " + http.getString());
    } else {
      Serial.println("Error al realizar la petición alerta presion.");
    }
  }
  } else{
    if (http.begin(clienteWiFi, urlpresion)) {
      int codigo = http.GET();  // Hace la petición y obtiene el código de respuesta
      Serial.printf("Código HTTP: %d\n", codigo);
      if (codigo > 0) {
        Serial.println("Respuesta del servidor: " + http.getString());
    } else {
      Serial.println("Error al realizar la petición presion.");
    }
  }
  }










// PARA INTENSIDAD DE LUZ
  if (lux >= 2000){
    String urlalerta = urlBasealerta + "?idSensor=12345BH1750" + "&valor=" + String(lux) + "&unidad=lux";
    if (http.begin(clienteWiFi, urlalerta)) { 
      int codigo = http.GET();  // Hace la petición y obtiene el código de respuesta
      Serial.printf("Código HTTP: %d\n", codigo);
      if (codigo > 0) {
        Serial.println("Respuesta del servidor: " + http.getString());
    } else {
      Serial.println("Error al realizar la petición alerta luz.");
    }
  }
  } else{

    if (http.begin(clienteWiFi, urlluz)) {
      int codigo = http.GET();  // Hace la petición y obtiene el código de respuesta
      Serial.printf("Código HTTP: %d\n", codigo);
      if (codigo > 0) {
        Serial.println("Respuesta del servidor: " + http.getString());
    } else {
      Serial.println("Error al realizar la petición luz.");
    }
  }
  }









  // Realizar la petición HTTP A LATITUD
  if (http.begin(clienteWiFi, urllat)) { 
    int codigo = http.GET();  // Hace la petición y obtiene el código de respuesta
    Serial.printf("Código HTTP: %d\n", codigo);
    if (codigo > 0) {
      Serial.println("Respuesta del servidor: " + http.getString());
    } else {
      Serial.println("Error al realizar la petición latitud.");
    }
  }




  // Realizar la petición HTTP A LONGITUD
  if (http.begin(clienteWiFi, urllong)) { 
    int codigo = http.GET();  // Hace la petición y obtiene el código de respuesta
    Serial.printf("Código HTTP: %d\n", codigo);
    if (codigo > 0) {
      Serial.println("Respuesta del servidor: " + http.getString());
    } else {
      Serial.println("Error al realizar la petición longitud.");
    }
  }











  http.end();
  delay(10000);  // Esperar 10 segundos antes de la siguiente lectura
    digitalWrite(LED1, HIGH);
    delay(100);
    digitalWrite(LED2, HIGH);
    delay(100);
    digitalWrite(LED3, HIGH);
    delay(100);
    digitalWrite(LED4, HIGH);

    // Mantén los LEDs encendidos por 500 ms
    delay(750);

    digitalWrite(LED1, LOW);
    delay(100);
    digitalWrite(LED2, LOW);
    delay(100);
    digitalWrite(LED3, LOW);
    delay(100);
    digitalWrite(LED4, LOW);
  delay(1500);
}

// Configuración inicial del monitor serial
void abrirSerial() {
  Serial.begin(115200);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, HIGH);
  digitalWrite(LED4, HIGH);
  delay(1000);
  Serial.println("Monitor serial listo..\n");
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  delay(1000);
}

// Conexión a la red WiFi
void conectarRed() {
  Serial.println("\nConectando...");
  WiFi.begin(red, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado a la red!");
}
