/*
  Codigo para Arduino Micro (o Nano) 
  - Espera recibir por Serial (115200 baudios) un caracter:
       '1' => Enciende LED en pin D6 y LED_BUILTIN
       '0' => Apaga LED en pin D6 y LED_BUILTIN
*/

const int pinLed = 6; // pin D6
void setup() {
  // Inicializar puerto serial a 115200 (ajusta igual que en PC)
  Serial.begin(115200);
  
  // Configurar pin D6 y LED_BUILTIN como salida
  pinMode(pinLed, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Apagar ambos de inicio
  digitalWrite(pinLed, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  for(int i=0; i<3; i++){
    digitalWrite(LED_BUILTIN, HIGH); 
    digitalWrite(pinLed, HIGH);
    delay(1000);
    digitalWrite(pinLed, LOW);
    digitalWrite(LED_BUILTIN, LOW); 
    delay(1000);
  }
}

void loop() {

  //String command = Serial.readStringUntil('\n');
  //command.trim(); // Limpia espacios en blanco y caracteres de nueva línea


  // Si hay datos disponibles en Serial
  if (Serial.available() > 0) {
    char cmd = Serial.read();  // lee un caracter
    if (cmd == '1') {
      // Enciende
      digitalWrite(pinLed, HIGH);
      digitalWrite(LED_BUILTIN, HIGH);
      // (Opcional) mandar mensaje de confirmación
      // Serial.println("LED Encendido");
    }
    else if (cmd == '0') {
      // Apaga
      digitalWrite(pinLed, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      // (Opcional) mandar mensaje de confirmación
      // Serial.println("LED Apagado");
    }
    // otros comandos si lo deseas ...
  }

  // No hace nada más, espera
}
