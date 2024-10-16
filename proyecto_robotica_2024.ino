// Definición de pines para los sensores ultrasonicos
const int pinSensorFrontal = 9; // Sensor frontal (obstáculo al frente)
const int pinSensorSuelo = 11;  // Sensor del suelo (altura al suelo)
long pulseWidth = 0;

// Umbrales de distancia
const int umbralObstaculo = 200;
const int umbralSuelo = 30;
const int distanciaDeAvance = umbralObstaculo + 50;

void setup() {
  // Inicializar los pines de los sensores
  pinMode(pinSensorFrontal, INPUT);
  pinMode(pinSensorSuelo, INPUT);

  // Inicializar la comunicación serie para monitorear
  Serial.begin(9600);
}

void loop() {
  // Leer las distancias de los sensores
  int distanciaFrontal = medirDistancia(pinSensorFrontal);
  int distanciaSuelo = medirDistancia(pinSensorSuelo);

  // Mostrar las distancias medidas en el monitor serial
  Serial.print("Distancia al obstáculo (frontal): ");
  Serial.print(distanciaFrontal);
  Serial.println(" cm");

  Serial.print("Distancia al suelo: ");
  Serial.print(distanciaSuelo);
  Serial.println(" cm");

  // Comprobar si existe un obstáculo al frente
  if (distanciaFrontal <= umbralObstaculo) {
    detenerDron(); // Si hay un obstáculo al frente, se detiene el dron
    int distanciaSueloInicial = distanciaSuelo; 
    bajarDron();   // Bajar el dron para esquivar el obstáculo

    // Mientras haya un obstáculo al frente y el dron esté a más de 30 cm del suelo
    while (distanciaFrontal <= umbralObstaculo && distanciaSuelo > umbralSuelo) {
      distanciaFrontal = medirDistancia(pinSensorFrontal);
      distanciaSuelo = medirDistancia(pinSensorSuelo);
    }

    // Si ya no hay obstáculos al frente y estamos a una altura segura, avanzamos y subimos
    if (distanciaFrontal > umbralObstaculo && distanciaSuelo > umbralSuelo) {
      avanzarYSubir(distanciaSueloInicial - distanciaSuelo); // Subir la misma distancia que bajamos
    }
  } else {
    continuarVuelo(); // Si no hay obstáculos, continuar el vuelo
  }

  delay(200); // Pequeña pausa para evitar ruido en las lecturas
}

// Función para medir la distancia con un sensor ultrasonico
int medirDistancia(int pinSensor) {
  pulseWidth = pulseIn(pinSensor, HIGH);
  int distanciaEnCM = (pulseWidth / 147.0) * 2.54;

  return distanciaEnCM;
}

// Función para detener el dron
void detenerDron() {
  Serial.println("Deteniendo dron...");
  // Código para detener los motores o el sistema de vuelo
}

// Función para bajar el dron
void bajarDron() {
  Serial.println("Bajando dron...");
  // Código para reducir la altitud del dron
}

// Función para avanzar y subir después de evitar un obstáculo
void avanzarYSubir(int distanciaBajada) {
  Serial.print("Avanzar ");
  Serial.print(distanciaDeAvance);
  Serial.println(" cm y subir ");
  Serial.print(distanciaBajada);
  Serial.println(" cm...");
  // Código para avanzar la distancia y luego subir el dron
}

// Función para continuar el plan de vuelo si no hay obstáculos
void continuarVuelo() {
  Serial.println("Continuar plan de vuelo...");
  // Código para seguir el vuelo normal
}