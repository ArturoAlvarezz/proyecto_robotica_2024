int pwPin = 7; // Pin donde está conectado el PWM del sensor
long pulseWidth; // Ancho del pulso en microsegundos
float distanceInInches;
float distanceInCm;

void setup() {
  Serial.begin(9600);  // Inicializa la comunicación serial
  pinMode(pwPin, INPUT);  // Configura el pin del sensor como entrada
}

void loop() {
  // Lee el ancho del pulso en microsegundos
  pulseWidth = pulseIn(pwPin, HIGH);

  // Convierte el ancho del pulso en distancia (usando 147us/inch)
  distanceInInches = pulseWidth / 147.0;

  // Convierte la distancia a centímetros
  distanceInCm = distanceInInches * 2.54;

  // Imprime la distancia
  Serial.print("Distancia: ");
  Serial.print(distanceInCm);
  Serial.println(" cm");
  

  delay(100);  // Espera 1 segundo antes de tomar otra lectura
}
