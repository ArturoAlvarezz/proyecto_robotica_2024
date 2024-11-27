#include <MAVLink.h>

uint8_t mavlinkBuf[2]; // Ajusta el tamaño según sea necesario

void setup() {
  Serial.begin(57600);  // Configuración de MAVLink
  delay(10000);

  // Inicializar los pines de los sensores
  pinMode(9, INPUT);
  pinMode(11, INPUT);
}

void loop() {
  // Comprobar si existe un obstáculo al frente
  if (medirDistancia(9) <= 300) {
    detenerDron(); // Si hay un obstáculo al frente, se detiene el dron
    bajarDron();   // Bajar el dron para esquivar el obstáculo
    moveForward(); // Avanzar la distancia especificada
    subirDron(getCurrentAltitude()); // Subir la misma distancia que bajamos
  } else {
    // continuarPlanDeVuelo(); // Si no hay obstáculos, continuar el vuelo
  }

  delay(200); // Pequeña pausa para evitar ruido en las lecturas
}

// Función para medir la distancia con un sensor ultrasonico
short medirDistancia(short pinSensor) {
  return (pulseIn(pinSensor, HIGH) / 147.0) * 2.54;
}

// Función para procesar mensajes MAVLink y obtener la altitud
short getCurrentAltitude() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial.available()) {
    // Si el mensaje MAVLink está completo
    if (mavlink_parse_char(MAVLINK_COMM_0, Serial.read(), &msg, &status)) {
      // Si el mensaje es de posición global (GPS)
      if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t gpsData;
        mavlink_msg_global_position_int_decode(&msg, &gpsData);

        // convertimos de milímetros a centímetros
        return gpsData.relative_alt / 10;
      }
    }
  }
  return 0; // Valor por defecto si no se recibe el mensaje
}

// Función para cambiar el modo del dron a "Loiter" o "Brake" vía MAVLink
void detenerDron() {
  mavlink_message_t msg;

  // Comando de cambio de modo a "Loiter" o "Brake"
  mavlink_msg_command_long_pack(
    1,    // ID del sistema (puede ser 1 para el dron)
    200,  // ID del componente (normalmente 200 para el controlador de vuelo)
    &msg,
    1,    // Target system
    0,    // Target component
    MAV_CMD_DO_SET_MODE, // Comando para cambiar el modo
    0,    // Confirmation
    0,    // Modo base (0 para GUIDED si es ArduPilot)
    4,    // Modo personalizado (4 para LOITER en ArduPilot, verifica si es tu caso)
    0, 0, 0, 0, 0  // Parámetros adicionales no requeridos
  );
  Serial.write(mavlinkBuf, mavlink_msg_to_send_buffer(mavlinkBuf, &msg));
}

// Función para bajar el dron hasta que no haya nada enfrente
void bajarDron() {
  while (medirDistancia(9) < 550) {
    gradualDescent(); // Baja el dron en 15 cm
    delay(1000); // Esperar un segundo entre cada descenso
  }
}

// Función para descenso gradual
void gradualDescent() {
  mavlink_message_t msg;

  // Suponiendo que tienes la altitud actual del dron en una variable
  short targetAltitude = getCurrentAltitude() - 15;  // Bajar 15 cm

  // Evitar altitudes negativas o muy cercanas al suelo
  if ( medirDistancia(11) < 30) {
    targetAltitude = (getCurrentAltitude() - 20) * 10; // Bajar 20 cm
    mavlink_msg_command_long_pack(
      1,     // ID del sistema (por lo general 1 para el dron)
      200,   // ID del componente (normalmente 200 para el controlador de vuelo)
      &msg,
      1,     // Target system (ID del dron)
      0,     // Target component (ID del componente)
      MAV_CMD_NAV_WAYPOINT,  // Comando para moverse a una posición específica (cambiamos solo la altitud)
      0,     // Confirmation
      0, 0, targetAltitude, 0, 0, 0, 0  // Solo configuramos la altitud en el tercer parámetro
    );
    uint16_t len = mavlink_msg_to_send_buffer(mavlinkBuf, &msg);
    Serial.write(mavlinkBuf, len);
  }

  // Transformar la altitud a milímetros
  targetAltitude *= 10;

  // Comando para cambiar la altitud objetivo
  mavlink_msg_command_long_pack(
    1,     // ID del sistema (por lo general 1 para el dron)
    200,   // ID del componente (normalmente 200 para el controlador de vuelo)
    &msg,
    1,     // Target system (ID del dron)
    0,     // Target component (ID del componente)
    MAV_CMD_NAV_WAYPOINT,  // Comando para moverse a una posición específica (cambiamos solo la altitud)
    0,     // Confirmation
    0, 0, targetAltitude, 0, 0, 0, 0  // Solo configuramos la altitud en el tercer parámetro
  );

  uint16_t len = mavlink_msg_to_send_buffer(mavlinkBuf, &msg);
  Serial.write(mavlinkBuf, len);
}

// Función para avanzar una distancia específica en metros
void moveForward() {
  mavlink_message_t msg;

  // Comando para enviar la posición objetivo
  mavlink_msg_set_position_target_local_ned_pack(
    1,     // ID del sistema (puede ser 1 para el dron)
    200,   // ID del componente (normalmente 200 para el controlador de vuelo)
    &msg,
    0,     // Time boot ms (0 para inmediato)
    1,     // Target system
    0,     // Target component
    MAV_FRAME_LOCAL_NED,  // Referencia local NED
    0b111111000111,             // Máscara de tipo para solo posición
    3500, 0, -0,  // Posiciones X, Y, Z
    0, 0, 0,               // Velocidades ignoradas
    0, 0, 0,               // Aceleraciones ignoradas
    0, 0                    // Yaw y velocidad de yaw ignoradas
  );

  // Convertir el mensaje MAVLink en un buffer y enviarlo
  uint16_t len = mavlink_msg_to_send_buffer(mavlinkBuf, &msg);
  Serial.write(mavlinkBuf, len);
}

// Función para subir el dron una distancia específica en metros
void subirDron(short alturaInicial) {
  mavlink_message_t msg;

  // Comando para enviar la posición objetivo
  mavlink_msg_set_position_target_local_ned_pack(
    1,     // ID del sistema (puede ser 1 para el dron)
    200,   // ID del componente (normalmente 200 para el controlador de vuelo)
    &msg,
    0,     // Time boot ms (0 para inmediato)
    1,     // Target system
    0,     // Target component
    MAV_FRAME_LOCAL_NED,  // Referencia local NED
    0b111111000111,             // Máscara de tipo para solo posición
    3500, 0, -0,  // Posiciones X, Y, Z
    0, 0, 0,               // Velocidades ignoradas
    0, 0, 0,               // Aceleraciones ignoradas
    0, 0                    // Yaw y velocidad de yaw ignoradas
  );

  // Convertir el mensaje MAVLink en un buffer y enviarlo
  uint16_t len = mavlink_msg_to_send_buffer(mavlinkBuf, &msg);
  Serial.write(mavlinkBuf, len);
}

// Función para continuar el plan de vuelo
void continuarPlanDeVuelo() {
    mavlink_message_t msg;

    // Cambiar al modo automático para retomar el plan de vuelo
    mavlink_msg_command_long_pack(
        1,                         // ID del sistema
        MAV_COMP_ID_SYSTEM_CONTROL,// ID del componente
        &msg,
        1,                         // Destino: ID del sistema
        MAV_COMP_ID_AUTOPILOT1,    // Componente
        MAV_CMD_DO_SET_MODE,       // Comando para establecer modo de vuelo
        0,                         // Confirmación
        4,                         // Modo automático (4 para AUTO)
        0, 0, 0, 0, 0, 0          // Otros parámetros no usados
    );

    uint16_t len = mavlink_msg_to_send_buffer(mavlinkBuf, &msg);
    Serial.write(mavlinkBuf, len);
}
