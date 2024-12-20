#include <MAVLink.h>

uint8_t mavlinkBuf[MAVLINK_MAX_PACKET_LEN]; 


void setup() {
  Serial.begin(57600);  // Configuración de MAVLink
  delay(30000);

  // Inicializar los pines de los sensores
  pinMode(9, INPUT);
  pinMode(11, INPUT);
}

void loop() {
  setModeGuided();
  moverArriba(4.0);
  delay(1000);
}

// Función para medir la distancia con un sensor ultrasonico
short medirDistancia(short pinSensor) {
  long pulseWidth = pulseIn(pinSensor, HIGH);
  short distanciaEnCM = (pulseWidth / 147.0) * 2.54;

  return distanciaEnCM;
}

// Función para procesar mensajes MAVLink y obtener la altitud
short getCurrentAltitude() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial.available()) {
    uint8_t c = Serial.read();

    // Si el mensaje MAVLink está completo
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Si el mensaje es de posición global (GPS)
      if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t gpsData;
        mavlink_msg_global_position_int_decode(&msg, &gpsData);

        // convertimos de milímetros a centímetros
        return gpsData.relative_alt / 10;
      }
    }
  }
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

// Función para descender el dron en pequeños intervalos
void gradualDescent() {
  mavlink_message_t msg;

  // Suponiendo que tienes la altitud actual del dron en una variable
  short currentAltitude = getCurrentAltitude(); // Altitud actual en cm
  short targetAltitude = (currentAltitude - 15) * 10;  // Bajar 15 cm y convertir a milimetros

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

  Serial.write(mavlinkBuf, mavlink_msg_to_send_buffer(mavlinkBuf, &msg));
}

// Función para avanzar una distancia específica en metros
void moveForward() {
  mavlink_message_t msg;

  // Valores de posición relativa (adelante en el eje X)
  short targetX = 350*10;  // Moverse hacia adelante la distancia especificada
  short targetY = 0;  // Mantener la misma posición en Y
  short targetZ = 0;  // Mantener la misma altitud
  uint16_t typeMask = 0b111111000111;  // Ignorar velocidades y aceleración

  // Comando para enviar la posición objetivo
  mavlink_msg_set_position_target_local_ned_pack(
    1,     // ID del sistema (puede ser 1 para el dron)
    200,   // ID del componente (normalmente 200 para el controlador de vuelo)
    &msg,
    0,     // Time boot ms (0 para inmediato)
    1,     // Target system
    0,     // Target component
    MAV_FRAME_LOCAL_NED,  // Referencia local NED
    typeMask,             // Máscara de tipo para solo posición
    targetX, targetY, -targetZ,  // Posiciones X, Y, Z
    0, 0, 0,               // Velocidades ignoradas
    0, 0, 0,               // Aceleraciones ignoradas
    0, 0                    // Yaw y velocidad de yaw ignoradas
  );

  // Convertir el mensaje MAVLink en un buffer y enviarlo
  Serial.write(mavlinkBuf, mavlink_msg_to_send_buffer(mavlinkBuf, &msg));
}

// Función para subir el dron una distancia específica en metros
void subirDron(int alturaInicial) {
  mavlink_message_t msg;

  // Valores de posición relativa (subir en el eje Z)
  short targetX = 0;  // Mantener la misma posición en X
  short targetY = 0;  // Mantener la misma posición en Y
  short targetZ = alturaInicial * 10;  // Subir la distancia especificada
  uint16_t typeMask = 0b111111000111;  // Ignorar velocidades y aceleración

  // Comando para enviar la posición objetivo
  mavlink_msg_set_position_target_local_ned_pack(
    1,     // ID del sistema (puede ser 1 para el dron)
    200,   // ID del componente (normalmente 200 para el controlador de vuelo)
    &msg,
    0,     // Time boot ms (0 para inmediato)
    1,     // Target system
    0,     // Target component
    MAV_FRAME_LOCAL_NED,  // Referencia local NED
    typeMask,             // Máscara de tipo para solo posición
    targetX, targetY, -targetZ,  // Posiciones X, Y, Z
    0, 0, 0,               // Velocidades ignoradas
    0, 0, 0,               // Aceleraciones ignoradas
    0, 0                    // Yaw y velocidad de yaw ignoradas
  );

  // Convertir el mensaje MAVLink en un buffer y enviarlo
  Serial.write(mavlinkBuf, mavlink_msg_to_send_buffer(mavlinkBuf, &msg));
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

    Serial.write(mavlinkBuf, mavlink_msg_to_send_buffer(mavlinkBuf, &msg));
}

void armarDron() {
    mavlink_message_t msg;

    // Crear un mensaje MAVLink para armar el dron
    mavlink_msg_command_long_pack(
        1,                         // ID del sistema (normalmente 1 para el dron)
        MAV_COMP_ID_SYSTEM_CONTROL,// ID del componente (200 es común para el controlador de vuelo)
        &msg,
        1,                         // ID del sistema de destino (tu dron)
        MAV_COMP_ID_AUTOPILOT1,    // ID del componente de destino (autopiloto del dron)
        MAV_CMD_COMPONENT_ARM_DISARM, // Comando para armar/desarmar el dron
        0,                         // Confirmación
        1,                         // Parámetro 1: 1 para armar (0 para desarmar)
        0, 0, 0, 0, 0, 0          // Otros parámetros no usados
    );

    // Convertir el mensaje a formato binario y enviarlo por Serial

    Serial.write(mavlinkBuf, mavlink_msg_to_send_buffer(mavlinkBuf, &msg));
}

void moverArriba(float alturaEnMetros) {
    mavlink_message_t msg;

    // Configuración de la posición relativa
    float targetX = 0;  // Mantener posición en X
    float targetY = 0;  // Mantener posición en Y
    float targetZ = -alturaEnMetros;  // Subir altura (en NED, valores negativos suben)
    uint16_t typeMask = 0b111111000111;  // Ignorar velocidades y aceleración, solo usar posición

    // Comando para enviar la posición relativa
    mavlink_msg_set_position_target_local_ned_pack(
        1,                         // ID del sistema (1 para el dron)
        MAV_COMP_ID_SYSTEM_CONTROL,// ID del componente (200 para controlador de vuelo)
        &msg,
        0,                         // Tiempo desde boot (0 para inmediato)
        1,                         // ID del sistema de destino
        0,                         // ID del componente de destino
        MAV_FRAME_LOCAL_NED,       // Referencia de posición local NED
        typeMask,                  // Máscara de tipo (ignorar velocidades y aceleraciones)
        targetX, targetY, targetZ, // Posición X, Y, Z relativa
        0, 0, 0,                   // Ignorar velocidades
        0, 0, 0,                   // Ignorar aceleraciones
        0, 0                       // Ignorar yaw y velocidad de yaw
    );

    // Convertir el mensaje a formato binario y enviarlo
    uint16_t len = mavlink_msg_to_send_buffer(mavlinkBuf, &msg);
    Serial.write(mavlinkBuf, len);
}

void setModeGuided() {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        1,                         // ID del sistema
        MAV_COMP_ID_SYSTEM_CONTROL,// ID del componente
        &msg,
        1,                         // Target system
        0,                         // Target component
        MAV_CMD_DO_SET_MODE,       // Comando para establecer modo
        0,                         // Confirmation
        1,                         // Modo base (1 para GUIDED)
        4,                         // Modo personalizado (4 para GUIDED)
        0, 0, 0, 0, 0             // Parámetros adicionales no usados
    );

    uint16_t len = mavlink_msg_to_send_buffer(mavlinkBuf, &msg);
    Serial.write(mavlinkBuf, len);
}
