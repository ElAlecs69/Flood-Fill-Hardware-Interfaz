/*
 * Micromouse con giroscopio + Bluetooth + Encoders
 * Centrado automático y reversa INTELIGENTE hasta encontrar salida
 * Desconectar pines 0 y 1 para subir código
 */

#include <Wire.h>

// Motores
#define ENA 9
#define IN1 7
#define IN2 8
#define ENB 10
#define IN3 4
#define IN4 5

// Encoders
#define ENCODER_A 3  // Motor izquierdo (interrupt pin)
#define ENCODER_B 2  // Motor derecho (interrupt pin)

// Sensores ultrasónicos
#define TRIG_FRONT 11
#define ECHO_FRONT 12
#define TRIG_LEFT A0
#define ECHO_LEFT A1
#define TRIG_RIGHT A2
#define ECHO_RIGHT A3

// MPU6050
const int MPU = 0x68;

// Flood-fill / maze planning
#define MAZE_W 12
#define MAZE_H 7
#define WALL_N (1<<0)
#define WALL_E (1<<1)
#define WALL_S (1<<2)
#define WALL_W (1<<3)

// Arrays use [row][col] = [x][y]
uint8_t walls[MAZE_H][MAZE_W]; // bitmask: N E S W
uint8_t floodMap[MAZE_H][MAZE_W];
bool visitedMap[MAZE_H][MAZE_W];

int robotX = MAZE_H - 1; // start X (row) default bottom-left
int robotY = 0;         // start Y (col)
int robotDir = 0; // 0=N,1=E,2=S,3=W

enum Move {MOVE_FORWARD, MOVE_LEFT, MOVE_RIGHT, MOVE_BACK, MOVE_NONE};

// Planning prototypes
void setWallAt(int x, int y, int dir);
void updateWalls(bool front, bool left, bool right);
void computeFloodFill();
Move chooseNextMove();
void driveOneCell();
bool canGo(int x, int y, int dir);

// Constantes de velocidad
const int TURN_SPEED = 85;
const int SPEED_NORMAL = 80;
const int SPEED_REVERSE = 70;  // Velocidad para reversa (un poco más lento)
const int MOTOR_OFFSET = 5;

// Constantes de distancia (en cm)
const int FRONT_STOP = 7;
const int MAX_SIDE_DISTANCE = 12;
const int WALL_TOO_CLOSE = 4;
const int WALL_TOO_FAR = 8;
const int IDEAL_WALL_DISTANCE = 6;
const int BACK_WALL_DETECT = 7;  // Distancia para detectar pared atrás (si retrocedemos mucho)

// Constantes de giro y corrección
const int TURN_ANGLE = 38;
const float CORRECTION_FACTOR = 3.0;
const float WALL_CORRECTION_FACTOR = 8.0;
const unsigned long TURN_TIMEOUT_MS = 900; // safety timeout for turns

// Constantes de encoder y laberinto
const int PULSES_PER_CELL = 25;  // 25 pulsos = 17cm = 1 celda
const int MAX_REVERSE_CELLS = 6; // Máximo de celdas a retroceder (seguridad)

// Variables del giroscopio
float yaw = 0;
float gyroZ_offset = 0;
unsigned long lastTime;

// Variables de Bluetooth
unsigned long lastSend = 0;
const int SEND_INTERVAL = 100;

// Variables de encoders (volatile porque se usan en interrupciones)
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

// Prototipos de funciones
void encoderISR_A();
void encoderISR_B();
void driveForwardWithCentering(int distLeft, int distRight, bool wallLeft, bool wallRight);
void driveForwardStraight();
void driveBackwardOneCell();
void resetStraightDrive();
void turnRight90();
void turnLeft90();
void driveForward();
void updateGyro();
int getDistance(int trigPin, int echoPin);
void stopMotors();
void resetEncoders();
long getEncoderAverage();

void setup() {
  Serial.begin(9600);
  
  // Configurar pines de motores
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Configurar pines de sensores
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  // Configurar encoders con interrupciones
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR_B, RISING);
  
  // Inicializar MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  delay(2000);
  
  // Calibrar giroscopio
  float suma = 0;
  for (int i = 0; i < 1000; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 2, true);
    int16_t gz = Wire.read() << 8 | Wire.read();
    suma += gz;
    delay(1);
  }
  gyroZ_offset = suma / 1000.0;
  
  delay(3000);
  Serial.println("START");
  resetStraightDrive();

  // initialize maps
  for (int i = 0; i < MAZE_H; ++i) {
    for (int j = 0; j < MAZE_W; ++j) {
      walls[i][j] = 0;
      floodMap[i][j] = 255;
      visitedMap[i][j] = false;
    }
  }
  // mark starting cell visited
  visitedMap[robotX][robotY] = true;
  computeFloodFill();

  // mark outer boundary walls so planner doesn't try to leave maze
  for (int c = 0; c < MAZE_W; ++c) {
    setWallAt(0, c, 0); // north row
    setWallAt(MAZE_H-1, c, 2); // south row
  }
  for (int r = 0; r < MAZE_H; ++r) {
    setWallAt(r, 0, 3); // west col
    setWallAt(r, MAZE_W-1, 1); // east col
  }
}

void loop() {
  // We assume the robot is centered in a cell at loop start.
  // Read sensors at center, update walls, then compute flood and choose move.
  int distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
  int distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
  int distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  bool wallFront = (distFront <= FRONT_STOP) && (distFront > 0);
  bool wallLeft = (distLeft < MAX_SIDE_DISTANCE) && (distLeft > 0);
  bool wallRight = (distRight < MAX_SIDE_DISTANCE) && (distRight > 0);

  // Update map with current sensor readings (we are at cell center)
  updateWalls(wallFront, wallLeft, wallRight);
  computeFloodFill();

  // Optional telemetry
  if (millis() - lastSend > SEND_INTERVAL) {
    Serial.print(",");
Serial.print(robotX);
Serial.print(",");
Serial.print(robotY);
Serial.print(",");
Serial.print(robotDir);

    Serial.print(distFront);
    Serial.print(",");
    Serial.print(distLeft);
    Serial.print(",");
    Serial.print(distRight);
    Serial.print(",");
    Serial.print(yaw, 1);
    Serial.print(",");
    if (wallFront && wallLeft && wallRight) Serial.println("ATRAPADO");
    else if (wallFront) {
      if (!wallRight) Serial.println("GIRO_DER");
      else if (!wallLeft) Serial.println("GIRO_IZQ");
      else Serial.println("MEDIA_VUELTA");
    } else {
      Serial.println("AVANZA");
    }
    lastSend = millis();
  }

  Move next = chooseNextMove();
  switch (next) {
    case MOVE_FORWARD:
      driveOneCell();
      break;
    case MOVE_LEFT:
      turnLeft90();
      driveOneCell();
      break;
    case MOVE_RIGHT:
      turnRight90();
      driveOneCell();
      break;
    case MOVE_BACK:
      turnLeft90();
      turnLeft90();
      driveOneCell();
      break;
    default:
      stopMotors();
      break;
  }

  // After arriving to new cell center, read sensors and update map
  distFront = getDistance(TRIG_FRONT, ECHO_FRONT);
  distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
  distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);
  wallFront = (distFront <= FRONT_STOP) && (distFront > 0);
  wallLeft = (distLeft < MAX_SIDE_DISTANCE) && (distLeft > 0);
  wallRight = (distRight < MAX_SIDE_DISTANCE) && (distRight > 0);
  updateWalls(wallFront, wallLeft, wallRight);
  computeFloodFill();
  
  delay(50);
}

/* escapeDeadEnd removed: flood-fill planner + exploration handles dead-ends */

/*
 * Retroceder exactamente una celda usando encoders
 */
void driveBackwardOneCell() {
  resetEncoders();
  
  // Resetear yaw para mantener línea recta en reversa
  yaw = 0;
  lastTime = micros();
  
  while (getEncoderAverage() < PULSES_PER_CELL) {
    // Actualizar giroscopio para corrección
    updateGyro();
    
    // Corrección para ir recto en reversa (invertida)
    int correccion = yaw * CORRECTION_FACTOR;
    
    int velIzq = SPEED_REVERSE - correccion;  // Invertido porque vamos en reversa
    int velDer = (SPEED_REVERSE - MOTOR_OFFSET) + correccion;
    
    velIzq = constrain(velIzq, 50, 120);
    velDer = constrain(velDer, 50, 120);
    
    // Motores en reversa
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, velIzq);
    analogWrite(ENB, velDer);
    
    delay(5);
  }
  
  stopMotors();
}

// ISRs para encoders
void encoderISR_A() {
  encoderCountA++;
}

void encoderISR_B() {
  encoderCountB++;
}

// Resetear contadores de encoder
void resetEncoders() {
  noInterrupts();
  encoderCountA = 0;
  encoderCountB = 0;
  interrupts();
}

// Obtener promedio de pulsos de ambos encoders
long getEncoderAverage() {
  noInterrupts();
  long avg = (encoderCountA + encoderCountB) / 2;
  interrupts();
  return avg;
}

// Avanzar con centrado entre paredes
void driveForwardWithCentering(int distLeft, int distRight, bool wallLeft, bool wallRight) {
  updateGyro();
  
  // Corrección base por giroscopio
  int correccion = yaw * CORRECTION_FACTOR;
  
  // Corrección adicional por paredes laterales (centrado)
  int wallCorrection = 0;
  
  if (wallLeft && wallRight) {
    // Ambas paredes - centrar
    int diff = distLeft - distRight;
    wallCorrection = diff * WALL_CORRECTION_FACTOR / 2;
  }
  else if (wallLeft && distLeft < WALL_TOO_CLOSE) {
    // Muy cerca de pared izquierda
    wallCorrection = -(IDEAL_WALL_DISTANCE - distLeft) * WALL_CORRECTION_FACTOR;
  }
  else if (wallRight && distRight < WALL_TOO_CLOSE) {
    // Muy cerca de pared derecha
    wallCorrection = (IDEAL_WALL_DISTANCE - distRight) * WALL_CORRECTION_FACTOR;
  }
  else if (wallLeft && distLeft > WALL_TOO_FAR) {
    wallCorrection = (distLeft - IDEAL_WALL_DISTANCE) * WALL_CORRECTION_FACTOR / 2;
  }
  else if (wallRight && distRight > WALL_TOO_FAR) {
    wallCorrection = -(distRight - IDEAL_WALL_DISTANCE) * WALL_CORRECTION_FACTOR / 2;
  }
  
  wallCorrection = constrain(wallCorrection, -20, 20);
  
  int totalCorrection = correccion + wallCorrection;
  
  int velIzq = SPEED_NORMAL + totalCorrection;
  int velDer = (SPEED_NORMAL - MOTOR_OFFSET) - totalCorrection;
  
  velIzq = constrain(velIzq, 50, 150);
  velDer = constrain(velDer, 50, 150);
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, velIzq);
  analogWrite(ENB, velDer);
}

void driveForwardStraight() {
  updateGyro();
  int correccion = yaw * CORRECTION_FACTOR;
  int velIzq = SPEED_NORMAL + correccion;
  int velDer = (SPEED_NORMAL - MOTOR_OFFSET) - correccion;
  velIzq = constrain(velIzq, 60, 150);
  velDer = constrain(velDer, 60, 150);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, velIzq);
  analogWrite(ENB, velDer);
}

void resetStraightDrive() {
  yaw = 0;
  lastTime = micros();
  resetEncoders();
}

void turnRight90() {
  yaw = 0;
  lastTime = micros();
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
  unsigned long t0 = millis();
  while (abs(yaw) < TURN_ANGLE && (millis() - t0) < TURN_TIMEOUT_MS) {
    updateGyro();
    delay(5);
  }
  stopMotors();
  delay(200);
  // update logical heading
  robotDir = (robotDir + 1) % 4;
}

void turnLeft90() {
  yaw = 0;
  lastTime = micros();
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
  unsigned long t0 = millis();
  while (abs(yaw) < TURN_ANGLE && (millis() - t0) < TURN_TIMEOUT_MS) {
    updateGyro();
    delay(5);
  }
  stopMotors();
  delay(200);
  // update logical heading
  robotDir = (robotDir + 3) % 4;
}

void driveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED_NORMAL);
  analogWrite(ENB, SPEED_NORMAL - MOTOR_OFFSET);
}

void updateGyro() {
  Wire.beginTransmission(MPU);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);
  int16_t gz = Wire.read() << 8 | Wire.read();
  float gyroZ = (gz - gyroZ_offset) / 131.0;
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;
  yaw += gyroZ * dt;
}

int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 15000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

/* --- Planning helpers --- */
bool canGo(int x, int y, int dir) {
  // return true only if there is no known wall in dir from (x,y) and the neighbor has been visited
  if (x < 0 || x >= MAZE_H || y < 0 || y >= MAZE_W) return false;
  uint8_t mask = (dir==0)?WALL_N:(dir==1)?WALL_E:(dir==2)?WALL_S:WALL_W;
  if (walls[x][y] & mask) return false; // known wall
  int nx = x + (dir==0?-1:(dir==2?1:0));
  int ny = y + (dir==1?1:(dir==3?-1:0));
  if (nx < 0 || nx >= MAZE_H || ny < 0 || ny >= MAZE_W) return false;
  if (!visitedMap[nx][ny]) return false; // unknown cell treated as blocked for planner
  return true;
}

void setWallAt(int x, int y, int dir) {
  if (x < 0 || x >= MAZE_H || y < 0 || y >= MAZE_W) return;
  uint8_t bit = 0;
  if (dir == 0) bit = WALL_N;
  else if (dir == 1) bit = WALL_E;
  else if (dir == 2) bit = WALL_S;
  else if (dir == 3) bit = WALL_W;
  walls[x][y] |= bit;
  // set opposite wall on neighbor
  int nx = x, ny = y;
  int opp = (dir + 2) % 4;
  if (dir == 0) nx = x - 1;
  else if (dir == 1) ny = y + 1;
  else if (dir == 2) nx = x + 1;
  else if (dir == 3) ny = y - 1;
  if (nx >= 0 && nx < MAZE_H && ny >= 0 && ny < MAZE_W) {
    uint8_t obit = (opp == 0) ? WALL_N : (opp == 1) ? WALL_E : (opp == 2) ? WALL_S : WALL_W;
    walls[nx][ny] |= obit;
  }
}

void updateWalls(bool front, bool left, bool right) {
  if (front) setWallAt(robotX, robotY, robotDir);
  if (left) setWallAt(robotX, robotY, (robotDir + 3) % 4);
  if (right) setWallAt(robotX, robotY, (robotDir + 1) % 4);
}

// Simple BFS flood-fill from center cells (supports rectangular mazes)
void computeFloodFill() {
  // initialize floodMap but DO NOT clear visitedMap (preserve exploration history)
  for (int i = 0; i < MAZE_H; ++i)
    for (int j = 0; j < MAZE_W; ++j)
      floodMap[i][j] = 255;

  // compute center coordinates (handle even/odd dimensions)
  int cx1 = (MAZE_H - 1) / 2;
  int cx2 = MAZE_H / 2;
  int cy1 = (MAZE_W - 1) / 2;
  int cy2 = MAZE_W / 2;

  // queue arrays (simple fixed-size queue)
  const int QMAX = MAZE_H * MAZE_W + 4;
  int qx[QMAX];
  int qy[QMAX];
  int qh = 0, qt = 0;

  // enqueue up to four center cells (deduplicated)
  int centers[4][2] = {{cx1, cy1}, {cx1, cy2}, {cx2, cy1}, {cx2, cy2}};
  for (int ci = 0; ci < 4; ++ci) {
    int gx = centers[ci][0];
    int gy = centers[ci][1];
    if (gx < 0 || gx >= MAZE_H || gy < 0 || gy >= MAZE_W) continue;
    if (floodMap[gx][gy] != 0) {
      floodMap[gx][gy] = 0;
      qx[qt] = gx; qy[qt] = gy; qt++;
    }
  }

  while (qh < qt) {
    int cx = qx[qh];
    int cy = qy[qh];
    qh++;
    uint8_t curVal = floodMap[cx][cy];
    // for each neighbor N(0),E(1),S(2),W(3)
    int nx, ny;
    // NORTH
    nx = cx - 1; ny = cy;
    if (nx >= 0 && canGo(cx, cy, 0)) {
      if (floodMap[nx][ny] > curVal + 1) {
        floodMap[nx][ny] = curVal + 1;
        qx[qt] = nx; qy[qt] = ny; qt++;
      }
    }
    // EAST
    nx = cx; ny = cy + 1;
    if (ny < MAZE_W && canGo(cx, cy, 1)) {
      if (floodMap[nx][ny] > curVal + 1) {
        floodMap[nx][ny] = curVal + 1;
        qx[qt] = nx; qy[qt] = ny; qt++;
      }
    }
    // SOUTH
    nx = cx + 1; ny = cy;
    if (nx < MAZE_H && canGo(cx, cy, 2)) {
      if (floodMap[nx][ny] > curVal + 1) {
        floodMap[nx][ny] = curVal + 1;
        qx[qt] = nx; qy[qt] = ny; qt++;
      }
    }
    // WEST
    nx = cx; ny = cy - 1;
    if (ny >= 0 && canGo(cx, cy, 3)) {
      if (floodMap[nx][ny] > curVal + 1) {
        floodMap[nx][ny] = curVal + 1;
        qx[qt] = nx; qy[qt] = ny; qt++;
      }
    }
  }
}

Move chooseNextMove() {
  // First: prefer exploring adjacent unvisited cells (no wall marked)
  // Preference order: forward, left, right, back
  int pref[4] = {0, 3, 1, 2};
  for (int p = 0; p < 4; ++p) {
    int dir = (robotDir + pref[p]) % 4;
    int nx = robotX + (dir==0?-1:(dir==2?1:0));
    int ny = robotY + (dir==1?1:(dir==3?-1:0));
    if (nx < 0 || nx >= MAZE_H || ny < 0 || ny >= MAZE_W) continue;
    // prefer exploring adjacent unvisited cells where movement is possible
    if (!visitedMap[nx][ny]) {
      // ensure we don't try to move into a known wall
      if (!canGo(robotX, robotY, dir)) continue;
      // choose relative move based on pref[p]
      if (pref[p] == 0) return MOVE_FORWARD;
      if (pref[p] == 3) return MOVE_LEFT;
      if (pref[p] == 1) return MOVE_RIGHT;
      return MOVE_BACK;
    }
  }

  // No adjacent unvisited cells -> use flood values but only through visited cells
  int bestDir = -1;
  int bestVal = 10000;
  for (int d = 0; d < 4; ++d) {
    int nx = robotX + (d==0?-1:(d==2?1:0));
    int ny = robotY + (d==1?1:(d==3?-1:0));
    if (nx < 0 || nx >= MAZE_H || ny < 0 || ny >= MAZE_W) continue;
    // only consider movement if canGo (no known wall and neighbor visited)
    if (!canGo(robotX, robotY, d)) continue;
    int v = floodMap[nx][ny];
    if (v < bestVal) { bestVal = v; bestDir = d; }
  }
  if (bestDir < 0) return MOVE_NONE;
  if (bestDir == robotDir) return MOVE_FORWARD;
  if (bestDir == (robotDir + 1) % 4) return MOVE_RIGHT;
  if (bestDir == (robotDir + 3) % 4) return MOVE_LEFT;
  return MOVE_BACK;
}

void driveOneCell() {
  resetEncoders();
  yaw = 0;
  lastTime = micros();
  while (getEncoderAverage() < PULSES_PER_CELL) {
    updateGyro();
    // check front sensor while moving; if an unexpected wall appears, stop and mark it
    int dfront = getDistance(TRIG_FRONT, ECHO_FRONT);
    if (dfront > 0 && dfront <= FRONT_STOP) {
      // mark wall at current cell in facing direction
      setWallAt(robotX, robotY, robotDir);
      stopMotors();
      resetStraightDrive();
      // recompute flood with new wall info
      computeFloodFill();
      return; // abort movement
    }
    int correccion = yaw * CORRECTION_FACTOR;
    int velIzq = SPEED_NORMAL + correccion;
    int velDer = (SPEED_NORMAL - MOTOR_OFFSET) - correccion;
    velIzq = constrain(velIzq, 50, 150);
    velDer = constrain(velDer, 50, 150);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, velIzq);
    analogWrite(ENB, velDer);
    delay(5);
  }
  stopMotors();
  resetStraightDrive();
  // update robot pose
  if (robotDir == 0) robotX -= 1;
  else if (robotDir == 1) robotY += 1;
  else if (robotDir == 2) robotX += 1;
  else if (robotDir == 3) robotY -= 1;
  // mark visited on arrival to center
  if (robotX >= 0 && robotX < MAZE_H && robotY >= 0 && robotY < MAZE_W)
    visitedMap[robotX][robotY] = true;
  // keep robotDir unchanged (turns change it where appropriate)
}