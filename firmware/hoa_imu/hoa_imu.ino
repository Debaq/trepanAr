// HOA IMU — ESP32-C3 Super Mini + MPU9250 (o MPU6500/9255)
// Salida 100 Hz por USB a 115200: roll,pitch,yaw,qw,qx,qy,qz,campo,toques,ix,iy,iz,mx,my,mz
// (grados, cuaternión, |B| en µT, contador de golpecitos laterales, vector de impacto en g sin gravedad,
// vector del magnetómetro en µT para la polaridad de los imanes)
// Líneas que empiezan con '#' son diagnóstico.
//
// Comandos por serie:
//   g  recalibrar giroscopio (dejar quieto ~1 s)
//   m  calibrar magnetómetro 20 s (girar en todas direcciones); se guarda en flash
//   x  borrar calibración del magnetómetro (vuelve a 6 ejes)
//   s  re-escanear I2C
//
// Si el MPU no responde en SDA=6/SCL=7, prueba otros pares de pines y avisa cuál sirvió.

#include <Wire.h>
#include <Preferences.h>
#include <math.h>

#define SDA_DEFECTO 6
#define SCL_DEFECTO 7
#define MAG_ADDR 0x0C
#define PERIODO_US 10000  // 100 Hz

static const uint8_t PINES[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 21};

Preferences prefs;
int pinSda = SDA_DEFECTO, pinScl = SCL_DEFECTO;
uint8_t mpuAddr = 0;
bool hayMag = false, magCalibrado = false;
float asa[3] = {1, 1, 1};           // ajuste de fábrica AK8963
float magOff[3] = {0, 0, 0};        // hierro duro
float magEsc[3] = {1, 1, 1};        // escala por eje (hierro blando simple)
float gBias[3] = {0, 0, 0};

// Mahony
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float iFbx = 0, iFby = 0, iFbz = 0;
const float twoKp = 2.0f * 0.5f, twoKi = 2.0f * 0.0f;

uint32_t ultimoErr = 0;
uint32_t ajustesSesgo = 0;  // veces que se re-midió el sesgo del giro estando quieto

// Modo crudo ('r'): R,ax,ay,az,gx,gy,gz,mx,my,mz
// acel en g, giro en °/s (sin sesgo), mag en µT (sin calibrar)
bool modoCrudo = false;
bool magSaturado = false;  // en crudo se reporta como mx=my=mz=9999

// Golpecitos con el dedo al costado del lápiz (eje del lápiz = Z del sensor).
// Medido: golpe 3-5 g lateral; apoyar la punta en el cráneo <=1.4 g lateral (sobre todo axial).
#define TOQUE_G 2.0f        // aceleración lateral mínima (g)
#define TOQUE_RAZON 1.5f    // lateral / axial mínima
#define TOQUE_REFRACT 250   // ms entre golpes
uint32_t toques = 0, tUltimoToque = 0;
float aLenta[3] = {0, 0, 1};
float ultHp[3] = {0, 0, 0};  // vector de impacto: la demo aprende dirección y umbral de los golpes

void detectarToque(const float a[3]) {
  float hp[3];
  for (int k = 0; k < 3; k++) {
    hp[k] = a[k] - aLenta[k];
    aLenta[k] += 0.05f * hp[k];  // promedio lento (~0.2 s): gravedad + movimiento de la mano
  }
  float lateral = sqrtf(hp[0] * hp[0] + hp[1] * hp[1]);
  for (int k = 0; k < 3; k++) ultHp[k] = hp[k];
  if (lateral > TOQUE_G && lateral > TOQUE_RAZON * fabsf(hp[2]) && millis() - tUltimoToque > TOQUE_REFRACT) {
    toques++;
    tUltimoToque = millis();
  }
}

// ---------- I2C ----------
bool escribir(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool leer(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool responde(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

uint8_t buscarMpu() {
  if (responde(0x68)) return 0x68;
  if (responde(0x69)) return 0x69;
  return 0;
}

void listarBus() {
  Serial.print("# i2c:");
  int n = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    if (responde(a)) { Serial.printf(" 0x%02X", a); n++; }
  }
  Serial.println(n ? "" : " (nada)");
}

// Prueba SDA/SCL por defecto y, si no hay MPU, todos los pares posibles.
bool encontrarPines() {
  Wire.end();
  Wire.begin(pinSda, pinScl, 400000);
  Wire.setTimeOut(10);
  Serial.printf("# probando SDA=GPIO%d SCL=GPIO%d\n", pinSda, pinScl);
  listarBus();
  mpuAddr = buscarMpu();
  if (mpuAddr) return true;

  Serial.println("# MPU no responde ahi; escaneando otros pines...");
  for (uint8_t a : PINES) {
    for (uint8_t b : PINES) {
      if (a == b) continue;
      Wire.end();
      if (!Wire.begin(a, b, 100000)) continue;
      Wire.setTimeOut(5);
      uint8_t m = buscarMpu();
      if (m) {
        pinSda = a; pinScl = b; mpuAddr = m;
        Wire.end();
        Wire.begin(pinSda, pinScl, 400000);
        Wire.setTimeOut(10);
        Serial.printf("# MPU encontrado en SDA=GPIO%d SCL=GPIO%d (0x%02X)\n", a, b, m);
        return true;
      }
    }
  }
  Wire.end();
  Wire.begin(pinSda = SDA_DEFECTO, pinScl = SCL_DEFECTO, 400000);
  Wire.setTimeOut(10);
  Serial.println("# ERR no hay MPU en ningun par de pines: revisar VCC (3V3), GND, SDA, SCL y AD0");
  return false;
}

// ---------- Sensor ----------
bool iniciarMpu() {
  if (!encontrarPines()) return false;
  uint8_t who = 0;
  leer(mpuAddr, 0x75, &who, 1);
  const char *nombre = who == 0x71 ? "MPU9250" : who == 0x73 ? "MPU9255" :
                       who == 0x70 ? "MPU6500 (sin magnetometro)" : who == 0x68 ? "MPU6050/9150" : "desconocido";
  Serial.printf("# WHO_AM_I=0x%02X -> %s\n", who, nombre);

  escribir(mpuAddr, 0x6B, 0x80); delay(100);   // reset
  escribir(mpuAddr, 0x6B, 0x01); delay(50);    // reloj PLL
  escribir(mpuAddr, 0x1A, 0x03);               // DLPF giro 41 Hz
  escribir(mpuAddr, 0x19, 0x04);               // 1 kHz / 5 = 200 Hz
  escribir(mpuAddr, 0x1B, 0x10);               // giro ±1000 °/s
  escribir(mpuAddr, 0x1C, 0x08);               // acel ±4 g
  escribir(mpuAddr, 0x1D, 0x03);               // DLPF acel 41 Hz
  escribir(mpuAddr, 0x37, 0x02);               // bypass -> AK8963 visible en el bus
  delay(10);

  hayMag = false;
  uint8_t wia = 0;
  if (leer(MAG_ADDR, 0x00, &wia, 1) && wia == 0x48) {
    uint8_t raw[3];
    escribir(MAG_ADDR, 0x0A, 0x00); delay(10);
    escribir(MAG_ADDR, 0x0A, 0x0F); delay(10);  // fuse ROM
    leer(MAG_ADDR, 0x10, raw, 3);
    for (int i = 0; i < 3; i++) asa[i] = (raw[i] - 128) / 256.0f + 1.0f;
    escribir(MAG_ADDR, 0x0A, 0x00); delay(10);
    escribir(MAG_ADDR, 0x0A, 0x16); delay(10);  // 16 bit, 100 Hz continuo
    hayMag = true;
    Serial.println("# AK8963 ok");
  } else {
    Serial.println("# sin magnetometro: yaw va a derivar lentamente");
  }
  return true;
}

bool leerImu(float a[3], float g[3]) {
  uint8_t b[14];
  if (!leer(mpuAddr, 0x3B, b, 14)) return false;
  for (int i = 0; i < 3; i++) {
    a[i] = (int16_t)(b[2 * i] << 8 | b[2 * i + 1]) / 8192.0f;                       // g
    g[i] = (int16_t)(b[8 + 2 * i] << 8 | b[9 + 2 * i]) / 32.8f * DEG_TO_RAD;         // rad/s
  }
  return true;
}

// Devuelve el campo en el marco del acelerómetro (µT aprox).
bool leerMag(float m[3]) {
  uint8_t st1;
  if (!leer(MAG_ADDR, 0x02, &st1, 1) || !(st1 & 0x01)) return false;
  uint8_t b[7];
  if (!leer(MAG_ADDR, 0x03, b, 7)) return false;
  magSaturado = b[6] & 0x08;  // imán muy cerca (>4800 µT)
  if (magSaturado) return false;
  float r[3];
  for (int i = 0; i < 3; i++) r[i] = (int16_t)(b[2 * i + 1] << 8 | b[2 * i]) * asa[i] * 0.15f;
  m[0] = r[1]; m[1] = r[0]; m[2] = -r[2];  // ejes del AK8963 -> ejes del MPU
  return true;
}

// Sesgo del giroscopio automático: cada ~1 s en que el sensor está quieto (giro y aceleración casi
// constantes) se vuelve a medir. Corrige un sesgo mal medido al encender (lápiz en movimiento) y
// los cambios por temperatura. Medido: en reposo el giro varía ~0,05-0,1 °/s; con la mano, mucho más.
#define QUIETO_N 100            // muestras por ventana (1 s a 100 Hz)
#define QUIETO_GIRO_STD 0.6f    // °/s
#define QUIETO_ACEL_STD 0.015f  // g
float qSum[3], qSum2[3], qAcSum, qAcSum2;
int qN = 0;

void sesgoAutomatico(const float a[3], const float g[3]) {
  for (int k = 0; k < 3; k++) { qSum[k] += g[k]; qSum2[k] += g[k] * g[k]; }
  float am = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  qAcSum += am; qAcSum2 += am * am;
  if (++qN < QUIETO_N) return;
  bool quieto = true;
  for (int k = 0; k < 3; k++) {
    float media = qSum[k] / qN, var = qSum2[k] / qN - media * media;
    if (sqrtf(fmaxf(var, 0)) * RAD_TO_DEG > QUIETO_GIRO_STD) quieto = false;
  }
  float am_m = qAcSum / qN;
  if (sqrtf(fmaxf(qAcSum2 / qN - am_m * am_m, 0)) > QUIETO_ACEL_STD) quieto = false;
  if (quieto) {
    for (int k = 0; k < 3; k++) gBias[k] = qSum[k] / qN;
    ajustesSesgo++;
  }
  for (int k = 0; k < 3; k++) { qSum[k] = 0; qSum2[k] = 0; }
  qAcSum = qAcSum2 = 0;
  qN = 0;
}

void calibrarGiro() {
  Serial.println("# calibrando giro: no mover...");
  float s[3] = {0, 0, 0}, a[3], g[3];
  int n = 0;
  for (int i = 0; i < 200; i++) {
    if (leerImu(a, g)) { for (int k = 0; k < 3; k++) s[k] += g[k]; n++; }
    delay(5);
  }
  if (n) for (int k = 0; k < 3; k++) gBias[k] = s[k] / n;
  Serial.printf("# sesgo giro (dps): %.2f %.2f %.2f\n", gBias[0] * RAD_TO_DEG, gBias[1] * RAD_TO_DEG, gBias[2] * RAD_TO_DEG);
}

void calibrarMag() {
  if (!hayMag) { Serial.println("# no hay magnetometro"); return; }
  Serial.println("# calibrando magnetometro 20 s: girar en todas direcciones (ochos)...");
  float mn[3] = {1e9, 1e9, 1e9}, mx[3] = {-1e9, -1e9, -1e9}, m[3];
  uint32_t t0 = millis();
  while (millis() - t0 < 20000) {
    if (leerMag(m)) for (int k = 0; k < 3; k++) { mn[k] = min(mn[k], m[k]); mx[k] = max(mx[k], m[k]); }
    delay(10);
  }
  float rad[3], prom = 0;
  for (int k = 0; k < 3; k++) { magOff[k] = (mx[k] + mn[k]) / 2; rad[k] = (mx[k] - mn[k]) / 2; prom += rad[k] / 3; }
  for (int k = 0; k < 3; k++) magEsc[k] = rad[k] > 1 ? prom / rad[k] : 1;
  magCalibrado = prom > 5;
  prefs.putBytes("off", magOff, sizeof(magOff));
  prefs.putBytes("esc", magEsc, sizeof(magEsc));
  prefs.putBool("ok", magCalibrado);
  Serial.printf("# mag offset %.1f %.1f %.1f  escala %.2f %.2f %.2f  %s\n", magOff[0], magOff[1], magOff[2],
                magEsc[0], magEsc[1], magEsc[2], magCalibrado ? "OK (9 ejes)" : "FALLO: pocos datos");
}

// ---------- Mahony AHRS ----------
void mahony(float gx, float gy, float gz, float ax, float ay, float az,
            float mx, float my, float mz, bool usarMag, float dt) {
  float n = sqrtf(ax * ax + ay * ay + az * az);
  if (n > 0) {
    ax /= n; ay /= n; az /= n;
    float vx = q1 * q3 - q0 * q2, vy = q0 * q1 + q2 * q3, vz = q0 * q0 - 0.5f + q3 * q3;
    float ex = ay * vz - az * vy, ey = az * vx - ax * vz, ez = ax * vy - ay * vx;

    if (usarMag) {
      float nm = sqrtf(mx * mx + my * my + mz * mz);
      if (nm > 0) {
        mx /= nm; my /= nm; mz /= nm;
        float hx = 2 * (mx * (0.5f - q2 * q2 - q3 * q3) + my * (q1 * q2 - q0 * q3) + mz * (q1 * q3 + q0 * q2));
        float hy = 2 * (mx * (q1 * q2 + q0 * q3) + my * (0.5f - q1 * q1 - q3 * q3) + mz * (q2 * q3 - q0 * q1));
        float bx = sqrtf(hx * hx + hy * hy);
        float bz = 2 * (mx * (q1 * q3 - q0 * q2) + my * (q2 * q3 + q0 * q1) + mz * (0.5f - q1 * q1 - q2 * q2));
        float wx = bx * (0.5f - q2 * q2 - q3 * q3) + bz * (q1 * q3 - q0 * q2);
        float wy = bx * (q1 * q2 - q0 * q3) + bz * (q0 * q1 + q2 * q3);
        float wz = bx * (q0 * q2 + q1 * q3) + bz * (0.5f - q1 * q1 - q2 * q2);
        ex += my * wz - mz * wy; ey += mz * wx - mx * wz; ez += mx * wy - my * wx;
      }
    }
    if (twoKi > 0) {
      iFbx += twoKi * ex * dt; iFby += twoKi * ey * dt; iFbz += twoKi * ez * dt;
      gx += iFbx; gy += iFby; gz += iFbz;
    }
    gx += twoKp * ex; gy += twoKp * ey; gz += twoKp * ez;
  }
  gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
  float qa = q0, qb = q1, qc = q2;
  q0 += -qb * gx - qc * gy - q3 * gz;
  q1 += qa * gx + qc * gz - q3 * gy;
  q2 += qa * gy - qb * gz + q3 * gx;
  q3 += qa * gz + qb * gy - qc * gx;
  n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= n; q1 /= n; q2 /= n; q3 /= n;
}

// ---------- Arduino ----------
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);
  Serial.println("# hoa-imu v1 (ESP32-C3 + MPU9250)");
  Serial.println("# formato: roll,pitch,yaw,qw,qx,qy,qz,campo_uT (9999 = saturado),toques,impacto_x,y,z (g),mag_x,y,z (uT; 9999 = saturado)");

  prefs.begin("hoaimu", false);
  magCalibrado = prefs.getBool("ok", false);
  if (magCalibrado) {
    prefs.getBytes("off", magOff, sizeof(magOff));
    prefs.getBytes("esc", magEsc, sizeof(magEsc));
  }

  if (iniciarMpu()) calibrarGiro();
  Serial.printf("# modo: %s\n", hayMag && magCalibrado ? "9 ejes" : "6 ejes (enviar 'm' para calibrar magnetometro)");
}

void loop() {
  static uint32_t tPrev = micros();
  static float m[3] = {0, 0, 0};

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'g') calibrarGiro();
    else if (c == 'm') calibrarMag();
    else if (c == 'x') { magCalibrado = false; prefs.putBool("ok", false); Serial.println("# mag borrado: 6 ejes"); }
    else if (c == 's') { if (iniciarMpu()) calibrarGiro(); }
    else if (c == 'r') {
      modoCrudo = !modoCrudo;
      Serial.println(modoCrudo ? "# crudo: R,ax,ay,az,gx,gy,gz,mx,my,mz" : "# modo normal");
    }
    tPrev = micros();
  }

  uint32_t ahora = micros();
  if (ahora - tPrev < PERIODO_US) return;
  float dt = (ahora - tPrev) * 1e-6f;
  tPrev = ahora;

  float a[3], g[3];
  if (!mpuAddr || !leerImu(a, g)) {
    if (millis() - ultimoErr > 1000) {
      ultimoErr = millis();
      // Reconexión en caliente: si el MPU aparece en los pines actuales, reiniciarlo.
      if (buscarMpu()) {
        Serial.println("# MPU detectado, iniciando...");
        if (iniciarMpu()) calibrarGiro();
        tPrev = micros();
        return;
      }
      Serial.println("# ERR sin MPU: revisar VCC/GND/SDA/SCL del modulo (se reintenta solo)");
    }
    return;
  }
  detectarToque(a);
  sesgoAutomatico(a, g);
  bool usarMag = false;
  static float mCrudo[3] = {NAN, NAN, NAN};
  if (hayMag) {
    float mr[3];
    if (leerMag(mr)) for (int k = 0; k < 3; k++) { mCrudo[k] = mr[k]; m[k] = (mr[k] - magOff[k]) * magEsc[k]; }
    else if (magSaturado) for (int k = 0; k < 3; k++) mCrudo[k] = 9999;
    usarMag = false;  // nunca: los imanes del cráneo desviarían la orientación (el mag solo detecta imanes)
  }
  if (modoCrudo) {
    Serial.printf("R,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                  a[0], a[1], a[2], (g[0] - gBias[0]) * RAD_TO_DEG, (g[1] - gBias[1]) * RAD_TO_DEG,
                  (g[2] - gBias[2]) * RAD_TO_DEG, mCrudo[0], mCrudo[1], mCrudo[2]);
    return;
  }
  mahony(g[0] - gBias[0], g[1] - gBias[1], g[2] - gBias[2], a[0], a[1], a[2], m[0], m[1], m[2], usarMag, dt);

  float roll = atan2f(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
  float pitch = asinf(constrain(2 * (q0 * q2 - q3 * q1), -1.0f, 1.0f)) * RAD_TO_DEG;
  float yaw = atan2f(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
  float campo = magSaturado ? 9999 : sqrtf(mCrudo[0] * mCrudo[0] + mCrudo[1] * mCrudo[1] + mCrudo[2] * mCrudo[2]);
  Serial.printf("%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.1f,%lu,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f\n", roll, pitch, yaw,
                q0, q1, q2, q3, campo, (unsigned long)toques, ultHp[0], ultHp[1], ultHp[2],
                mCrudo[0], mCrudo[1], mCrudo[2]);
}
