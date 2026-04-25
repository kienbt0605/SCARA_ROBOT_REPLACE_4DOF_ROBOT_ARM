#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// ============================================================
//  4-DOF Robot Arm Controller — FK + IK + Trajectory
//  Board: ESP32 | Driver: PCA9685 (0x41)
//  Servo channels: 4 (base), 5 (shoulder), 6 (elbow), 7 (wrist)
//  Commands via Serial (115200):
//    LINE X Y Z        — Di chuyển thẳng đến (X,Y,Z)
//    CIRCLE Xc Yc Zc R — Vẽ đường tròn
//    HOME               — Về vị trí home
//    FK                 — Hiển thị vị trí EE hiện tại
// ============================================================

// ---- PCA9685 ----
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

#define USMIN      500
#define USMAX      2400
#define SERVO_FREQ 50

// ---- Servo channels ----
#define NUM_SERVOS 4
uint8_t servoChannels[NUM_SERVOS] = {4, 5, 6, 7};
/*
  Ch4: Base rotation  (J1 — xoay quanh Z)
  Ch5: Shoulder pitch (J2)
  Ch6: Elbow pitch    (J3)
  Ch7: Wrist pitch    (J4)
*/

// ---- Robot geometry (mm) — khớp giống file Python ----
#define NUM_JOINTS 4
#define LINK1      90.0f
#define LINK2      90.0f
#define LINK3      30.0f
#define BASE_H     75.0f

// ---- Joint limits trong không gian IK (độ) ----
// Giới hạn bởi servo 0-180°, offset 90 → IK range -90..+90
float jointLo[NUM_JOINTS] = { -90, -90, -90, -90 };
float jointHi[NUM_JOINTS] = {  90,  90,  90,  90 };

// ---- Offset:  servo_angle = ik_angle + offset ----
// Khi IK = 0° → servo = offset. Điều chỉnh theo cách lắp thực tế.
float servoOffset[NUM_JOINTS] = { 90, 180, 90, 90 };

// ---- Đảo chiều servo (true = servo lắp ngược) ----
// J2 (shoulder) lắp ngược: IK tăng → servo giảm
bool servoReverse[NUM_JOINTS] = { false, true, false, false };

// ---- State ----
float currentThetas[NUM_JOINTS] = { 0, 0, 0, 0 };  // IK degrees

// ---- Trajectory settings ----
#define LINE_STEPS      50
#define CIRCLE_STEPS    100
#define STEP_DELAY_MS   50

// ===================== MATH HELPERS =====================

static inline float deg2rad(float d) { return d * M_PI / 180.0f; }
static inline float rad2deg(float r) { return r * 180.0f / M_PI; }

// 4×4 matrix stored as float[4][4]

static void mat4_identity(float m[4][4]) {
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      m[i][j] = (i == j) ? 1.0f : 0.0f;
}

// C = A * B  (safe even when C == A)
static void mat4_mult(const float a[4][4], const float b[4][4], float c[4][4]) {
  float tmp[4][4];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      tmp[i][j] = 0;
      for (int k = 0; k < 4; k++)
        tmp[i][j] += a[i][k] * b[k][j];
    }
  memcpy(c, tmp, sizeof(float) * 16);
}

// Build one DH transformation matrix
static void dh_matrix(float theta, float d, float a, float alpha, float out[4][4]) {
  float ct = cosf(theta), st = sinf(theta);
  float ca = cosf(alpha), sa = sinf(alpha);
  out[0][0] = ct;  out[0][1] = -st * ca; out[0][2] = st * sa;  out[0][3] = a * ct;
  out[1][0] = st;  out[1][1] = ct * ca;  out[1][2] = -ct * sa; out[1][3] = a * st;
  out[2][0] = 0;   out[2][1] = sa;       out[2][2] = ca;       out[2][3] = d;
  out[3][0] = 0;   out[3][1] = 0;        out[3][2] = 0;        out[3][3] = 1;
}

// ================== FORWARD KINEMATICS ==================
// DH bảng:  (theta,   d,       a,      alpha)
//   J1:     (θ1,      BASE_H,  0,      π/2 )
//   J2:     (θ2,      0,       L1,     0   )
//   J3:     (θ3,      0,       L2,     0   )
//   J4:     (θ4,      0,       L3,     0   )

void forwardKinematics(float thetas_deg[4], float ee[3]) {
  float th[4];
  for (int i = 0; i < 4; i++) th[i] = deg2rad(thetas_deg[i]);

  float dh_params[4][4] = {
    { th[0], BASE_H, 0,     (float)(M_PI / 2.0) },
    { th[1], 0,      LINK1, 0 },
    { th[2], 0,      LINK2, 0 },
    { th[3], 0,      LINK3, 0 }
  };

  float T[4][4];
  mat4_identity(T);
  for (int i = 0; i < 4; i++) {
    float Ai[4][4];
    dh_matrix(dh_params[i][0], dh_params[i][1],
              dh_params[i][2], dh_params[i][3], Ai);
    mat4_mult(T, Ai, T);
  }

  ee[0] = T[0][3];
  ee[1] = T[1][3];
  ee[2] = T[2][3];
}

// ================== JACOBIAN (numerical) ==================

void computeJacobian(float thetas_deg[4], float J[3][4]) {
  const float delta = 0.01f;           // degrees
  float ee0[3];
  forwardKinematics(thetas_deg, ee0);

  for (int i = 0; i < 4; i++) {
    float tp[4];
    for (int j = 0; j < 4; j++) tp[j] = thetas_deg[j];
    tp[i] += delta;

    float ee1[3];
    forwardKinematics(tp, ee1);

    float drad = deg2rad(delta);
    for (int j = 0; j < 3; j++)
      J[j][i] = (ee1[j] - ee0[j]) / drad;
  }
}

// ================== 3×3 LINEAR SOLVER ==================
// Gaussian elimination with partial pivoting
// Solves  A·x = b  in-place, returns false if singular

bool solve3x3(float A[3][3], float b[3], float x[3]) {
  float aug[3][4];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) aug[i][j] = A[i][j];
    aug[i][3] = b[i];
  }

  for (int col = 0; col < 3; col++) {
    // pivot
    int best = col;
    float bestVal = fabsf(aug[col][col]);
    for (int row = col + 1; row < 3; row++) {
      if (fabsf(aug[row][col]) > bestVal) {
        bestVal = fabsf(aug[row][col]);
        best = row;
      }
    }
    if (best != col)
      for (int j = 0; j < 4; j++) {
        float t = aug[col][j]; aug[col][j] = aug[best][j]; aug[best][j] = t;
      }
    if (fabsf(aug[col][col]) < 1e-10f) return false;

    // eliminate
    for (int row = col + 1; row < 3; row++) {
      float f = aug[row][col] / aug[col][col];
      for (int j = col; j < 4; j++) aug[row][j] -= f * aug[col][j];
    }
  }

  // back-substitution
  for (int i = 2; i >= 0; i--) {
    x[i] = aug[i][3];
    for (int j = i + 1; j < 3; j++) x[i] -= aug[i][j] * x[j];
    x[i] /= aug[i][i];
  }
  return true;
}

// ================== INVERSE KINEMATICS ==================
// Damped-Least-Squares (DLS) — giống Python:
//   dθ = Jᵀ · (J·Jᵀ + λ²I)⁻¹ · e

float inverseKinematics(float target[3], float thetas[4],
                        int maxIter = 500, float tol = 0.1f,
                        float alpha = 0.4f,  float clamp = 4.0f) {
  float errNorm = 999.0f;

  for (int iter = 0; iter < maxIter; iter++) {
    float ee[3];
    forwardKinematics(thetas, ee);

    float error[3];
    for (int i = 0; i < 3; i++) error[i] = target[i] - ee[i];
    errNorm = sqrtf(error[0] * error[0] + error[1] * error[1] + error[2] * error[2]);
    if (errNorm < tol) break;

    // Jacobian 3×4
    float J[3][4];
    computeJacobian(thetas, J);

    // A = J·Jᵀ + λ²·I   (3×3)
    const float lam = 0.8f;
    float A[3][3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        A[i][j] = 0;
        for (int k = 0; k < 4; k++) A[i][j] += J[i][k] * J[j][k];
        if (i == j) A[i][j] += lam * lam;
      }

    // solve A·v = error
    float v[3];
    if (!solve3x3(A, error, v)) break;

    // dθ = Jᵀ · v  (4×1)
    for (int i = 0; i < 4; i++) {
      float dt = 0;
      for (int j = 0; j < 3; j++) dt += J[j][i] * v[j];
      float dd = rad2deg(dt) * alpha;
      if (dd >  clamp) dd =  clamp;
      if (dd < -clamp) dd = -clamp;
      thetas[i] += dd;
      if (thetas[i] < jointLo[i]) thetas[i] = jointLo[i];
      if (thetas[i] > jointHi[i]) thetas[i] = jointHi[i];
    }
    // wrap J1
    while (thetas[0] >  180) thetas[0] -= 360;
    while (thetas[0] < -180) thetas[0] += 360;
  }
  return errNorm;
}

// ===================== SERVO OUTPUT =====================

// mapf — float version of map()
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setServoAngles(float ikAngles[4]) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    float sa = ikAngles[i] + servoOffset[i];
    // Nếu servo lắp ngược, đảo chiều: 180 - sa
    if (servoReverse[i]) sa = 180.0f - sa;
    if (sa <   0) sa =   0;
    if (sa > 180) sa = 180;
    int pulse = (int)mapf(sa, 0.0f, 180.0f, (float)USMIN, (float)USMAX);
    pwm.writeMicroseconds(servoChannels[i], pulse);
  }
}

void printEE() {
  float ee[3];
  forwardKinematics(currentThetas, ee);
  Serial.print("  EE → X="); Serial.print(ee[0], 2);
  Serial.print("  Y="); Serial.print(ee[1], 2);
  Serial.print("  Z="); Serial.println(ee[2], 2);
  Serial.print("  θ = [");
  for (int i = 0; i < 4; i++) {
    Serial.print(currentThetas[i], 1);
    if (i < 3) Serial.print(", ");
  }
  Serial.println("]");
}

// ==================== TRAJECTORIES ====================

// -------- Straight line --------
void executeLine(float tx, float ty, float tz) {
  float ee0[3];
  forwardKinematics(currentThetas, ee0);

  Serial.println("===== STRAIGHT LINE =====");
  Serial.print("  From: ("); Serial.print(ee0[0], 1); Serial.print(", ");
  Serial.print(ee0[1], 1); Serial.print(", "); Serial.print(ee0[2], 1); Serial.println(")");
  Serial.print("  To:   ("); Serial.print(tx, 1); Serial.print(", ");
  Serial.print(ty, 1); Serial.print(", "); Serial.print(tz, 1); Serial.println(")");

  for (int s = 0; s <= LINE_STEPS; s++) {
    float t = (float)s / LINE_STEPS;
    float target[3] = {
      ee0[0] + (tx - ee0[0]) * t,
      ee0[1] + (ty - ee0[1]) * t,
      ee0[2] + (tz - ee0[2]) * t
    };

    float err = inverseKinematics(target, currentThetas);
    setServoAngles(currentThetas);

    if (s % 10 == 0) {
      Serial.print("  step "); Serial.print(s); Serial.print("/"); Serial.print(LINE_STEPS);
      Serial.print("  err="); Serial.println(err, 2);
    }
    delay(STEP_DELAY_MS);
  }
  Serial.println("  ✓ Line complete!");
  printEE();
}

// -------- Circle trajectory --------
void executeCircle(float xc, float yc, float zc, float r) {
  Serial.println("===== CIRCLE TRAJECTORY =====");
  Serial.print("  Center: ("); Serial.print(xc, 1); Serial.print(", ");
  Serial.print(yc, 1); Serial.print(", "); Serial.print(zc, 1); Serial.println(")");
  Serial.print("  Radius: "); Serial.println(r, 1);

  // 1) Di chuyển thẳng đến điểm bắt đầu (Xc+R, Yc, Zc)
  float startPt[3] = { xc + r, yc, zc };
  float ee0[3];
  forwardKinematics(currentThetas, ee0);

  Serial.println("  Moving to circle start...");
  for (int s = 0; s <= 30; s++) {
    float t = (float)s / 30.0f;
    float target[3] = {
      ee0[0] + (startPt[0] - ee0[0]) * t,
      ee0[1] + (startPt[1] - ee0[1]) * t,
      ee0[2] + (startPt[2] - ee0[2]) * t
    };
    inverseKinematics(target, currentThetas);
    setServoAngles(currentThetas);
    delay(STEP_DELAY_MS);
  }

  // 2) Vẽ vòng tròn
  Serial.println("  Tracing circle...");
  for (int s = 0; s <= CIRCLE_STEPS; s++) {
    float angle = ((float)s / CIRCLE_STEPS) * 2.0f * M_PI;
    float target[3] = {
      xc + r * cosf(angle),
      yc + r * sinf(angle),
      zc
    };

    float err = inverseKinematics(target, currentThetas);
    setServoAngles(currentThetas);

    if (s % 20 == 0) {
      Serial.print("  circle "); Serial.print(s); Serial.print("/"); Serial.print(CIRCLE_STEPS);
      Serial.print("  err="); Serial.println(err, 2);
    }
    delay(STEP_DELAY_MS);
  }
  Serial.println("  ✓ Circle complete!");
  printEE();
}

// -------- Home --------
void goHome() {
  float home[4] = { 0, 0, 0, 0 };
  float start[4];
  for (int i = 0; i < 4; i++) start[i] = currentThetas[i];

  Serial.println("===== GOING HOME =====");
  for (int s = 0; s <= 30; s++) {
    float t = (float)s / 30.0f;
    float te = t * t * (3.0f - 2.0f * t);            // smooth ease in-out
    for (int i = 0; i < 4; i++)
      currentThetas[i] = start[i] + (home[i] - start[i]) * te;
    setServoAngles(currentThetas);
    delay(STEP_DELAY_MS);
  }
  Serial.println("  ✓ Home reached!");
  printEE();
}

// =================== SERIAL PARSER ===================

void parseCommand(String cmd) {
  cmd.trim();

  // Tách lệnh (case-insensitive)
  String upper = cmd;
  upper.toUpperCase();

  if (upper.startsWith("LINE")) {
    float x, y, z;
    if (sscanf(cmd.c_str() + 4, "%f %f %f", &x, &y, &z) == 3) {
      executeLine(x, y, z);
    } else {
      Serial.println("⚠ Usage: LINE X Y Z");
    }
  }
  else if (upper.startsWith("CIRCLE")) {
    float xc, yc, zc, r;
    if (sscanf(cmd.c_str() + 6, "%f %f %f %f", &xc, &yc, &zc, &r) == 4) {
      executeCircle(xc, yc, zc, r);
    } else {
      Serial.println("⚠ Usage: CIRCLE Xc Yc Zc R");
    }
  }
  else if (upper.startsWith("HOME")) {
    goHome();
  }
  else if (upper.startsWith("FK")) {
    printEE();
  }
  else {
    Serial.println("─── Available Commands ───");
    Serial.println("  LINE X Y Z");
    Serial.println("  CIRCLE Xc Yc Zc R");
    Serial.println("  HOME");
    Serial.println("  FK");
    Serial.println("──────────────────────────");
  }
}

// =================== SETUP & LOOP ===================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("╔════════════════════════════════════╗");
  Serial.println("║  4-DOF Robot Arm Controller        ║");
  Serial.println("║  FK + IK + Trajectory (DH-based)   ║");
  Serial.println("╠════════════════════════════════════╣");
  Serial.println("║  LINE X Y Z      → Đường thẳng     ║");
  Serial.println("║  CIRCLE Xc Yc Zc R → Đường tròn    ║");
  Serial.println("║  HOME             → Về gốc         ║");
  Serial.println("║  FK               → Vị trí EE      ║");
  Serial.println("╚════════════════════════════════════╝");
  Serial.println();

  Wire.begin(21, 22);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Set servo về vị trí khởi tạo: servo 4,5,6,7 = 0°, 180°, 0°, 0°
  {
    float initServoAngles[NUM_SERVOS] = { 0, 180, 0, 0 };
    for (int i = 0; i < NUM_SERVOS; i++) {
      float sa = initServoAngles[i];
      if (sa < 0) sa = 0;
      if (sa > 180) sa = 180;
      int pulse = (int)mapf(sa, 0.0f, 180.0f, (float)USMIN, (float)USMAX);
      pwm.writeMicroseconds(servoChannels[i], pulse);

      // Tính ngược IK angle từ servo angle
      if (servoReverse[i])
        currentThetas[i] = (180.0f - sa) - servoOffset[i];
      else
        currentThetas[i] = sa - servoOffset[i];
    }
  }

  Serial.println("Robot ready!");
  printEE();
  Serial.println();
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.length() > 0) {
      Serial.print("> "); Serial.println(cmd);
      parseCommand(cmd);
      Serial.println();
    }
  }
}