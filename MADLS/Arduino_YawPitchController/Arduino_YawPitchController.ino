#include <math.h>

class LinearActuator {
private:
  int pinOut, pinIn;
  float minStroke, maxStroke, potInit, potFinal;

public:
  LinearActuator(int pinOut, int pinIn, float minStroke, float maxStroke) {
    this->pinOut = pinOut;
    this->pinIn = pinIn;
    this->minStroke = minStroke;
    this->maxStroke = maxStroke;
  }

  void calibrate() {
    analogWrite(pinOut, 0);
    delay(2000);
    this->potInit = analogRead(pinIn);
    Serial.print("potInit: ");
    Serial.println(potInit);
    delay(100);

    analogWrite(pinOut, 255);
    delay(2000);
    this->potFinal = analogRead(pinIn);
    Serial.print("potFinal: ");
    Serial.println(potFinal);
    delay(100);

    analogWrite(pinOut, 0);
  }

  void setPosition(float pos) {
    float analogPos = (pos / maxStroke) * 255;
    analogWrite(pinOut, analogPos);
  }
};

class StewartPlatform4 {
  private:
    LinearActuator &la1, &la2, &la3, &la4;
    float platformWidth, actuatorAngle, centroid[3], b1[3], b2[3], b3[3], b4[3];
    float cosA1, sinA1, cosA2, sinA2, cosA3, sinA3, cosA4, sinA4; 
    struct trig;
    const float s22 = sqrt(2) / 2;

  public:
    StewartPlatform4(LinearActuator& la1, LinearActuator& la2, LinearActuator& la3, LinearActuator& la4, float laA1, float laA2, float laA3, float laA4, float* centroidIn)
    : la1(la1), la2(la2), la3(la3), la4(la4) {
      this->la1 = la1;
      this->la2 = la2;
      this->la3 = la3;
      this->la4 = la4;
      this->actuatorAngle = actuatorAngle;
      for (int i=0; i<3; i++) {
        centroid[i] = centroidIn[i];
      }

      this->b1[0] = 11.95;
      this->b1[1] = 11.95;
      this->b1[2] = 0;

      this->b2[0] = -8.87;
      this->b2[1] = 8.87;
      this->b2[2] = 0;

      this->b3[0] = -11.95;
      this->b3[1] = -11.95;
      this->b3[2] = 0;

      this->b4[0] = 8.87;
      this->b4[1] = -8.87;
      this->b4[2] = 0;

      this->cosA1 = cosd(laA1);
      this->sinA1 = sind(laA1);
      this->cosA2 = cosd(laA2);
      this->sinA2 = sind(laA2);
      this->cosA3 = cosd(laA3);
      this->sinA3 = sind(laA3);
      this->cosA4 = cosd(laA4);
      this->sinA4 = sind(laA4);
    }

    float vDot(const float* a, const float* b) {
      float result = 0;
      for (int i=0; i<3; i++) {
        result += a[i] * b[i];
      }
      return result;
    }

    float* vAdd(const float* a, const float* b) {
      static float result[3];
      for (int i=0; i<3; i++) result[i] = a[i] + b[i];
      return result;
    }

    float* vSub(const float* a, const float* b) {
      static float result[3];
      for (int i = 0; i < 3; i++) result[i] = a[i] - b[i];
      return result;
    }

    float sind(float deg) {
      return sin(deg * PI / 180.0);
    }

    float cosd(float deg) {
      return cos(deg * PI / 180.0);
    }

    void setYawPitch(float yaw, float p, float* centroidIn) {
      float offsetY = 15.1;
      float pitch = 90 - p;

      float v[3] = { sind(yaw)*cosd(pitch), cosd(yaw)*cosd(pitch), sind(pitch) };
      
      float d1[3] = {-s22*cosA1, -s22*cosA1, sinA1};
      float l1 = vDot(v, vSub(centroidIn, b1)) / vDot(v, d1) + offsetY/sinA1;

      float d2[3] = {s22*cosA2, -s22*cosA2, sinA2};
      float l2 = vDot(v, vSub(centroidIn, b2)) / vDot(v, d2);

      float d3[3] = {s22*cosA3, s22*cosA3, sinA3};
      float l3 = vDot(v, vSub(centroidIn, b3)) / vDot(v, d3) + offsetY/sinA1;

      float d4[3] = {-s22*cosA4, s22*cosA4, sinA4};
      float l4 = vDot(v, vSub(centroidIn, b4)) / vDot(v, d4);

      // Serial.print(l1);
      // Serial.print(", ");
      // Serial.print(l2);
      // Serial.print(", ");
      // Serial.print(l3);
      // Serial.print(", ");
      // Serial.println(l4);

      la1.setPosition(l1);
      la2.setPosition(l2);
      la3.setPosition(l3);
      la4.setPosition(l4);
    }
};

LinearActuator la1 = LinearActuator(3, A4, 0, 30);
LinearActuator la2 = LinearActuator(5, A5, 0, 30);
LinearActuator la3 = LinearActuator(9, A2, 0, 30);
LinearActuator la4 = LinearActuator(10, A3, 0, 30);

float centroid[3] = {0, 0, 6};

StewartPlatform4 sp = StewartPlatform4(la1, la2, la3, la4, 70, 66.75, 70, 66.75, centroid);

int actuatorNum;
float setPoint, yaw, pitch, centroidX, centroidY, centroidZ;

float curYaw = 0.0;
float curPitch = 0.0;
float adjustmentSpeed = 10;

char msg[10];
float divisor = 1000.;

void setup() {
  Serial.begin(9600);

  delay(1000);

  Serial.println("Serial connected.");
  
  sp.setYawPitch(0, 0, centroid);

  pinMode(13, OUTPUT);
  delay(1000);
}

void loop() {
  if (Serial.available()) {
    int bytesRead = Serial.readBytesUntil('\n', msg, sizeof(msg) - 1);
    msg[bytesRead] = '\0';

    char flag = msg[0];

    float value = atof(&msg[1]) / divisor;

    switch (flag){
      case 'a':
        centroid[0] = value;
        digitalWrite(13, HIGH);
        break;
      case 'b':
        centroid[1] = value;
        break;
      case 'c':
        centroid[2] = value;
        break;
      case 'd':
        curYaw = value;
        break;
      case 'e':
        curPitch = value;
        break;
    }
    sp.setYawPitch(yaw, pitch, centroid);
  }
}