
#include "U8glib.h"

U8GLIB_NHD_C12864 u8g(13, 11, 10, 9, 8);    // SPI Com: SCK = 13, MOSI = 11, CS = 10, CD = 9, RST = 8

const int WIDTH = 128;
const int HEIGHT = 64;
const int LENGTH = WIDTH;

int x;
int y[LENGTH];

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int valueI = 0;
const int analogInPin = A1;
int analogInValue = 0;
int detect = 0;

//----------------Working Variables for Pre-Processing step of
const int N = 3;
const int Nd = 2;
int xn_buff[N + 1] = {0};
int xn_buff_WR_idx = -1;
int xn_buff_RD_idx = -1;
int number_iter = -1;
// we need to keep a history of N+1 samples for y0
int y0_buff[Nd + 1] = {0};
int y0_buff_WR_idx = -1;
int y0_buff_RD_idx = -1;

//Result variables yn
int yn = 0;
int y1 = 0;

//-------------------------------------State 1 working variables
unsigned long start_time = 0; //-------timer for state 1 of state machine
unsigned long fs_1 = 0;
unsigned long fs_1_endTime = 0;
unsigned long elapsed_t = 0;
int R_peak = 0;
int R_peak_buff = 0;
int counter_1 = 0;
int th_initial = 0;

//-------------------------------------State 2 working variables
unsigned long fs_2_duration = 0;
unsigned long RRmin = 300;
unsigned long foundTime = 0;

//-------------------------------------State 3 working variables
int counter_3 = 0;
int th_n = 0;
int th_d = 0;
int ecg_0 = 0;

//-------------------------------------Running average working variables
int P = 0;
int M = 29;
int th_ave[29] = {0};
int sum = 0;
int the_ave_idx = -1;

void setup() {
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(800);
  // Initial screen
  x = 0;
  clearY();
}

void loop() {
  // print the string when a newline arrives:
  if (stringComplete) {
    //    Serial.println(inputString);
    //    Serial.println(valueI);
    //     clear the string:
    inputString = "";
    stringComplete = false;

  }
  //  valueI = y_sig(valueI);
  drawSig();
  state_1();
  state_2();
  state_3();
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
    valueI = (inputString.toInt());

  }
}
void drawSig() {
  y[x] = map(valueI, -1, 2, HEIGHT - 1 , 0);
  u8g.firstPage();
  do {
    drawY();
  } while ( u8g.nextPage() );

  x++;
  if (x >= WIDTH) {
    x = 0;
    clearY();
  }
}
void clearY() {
  for (int i = 0; i < LENGTH; i++) {
    y[i] = -1;
  }
}

void drawY() {
  u8g.drawPixel(0, y[0]);
  for (int i = 1; i < LENGTH; i++) {
    if (y[i] != -1) {
      //            u8g.drawPixel(i, y[i]);
      u8g.drawLine(i - 1, y[i - 1], i, y[i]);
    } else {
      break;
    }
  }
}
//---------------------------------------Algorithm QRS Detect
int y_sig(int analogInValue) {
  number_iter++;
  xn_buff[xn_buff_WR_idx++] = analogInValue; // Fill up buffer with raw ecg signals
  xn_buff_WR_idx %= (Nd + 1);
  if (number_iter >= Nd) {
    xn_buff_RD_idx++;
    y0_buff[y0_buff_WR_idx++] = xn_buff[xn_buff_WR_idx] - xn_buff[xn_buff_RD_idx];     // Calculate the value of y0 if iterations > Nd
    y0_buff_WR_idx %= (N + 1);
    xn_buff_RD_idx %= (Nd + 1);
  }
  if (number_iter >= N) {
    for (int i = 0; i < N - 1 ; i++) {
      y1 += y0_buff[i] ;  // Calculate Integral
    }
  }
  y1 = y1 / (N - 1);
  yn = y1 * y1;
  //  drawSig();
  y1 = 0;
  return yn;
}
void state_1() {
  //  Serial.println("Start State 1");
  start_time = micros();
  R_peak = 0;
  //-----------------------------------Run state for 60 ms
  do {
    //    Serial.println("State 1 loop start");
    //    delayMicroseconds(2800);
    fs_1 = micros();
    R_peak_buff = y_sig(valueI);
    if (R_peak < R_peak_buff) {
      R_peak = R_peak_buff;
      foundTime = micros();
      counter_1++;
    }
    elapsed_t = fs_1 - start_time;
    //    Serial.println("state 1 loop end");
    serialEvent();
  } while (elapsed_t < 100);
  fs_1_endTime = micros();
  th_initial = th_average(R_peak);
  //  Serial.print("th_initial: ");
  //  Serial.println(th_initial);
  //  Serial.println("End State 1");
  //  Serial.print("State R_peak: ");
  //  Serial.println(R_peak);
  counter_1 = 0;
  elapsed_t = 0;
  state_2();
}
//-------------------------------------State 2: Waiting period
void state_2() {
  //  drawSig();
  //  Serial.println("Start State 2");
  fs_2_duration = RRmin - (fs_1_endTime - foundTime);
  delay(200);
  //  Serial.println("End State 2");
  //  Serial.print("FS2 Duration: ");
  //  Serial.println(fs_2_duration);
  state_3();
}
//--------------------------------------State 3: Adaptive Threshold
void state_3() {
  do {
    th_n = th_initial * exp(-6.6 / 360);
    serialEvent();
    ecg_0 = y_sig(valueI);
    //    delayMicroseconds(280);
    th_initial = th_n;
  } while (ecg_0 < th_n);
}
//--------------------------------------Average of last 30 R Peaks
int th_average(int R_peak) {
  if (P < M) {
    the_ave_idx++;
    th_ave[the_ave_idx] = R_peak;
    the_ave_idx %= (M + 1);
    for (int i = 0; i < P; i++) {
      sum += th_ave[i];
    }
    th_initial = sum / P;
    P++;
  }
  else {
    the_ave_idx++;
    th_ave[the_ave_idx] = R_peak;
    the_ave_idx %= (M + 1);
    for (int i = 0; i < M; i++) {
      sum += th_ave[i];
    }
    th_initial = sum / (M + 1);
  }
  sum = 0;
  return th_initial;
}
