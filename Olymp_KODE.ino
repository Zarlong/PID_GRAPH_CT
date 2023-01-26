/*
   arduino library for solving problems with graphs:
   https://github.com/Ni3nayka/navigator

   author: Egor Bakay <egor_bakay@inbox.ru>
   write:  January 2023
   modify: January 2023
*/

/* MANUAL:

   direction:
     1       N       U
     |       |       |
   2-0-4   W-0-E   L-0-R
     |       |       |
     3       S       D

    NAVIGATOR_DIR_N
    NAVIGATOR_DIR_W
    NAVIGATOR_DIR_S
    NAVIGATOR_DIR_E

    NAVIGATOR_DIR_U
    NAVIGATOR_DIR_L
    NAVIGATOR_DIR_D
    NAVIGATOR_DIR_R

   coordinate system:
    +-----> x
    |
    |
    \/ y
*/

#include <Wire.h>
#include <iarduino_I2C_Motor.h>
iarduino_I2C_Motor mot_L(0x0A);
iarduino_I2C_Motor mot_R(0x0B);
#include <iarduino_I2C_Bumper.h>
iarduino_I2C_Bumper bum(0x0C);

long duration, cm;
int M = 50;

int Mp = 50;
int Ms = 50;

int speed_m_l = 50;
int speed_m_r = -50;

int sr[9];
int m = 0;
int z[6];
int qw = 6;
int q, a, w, c = 0;

int ad, qd;

int k = 750;
int dper = 500; // время на остановку после проезда перекрестка.

#define QUANTITY_POINT 35 // количесвто точек на карте (обязательно с таким именем)
#define QUANTITY_ROAD 58 // количество связей (дорог) между этими точками (обязательно с таким именем)

#include <navigator.h> // подключаем библиотеку
Navigator navigator; // создаем объект навигатора

int point_array[QUANTITY_POINT][3] = { // массив точек
  // {number_point, x_coordinate, y_coordinate},
  {1, 0, 0},
  {2, 1, 0},
  {3, 2, 0},
  {4, 3, 0},
  {5, 4, 0},
  {6, 5, 0},
  {7, 6, 0},
  {8, 0, 1},
  {9, 1, 1},
  {10, 2, 1},
  {11, 3, 1},
  {12, 4, 1},
  {13, 5, 1},
  {14, 6, 1},
  {15, 0, 2},
  {16, 1, 2},
  {17, 2, 2},
  {18, 3, 2},
  {19, 4, 2},
  {20, 5, 2},
  {21, 6, 2},
  {22, 0, 3},
  {23, 1, 3},
  {24, 2, 3},
  {25, 3, 3},
  {26, 4, 3},
  {27, 5, 3},
  {28, 6, 3},
  {29, 0, 4},
  {30, 1, 4},
  {31, 2, 4},
  {32, 3, 4},
  {33, 4, 4},
  {34, 5, 4},
  {35, 6, 4}
};

int road_array[QUANTITY_ROAD][3] = { // массив связей (дорог)
  // {point_a, point_b, long_road>0},
  {1, 2, 14},
  {2, 3, 48},
  {3, 4, 45},
  {4, 5, 29},
  {5, 6, 37},
  {6, 7, 43},
  {8, 9, 12},
  {9, 10, 41},
  {10, 11, 37},
  {11, 12, 33},
  {12, 13, 42},
  {13, 14, 27},
  {15, 16, 29},
  {16, 17, 29},
  {17, 18, 41},
  {18, 19, 48},
  {19, 20, 46},
  {20, 21, 47},
  {22, 23, 34},
  {23, 24, 29},
  {24, 25, 39},
  {25, 26, 30},
  {26, 27, 38},
  {27, 28, 23},
  {29, 30, 48},
  {30, 31, 25},
  {31, 32, 36},
  {32, 33, 37},
  {33, 34, 31},
  {34, 35, 36},

  {1, 8, 45},
  {2, 9, 37},
  {3, 10, 21},
  {4, 11, 12},
  {5, 12, 38},
  {6, 13, 24},
  {7, 14, 25},
  {8, 15, 12},
  {9, 16, 11},
  {10, 17, 20},
  {11, 18, 37},
  {12, 19, 48},
  {13, 20, 12},
  {14, 21, 40},
  {15, 22, 16},
  {16, 23, 41},
  {17, 24, 37},
  {18, 25, 43},
  {19, 26, 48},
  {20, 27, 33},
  {21, 28, 45},
  {22, 29, 14},
  {23, 30, 22},
  {24, 31, 19},
  {25, 32, 13},
  {26, 33, 40},
  {27, 34, 20},
  {28, 35, 13}
};

/*
  map:
  (number_point)
  -long_point-
  0-------------------> x
  | (1)     (2)--1--(3)
  |  |       |       |
  |  1       1       1
  |  |       |       |
  | (4)--1--(5)     (6)
  |  |       |       |
  |  1       3      100
  |  |       |       |
  | (70)-1--(8)--10-(9)
  \/ y
*/

void setup() {
  // запустим монитр порта
  Serial.begin(9600);
  delay(500);
  mot_L.begin();
  mot_R.begin();
  mot_R.setDirection(false);
  mot_L.setDirection(true);
  bum.setTurnPeriod(250);
  pinMode(3, OUTPUT);

  bum.begin();
  motor(40,40);
  delay(2000);
  motor(0,0);
  while (bum.getLineAnalog (9) > 1500) {
    int P = (bum.getLineAnalog (8) - bum.getLineAnalog (2)) ;
    int z = P * -0.1;
    motor (M - z, M + z);
    bum.setTurnSignal(BUM_TURN_OFF);
  }
  motor(0, 0);
  delay(1000);
  motor(20, 20);
  delay(3000);
  motor(0, 0);
  delay(1000);
  kod();
  Serial.println(w);
  Serial.println(q);
  motor(20, 20);
  delay(2000);
  motor(0, 0);
  unsigned long int tt = millis();
  while (tt + 2000 > millis()) {
    int P = (bum.getLineAnalog (8) - bum.getLineAnalog (2)) ;
    int z = P * -0.1;
    motor (M - z, M + z);
  }
  forward();



  // добавить в навигатор карту
  navigator.set_point(point_array); // добавить в навигатор массив точек (обязательно перед всеми остальными опциями)
  navigator.set_road(road_array); // добавить в навигатор массив связей (дорог) (обязательно перед всеми остальными опциями)

  // задать точку старта и направление
  navigator.set_start_coordinate(0, 2, NAVIGATOR_DIR_R); // в координатах
  //navigator.set_start_point(1,NAVIGATOR_DIR_R) // по номеру точки

  // задать точку финиша и направление (не обязательно задавать навправление)
  navigator.set_finish_coordinate(w-1, q-1); // в координатах
  //navigator.set_finish_coordinate(2,2); // в координатах
  //navigator.set_finish_point(9,NAVIGATOR_DIR_U); // по номеру точки
  //navigator.set_finish_point(9); // по номеру точки

  // если необходимо отъехать от точки "старта", мы можем отметить это в навигаторе для анализа движений из другой точки:




  // задать точку финиша и направление (не обязательно задавать навправление)

  // т.е. мы приехали в точку 2, направление NAVIGATOR_DIR_U, и отсюда можем начать анализ

  // сделать анализ необходимых движений (чтобы из нашего местоположения доехать до точки финиша)
  navigator.operating();
  //navigator.operating_long_road(); // если нам нужно просчитать только расстояния до точек, без просчета движений


  // вывести наиктратчайшие расстояния до всех точек
  Serial.println("long to points:");
  for (int i = 0; i < QUANTITY_POINT; i++) {
    int number_point = point_array[i][0];
    int long_road_to_point = navigator.get_long_road_to_point(number_point);
    Serial.print("point: ");
    Serial.print(number_point);
    Serial.print("  long to point: ");
    Serial.println(long_road_to_point);
  }

  Serial.println("movies:");
  while (navigator.this_is_finish() == 0) { // пока мы не достигли финиша
    int t = navigator.next_move(); // запрашиваем следующее действие
    if (t == NAVIGATOR_END) { // если доехали до конца (в данном примере это не будет вызвано, т.к. как только достигнем финиша выйдем из цикла)

    }
    if (t == NAVIGATOR_MOVE_FORWARD) { // двигаться вперед
      forward();
    }
    if (t == NAVIGATOR_MOVE_LEFT) { // повернуть налево
      left();
    }
    if (t == NAVIGATOR_MOVE_RIGHT) { // повернуть направо
      right();
    }
    if (t == NAVIGATOR_MOVE_AROUND) { // развернуться
      left();
      left();
    }
  }

  analogWrite(3, 50);
  delay(500);
  analogWrite(3, 0);
  delay(500);
  analogWrite(3, 50);
  delay(500);
  analogWrite(3, 0);


  
  navigator.set_finish_coordinate(0, 2, NAVIGATOR_DIR_L); // в координатах
  navigator.operating();
  //navigator.operating_long_road(); // если нам нужно просчитать только расстояния до точек, без просчета движений

  // вывести наиктратчайшие расстояния до всех точек
  Serial.println("long to points:");
  for (int i = 0; i < QUANTITY_POINT; i++) {
    int number_point = point_array[i][0];
    int long_road_to_point = navigator.get_long_road_to_point(number_point);
    Serial.print("point: ");
    Serial.print(number_point);
    Serial.print("  long to point: ");
    Serial.println(long_road_to_point);
  }

  Serial.println("movies:");
  while (navigator.this_is_finish() == 0) { // пока мы не достигли финиша
    int t = navigator.next_move(); // запрашиваем следующее действие
    if (t == NAVIGATOR_END) { // если доехали до конца (в данном примере это не будет вызвано, т.к. как только достигнем финиша выйдем из цикла)

    }
    if (t == NAVIGATOR_MOVE_FORWARD) { // двигаться вперед
      forward();
    }
    if (t == NAVIGATOR_MOVE_LEFT) { // повернуть налево
      left();
    }
    if (t == NAVIGATOR_MOVE_RIGHT) { // повернуть направо
      right();
    }
    if (t == NAVIGATOR_MOVE_AROUND) { // развернуться
      left();
      left();
    }
  }
  tt = millis();
  while (tt + 2000 > millis()) {
    int P = (bum.getLineAnalog (8) - bum.getLineAnalog (2)) ;
    int z = P * -0.1;
    motor (M - z, M + z);
  }
  motor(40,40);
  delay(5000);
  

  unsigned long int rr = millis();
  while (millis()-rr < 3000) {
    if ( bum.getLineAnalog (5) < k){
      rr = millis();
    }
  }
  motor(0,0);
}

void loop() {}

void kod () {
  for (int r = 0; r < 7; ++r) {
    motor (40, 40);
    delay (450);
    motor (0, 0);
    delay(1000);

    for (int i = 0; i < 10; ++i) {
      sr[i] = bum.getLineAnalog (9);
      for (int j = 0; j < 8; ++j) {
        sr[j + 1] = sr[j];
      }
    }
    for (int i = 0; i < 9; ++i) {
      m = m + sr[i];
    }
    m = m / 9;
    if (m < 1500) { // пороги
      m = 1;
    }
    else {
      m = 0;
    }
    z[r] = m;
  }
  akod();
  Serial.println(z[0]);
  Serial.println(z[1]);
  Serial.println(z[2]);
  Serial.println(z[3]);
  Serial.println(z[4]);
  Serial.println(z[5]);
  Serial.println(z[6]);

  w = a / 10;
  q = a % 10;
  Serial.println(a);
  Serial.println(w);
  Serial.println(q);
  Serial.println(" mmm ");
}
int akod() {
  a = 0;
  c = 1;
  Serial.println("------------");
  for (int i = 0; i < 7; i++) {
    if (z[i] == 1 ) {
      a += c;
    }
    //    a =z[i] * c;
    c = c * 2;
    Serial.println(a);
  }
  Serial.println("----------");
  return a;
}

void motor(int l, int r) { // движение
  mot_L.setSpeed( l, MOT_PWM);
  mot_R.setSpeed( r, MOT_PWM);
}
void forward() { // вперед
  while (bum.getLineAnalog (1) > k && bum.getLineAnalog (9) > k ) {
    int P = (bum.getLineAnalog (8) - bum.getLineAnalog (2)) ;
    int z = P * -0.1;
    motor (M - z, M + z);
    bum.setTurnSignal(BUM_TURN_OFF);
  }
  motor (Ms, Ms);
  delay (dper);
  unsigned long int time = millis();
  while (time + 335 > millis ()) {
    int P = (bum.getLineAnalog (8) - bum.getLineAnalog (2)) ;
    int z = P * -0.1;
    motor (M - z, M + z);
    bum.setTurnSignal(BUM_TURN_OFF);
  }
  motor (0, 0);
}
void right () {  //вправо
  while (bum.getLineAnalog (8) < k) {
    motor (-Mp, Mp);
    bum.setTurnSignal(BUM_TURN_RIGHT);
  }
  while (bum.getLineAnalog (8) > k) {
    motor (-Mp, Mp);
    bum.setTurnSignal(BUM_TURN_RIGHT);
  }
  while (bum.getLineAnalog (8) < k) {
    motor (-Mp, Mp);
    bum.setTurnSignal(BUM_TURN_RIGHT);

  }
  motor(0, 0);
}

void left () { // лево
  while (bum.getLineAnalog (2) < k) {
    motor (Mp, -Mp);
    bum.setTurnSignal(BUM_TURN_LEFT);
  }
  while (bum.getLineAnalog (2) > k) {
    motor (Mp, -Mp);
    bum.setTurnSignal(BUM_TURN_LEFT);
  }
  while (bum.getLineAnalog (2) < k) {
    motor (Mp, -Mp);
    bum.setTurnSignal(BUM_TURN_LEFT);
  }
  motor(0, 0);
}
