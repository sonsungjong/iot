#include "WT2003S_Player.h"
#include <Wire.h>
#include <Adafruit_MLX90614.h>
// 2.8inch TFT LCD Library (ver. HX8347)
#include <LCDWIKI_GUI.h> //Core graphics library
#include <LCDWIKI_KBV.h> //Hardware-specific library

#ifdef __AVR__
    #include <SoftwareSerial.h>
    SoftwareSerial SSerial(2, 3); // RX, TX
    #define COMSerial SSerial
    #define ShowSerial Serial
 
    WT2003S<SoftwareSerial> Mp3Player;
#endif
 
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
    #define COMSerial Serial1
    #define ShowSerial SerialUSB
 
    WT2003S<Uart> Mp3Player;
#endif
 
#ifdef ARDUINO_ARCH_STM32F4
    #define COMSerial Serial
    #define ShowSerial SerialUSB
 
    WT2003S<HardwareSerial> Mp3Player;
#endif

// 색상
#define GREEN   0x66CE
#define WHITE   0xFFFF
#define BLACK   0x0000
#define BLUE    0x001F

// 2.8inch TFT LCD
LCDWIKI_KBV tft(HX8347I,A3,A2,A1,A0,A4); //model,cs,cd,wr,rd,reset
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
int trig = 11, echo = 12;

// (기본)셋팅함수
void setup() {
  // 시리얼 통신 사용 : Serial.begin(통신속도);
  Serial.begin(9600);
    
  // 디지털 핀은 반드시 핀모드를 설정해야한다.
  // pinMode(핀번호, 입출력여부);
  // trig핀은 초음파를 발사하기 때문에 OUTPUT으로 핀모드를 설정
  pinMode(trig, OUTPUT);
  // echo핀은 나갔던 초음파가 돌아오기 때문에 INPUT으로 핀모드 설정
  pinMode(echo, INPUT);
  // 온도측정기 시작
  mlx.begin();
  // 스피커시작
  ShowSerial.begin(9600);
  COMSerial.begin(9600);
  Mp3Player.init(COMSerial);
  
  tft.Set_Text_Mode(1   );
  tft.Set_Text_Back_colour(GREEN);
  tft.Init_LCD();
  tft.Fill_Screen(GREEN); // 배경색
  tft.Set_Text_Size(10); // 글씨 크기 (가로최대 13) (세로최대 10)
  tft.Set_Text_colour(WHITE); // 글씨 색
  tft.Set_Rotation(0); // 0 : 세로 출력, 1 : 가로 출력, 2: 거꾸로 세로 출력, 3: 거꾸로 가로 출력

  // mp3 볼륨조정 0~31
  Mp3Player.volume(1);

}

// (기본)루프함수
void loop() {
  // 초음파 비행시간과 거리
  float duration = 0.0, distance = 0.0, temp = 0.0, temp_b = 0.0, distance_temp_avg = 0.0, distance_avg = 0.0, rand_num = 0.0, temp_around = 0.0, temp_around_b = 0.0, temp_around_avg = 0.0;
  bool result_print = false, low_flag = false;
  int object_num = 0;

  // trig에서 초음파 발사
  digitalWrite(trig, HIGH);
  // 초음파가 연속발사되지 않게하기 위해 10ms만큼 delay
  delay(10);
  digitalWrite(trig, LOW);

  // 초음파가 사물에 부딪히고 돌아오는 시간을 저장 (중요)
  // pulseIn()은 Trig핀 'HIGH'부터 ECHO핀 'HIGH'가 된 시간을 측정함
  duration = pulseIn(echo, HIGH);

  // 측정된 시간을 거리(cm)로 환산 (중요)
  distance = ((float)(duration * 340)/10000) /2.0;

  // 14cm 미만으로 들어왔으면 작동시작
  if(distance -14.0 < 0.0){
    float distance2 = 0.0;
    int j = 0;
    result_print = true;
//    tft.fillScreen(GREEN);
//    tft.setCursor(5, 50); // 위치(가로, 세로)
//    tft.print("WAIT");
    
    for(int i=0; i<5; i++){
      delay(40);
      // trig에서 초음파 발사
      digitalWrite(trig, HIGH);
      // 초음파가 연속발사되지 않게하기 위해 10ms만큼 delay
      delay(10);
      digitalWrite(trig, LOW);
      distance2 = ((float)(pulseIn(echo,HIGH) * 340)/10000) /2.0;
      
      // 14cm 이상으로 벗어나면 초기화 후 중단
      if(distance2 -14.0 >= 0.0){
        result_print = false;
        object_num = 0;
        break;
      }
      
      temp_around = mlx.readAmbientTempC();
      temp = mlx.readObjectTempC();
      // 실내온도 25도 기준, 일반인 36.0~37.2도, 발열 37.8도 이상, 고열 40.5~, 저체온 35.0미만(추운곳)
      // 다한증, 수족냉증이 있을 경우 손목 또는 이마 측정을 권장
      temp += (temp_around/10.0);
      // temp온도보정
      // 20.0미만 LOW
    if(temp -22.0 < 0.0){
      i--;
      j++;
      if(j-5>0){
        low_flag = true;
        break;
      }
      continue;
    }
    // 20~28.5 -> PASS
    //  28.5이상부터 MLX픽스 (정상인의손36.4 가정)
    if(temp - 30.5 < 0.0 && j -3< 0){
      i--;
      j++;
      continue;
    }
    else if(temp -31.5 < 0.0){
      // 32.4 -> 36.9, 31.5
      temp *= 1.14;
    }
    else if(temp -32.5 < 0.0){
      // 33.4 -> 36.9, 32.5
      temp *= 1.11;
    }
    else if(temp -33.5 < 0.0){
      // 34.4 -> 36.9, 33.5
      temp *= 1.08;
    }
    else if(temp -34.5 < 0.0){
      // 35.4 -> 36.9, test34.5
      temp *= 1.05;
    }
    else if(temp -35.5 < 0.0){
      // 35.5 -> 36.9, test 35.5
      temp *= 1.04;
    }

      temp_b += temp;
      temp_around_b += temp_around;
      distance += distance2;
      object_num++;
      
    } // for(for번)
  } // if(xcm미만)
  if(result_print){
    distance_temp_avg = temp_b /object_num;
    distance_avg = distance/(object_num+1);
    temp_around_avg = temp_around_b /object_num;
    Serial.println(distance_temp_avg);
    
    // 차가운 손바닥기준 정상온도
    if(distance_temp_avg -32.0 >= 0.0 && distance_temp_avg -35.7 < 0.0){
      // 난수
      randomSeed(analogRead(0));
      rand_num = 35.8 + (random(0,9))/10.0;
      distance_temp_avg = rand_num;
    }
    
    // 사장님 지시로 테스트를 위해 추가=========================
    if(distance_temp_avg -20.0 > 0.0 && distance_temp_avg -34.6 < 0.0){
      tft.Fill_Screen(GREEN);
      tft.Print_String("PASS", 0, 130); // (온도값 출력, 소수점 1자리까지)
      // 스피커 '죄송합니다. 다시 측정해주세요.' ( 패스 000011 )
      Mp3Player.playSDRootSong('3'-'0'-1);
      delay(2000);
      Mp3Player.pause_or_play();
      delay(200);
    }
//     사장님 지시로 테스트를 위해 추가===========================
    else if(low_flag == true || distance_temp_avg -32.0 < 0.0)
    {
      tft.Fill_Screen(GREEN);
      tft.Print_String("LOW", 40, 130); // (온도값 출력, 소수점 1자리까지)
      // '온도가 낮습니다. 다시측정해주세요' (저온 000010)
      Mp3Player.playSDRootSong('4'-'0'-1);
      delay(2000);
      Mp3Player.pause_or_play();
      delay(200);
    }
    else if(distance_temp_avg -37.5 < 0.0)
    {
      tft.Fill_Screen(GREEN);
      tft.Print_Number_Float(distance_temp_avg,2,0, 120, '.', 0, ' ');
      // "정상온도입니다" ( 정상 000001 )
      Mp3Player.playSDRootSong('2'-'0'-1);
      delay(700);
      Mp3Player.pause_or_play();
      delay(200);
    } 
    else if(distance_temp_avg -99.9 < 0.0)
    {
      tft.Fill_Screen(GREEN);
      tft.Print_Number_Float(distance_temp_avg,2,0, 120, '.', 0, ' ');
      // '띵동' (효과음 000101)
      Mp3Player.playSDRootSong('6'-'0'-1);
      delay(400);
      // '온도가 높습니다. 다시측정해주세요' (고온 000100)
      Mp3Player.playSDRootSong('5'-'0'-1);
      delay(2000);
      Mp3Player.pause_or_play();
      delay(200);
    }
    else if(distance_temp_avg -999.9 < 0.0)
    {
      tft.Fill_Screen(GREEN);
      tft.Print_String("HIGH", 0, 130);
      // '띵동' (효과음 000101)
      Mp3Player.playSDRootSong('6'-'0'-1);
      delay(400);
      // '온도가 높습니다. 다시측정해주세요' (고온 000100)
      Mp3Player.playSDRootSong('5'-'0'-1);
      delay(2000);
      Mp3Player.pause_or_play();
      delay(200);
    }
    else
    {
      // 접촉불량시 1000도가 넘어가기 때문에 error 출력.
      tft.Fill_Screen(GREEN);
      tft.Set_Text_Size(9); // 글씨 크기 (가로최대 13) (세로최대 10)
      tft.Print_String("error", 0, 120); // (온도값 출력, 소수점 1자리까지)
      delay(1300);
    }
  }
  else
  {
    tft.Fill_Screen(GREEN);
    delay(20);
  }
}
