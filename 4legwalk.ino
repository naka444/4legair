#include <MsTimer2.h>            // タイマー割り込みを利用する為に必要なヘッダファイル
#include <stdio.h>
#define AD0 56 //センサのピンで指定する奴じゃない本当のアドレス？あとで書き直す
#define AD1 57 //0から順にデータ配列に入る、足の番号1~3とデータ配列1～3に入るセンサーデータはそろえる
#define AD2 58  //今はA0と本来のA0はそろっていない。要は+54
#define AD3 60
#define AD4 61
#define AD5 62
#define AD6 63
#define AD7 64
#define AD8 55
#define AD9 59

//54A0、なし、55A1正圧タンク、56A2足1-弁2と3、57A3足2弁5と6,58A4足3弁7と8、59A5負圧タンク，60A6足4弁10と11，61A2足5弁12と13，62A8足6弁14と15，63A9足7弁16と17，64A10足8弁18と19
int current = 0;
int count = 0; //データログ用のカウンタ
int pin = AD0; // 初期のADCチャンネル

// グローバル変数の定義
int currentPressure = 0; // 現在の空気圧
int exhaustValveTime = 0; // 排気バルブ制御時間
int supplyValveTime = 0;  // 吸気バルブ制御時間
int rooptime = 25 ; //制御タイミング

int EXITleg[10] = {0}; // 弁のピン番号を格納する配列
int inleg[10] = {0}; //上に同じ
int invuc[4] = {0}; //上に同じ

int nontauchpuls = 25; //不感帯の幅　たぶんアナログリードだから要計算
int airroomNum = 8; // 4脚 x 2空気室 = 8
int sensorNum = 10; //使ってるセンサの数　配列に使うタンク2空気室にづつで10
//+50 必要？
//センサ殺した？
//

int date[2][15] = {{0}};  // 全ての要素を0で初期化
 //データバッファ領域
int currentTime = 0;

enum OperatingMode {
  PC_COMMAND_MODE,
  TIME_WALK_MODE,
};

OperatingMode currentMode = TIME_WALK_MODE; // デフォルトは待機モード

unsigned long timeWalkStartTime = 0;
int timeWalkTargetIndex = 0;

int targetPressure[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 各空気室の目標圧力

void setup() {
   // セットアップ
  // センサーの初期化、バルブの初期化、通信の初期化などを行う
  // タイマー割り込みを設定する
  MsTimer2::set(rooptime, flash);     // 25ms毎にflash( )割込み関数を呼び出す様に設定
  EXITleg[1] = 2; // EXITleg1にピン番号2を割り当てる（例）
  inleg[1] = 3;
  EXITleg[2] = 6; 
  inleg[2] = 5;
  EXITleg[3] = 8; 
  inleg[3] = 7;
  inleg[4] = 10;
  EXITleg[4] = 11; 
  inleg[5] = 12;
  EXITleg[5] = 13; 
  inleg[6] = 14;
  EXITleg[6] = 15;
  inleg[7] = 16;
  EXITleg[7] = 17; 
  inleg[8] = 18;
  EXITleg[8] = 19;
  invuc[1] = 4;
  invuc[2] = 9;
  invuc[3] = 20;
  invuc[4] = 21;
  // 他のピン番号の設定も同様に行う
  // 圧力計ごとにアナログピンを使う
  Serial.begin(38400);
  for (int i = 1; i <= 8; i++) {
    pinMode(EXITleg[i], OUTPUT);
    pinMode(inleg[i], OUTPUT);
  }
  for (int i = 1; i <= 4; i++){
    pinMode(invuc[i], OUTPUT);
  } 

  delay(2000);
  MsTimer2::start(); 

}

void loop() {
  // その他の処理
  switch (currentMode) {
    case PC_COMMAND_MODE:
      if (Serial.available() >= 3) {
        for (int i = 0; i < 12; i++) {
          targetPressure[i] = Serial.parseInt();//例えば50-60-70にセットするときは50 60 70と送信 吸盤ように2個追加
        }
      }
      else{
        break;
      }
      break;
    case TIME_WALK_MODE:
      if (currentTime*25  - timeWalkStartTime < 12000) {
        // 5秒ごとにtargetPressureを任意セットに切り替える
        // 5秒ずれで動作する
        if (currentTime*25 < 3000) {
          targetPressure[0] = 30 ;
          targetPressure[1] = 270 ;
          targetPressure[8] = 1;
          targetPressure[2] = 250;
          targetPressure[3] = 270 ;
          targetPressure[9] = 1;
          targetPressure[4] = 280 ;
          targetPressure[5] = 30;
          targetPressure[10] = 0;
          targetPressure[6] = 30 ;
          targetPressure[7] = 30 ;
          targetPressure[11] = 0;
        } else if (currentTime * 25 < 6000) {
          targetPressure[0] = 280 ;
          targetPressure[1] = 270 ;
          targetPressure[8] = 1;
          targetPressure[2] = 250;
          targetPressure[3] = 30 ;
          targetPressure[9] = 0;
          targetPressure[4] = 30 ;
          targetPressure[5] = 30;
          targetPressure[10] = 0;
          targetPressure[6] = 30 ;
          targetPressure[7] = 270 ;
          targetPressure[11] = 1;
        } else if (currentTime * 25 < 9000) {
          targetPressure[0] = 280 ;
          targetPressure[1] = 30 ;
          targetPressure[8] = 0;
          targetPressure[2] = 30;
          targetPressure[3] = 30 ;
          targetPressure[9] = 0;
          targetPressure[4] = 30 ;
          targetPressure[5] = 270;
          targetPressure[10] = 1;
          targetPressure[6] = 250 ;
          targetPressure[7] = 270 ;
          targetPressure[11] = 1;
        } else {
          targetPressure[0] = 60 ;
          targetPressure[1] = 60 ;
          targetPressure[8] = 0;
          targetPressure[2] = 60;
          targetPressure[3] = 270 ;
          targetPressure[9] = 1;
          targetPressure[4] = 280 ;
          targetPressure[5] = 270;
          targetPressure[10] = 1;
          targetPressure[6] = 250 ;
          targetPressure[7] = 60 ;
          targetPressure[11] = 0;
        }
      } else {
        currentTime = 0; // タイマーリセット
      }
      break;
    default:
      break;
  }
}

void flash() {
  for (int i = 0; i < sensorNum; i++)
  {
   get_current_data();
  }

  for (int i = 0; i < airroomNum; i++){

   controlAirPressure(i); // 空気圧制御を25msごとに実行 
  }
  
  digitalWrite(invuc[1], targetPressure[8]);
  digitalWrite(invuc[2], targetPressure[9]);
  digitalWrite(invuc[3], targetPressure[10]);
  digitalWrite(invuc[4], targetPressure[11]);
  currentTime = currentTime + 1;
  datelog();
}

//標準的には、
//アクチュエータ制御するスレッドor割込み処理
//センサ値を読むスレッドor割り込みルーチン
//もし AD が終わってたら、AD値を current に入れて（過去データ溜めてる配列にも入れる？）、AD開始する

void get_current_data() {
  current = analogRead(pin);

  next_channel();
  current_to_bafa();
}

void next_channel() {
  switch (pin) {
    case AD0: pin = AD1; break;
    case AD1: pin = AD2; break;
    case AD2: pin = AD3; break;
    case AD3: pin = AD4; break;
    case AD4: pin = AD5; break;
    case AD5: pin = AD6; break;
    case AD6: pin = AD7; break;
    case AD7: pin = AD8; break;
    case AD8: pin = AD9; break;
    case AD9: pin = AD0; break;
    default: pin = AD0; break;
  }
}

void current_to_bafa() {
  date[0][count] = current;
  if(count == sensorNum-1) {
    count = 0;
  }
  else{
    count = count+1;
  }
}

//データを格納する、2*センサ数の配列≒(0~1、センサ数-1)
void datelog() {
    for (int j = 0; j < sensorNum; j++) {
      Serial.print(date[1][j]);
      Serial.print("\t");
      date[1][j] = date[0][j];
    }
    Serial.println( );              // 改行の出力
}


void controlAirPressure(int i) {
  // 空気圧制御アルゴリズムを実装
  int currentPressure = date[0][i];
  int pressureDifference = targetPressure[i] - currentPressure;

  if (pressureDifference > nontauchpuls) {
    // 目標値よりも低い場合の制御
    openSupplyValve(i+1); // 吸気バルブを開く
    closeExhaustValve(i+1); // 排気バルブを閉じる
  } else if (pressureDifference < -nontauchpuls) {
    // 目標値よりも高い場合の制御
    closeSupplyValve(i+1); // 吸気バルブを閉じる
    openExhaustValve(i+1); // 排気バルブを開く
  } else {
    // 目標値に近い場合の制御
    closeSupplyValve(i+1); // 吸気バルブを閉じる
    closeExhaustValve(i+1); // 排気バルブを閉じる
  }
}