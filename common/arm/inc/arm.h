#define ARM_NUM 2
#define UPPER_ARM_R_RANGE 665.0   //ToDo: 上側アームのアーム長可動域(mm)
#define UPPER_ARM_R_MIN 415.0     //ToDo: アーム長を一番短くしたときのR(mm)を測る
#define UPPER_ARM_DEG_RANGE 180.0 //偏角の可動域(degree)
#define UPPER_ARM_DEG_MIN 180.0     //偏角の下限。上のアームはハンドの取り付け側が下のアームと逆(180度回転してついている)ため，下のアームと違い0度にならない

#define UPPER_ARM_Z_SERVO_GEAR_DIAMETER 80  //mm
#define UPPER_ARM_Z_MIN 0.0                 //Zの下限(mm)
#define UPPER_ARM_Z_RANGE 174                //Zの可動域(mm)

#define LOWER_ARM_R_RANGE 834.0   //ToDo: 下側アームのアーム長可動域(mm)
#define LOWER_ARM_R_MIN 342.0     //ToDo: アーム長を一番短くしたときのR(mm)を測る
#define LOWER_ARM_DEG_RANGE 180.0 //偏角の可動域(degree)
#define LOWER_ARM_DEG_MIN 0.0     //偏角の下限


#define R_ROBOMAS_DIAMETER 30.0 //アーム長ロボマスにつくギアの直径(mm)

#define POLAR_RATIO (4.0/1.0) //アーム軸/モーター軸　直径比

// 出場チーム(青/赤)。競技開始前に決定した後は変更しない。
// 緊急停止スイッチが押されるとESP32/STM32の各基板は再起動するため，実行時にトグルする
// 方式では状態を保持できない。そのため，チームに応じてこの値を書き換えてビルド・
// 書き込みすることで固定する。
// このarm.hを使う全ファームウェア(ESP32 Main Controller，STM32の
// robomas_controller/lower_arm_servo/upper_arm_servo)で必ず同じ値にすること。
// (値がずれると，DEG軸のホーミングで検出するリミットスイッチと回転方向が
//  ファームウェア間で食い違い，可動域の反対側の固定端に衝突する恐れがある。)
#define ROBOT_TEAM_BLUE 0
#define ROBOT_TEAM_RED  1
#define ROBOT_TEAM ROBOT_TEAM_RED  //出場チームに応じて書き換えてビルドする

// 赤/青チームは，フィールドが鏡合わせのため整理機構を左右逆側に取り付け直す必要があり，
// それに合わせて上下アームとも「DEG軸(偏角)のどちら側の可動端をホーミング原点にするか」を
// 切り替える(実際に機体やロボマスの配線が反転するわけではない)。
// 赤チームは可動域下限(*_DEG_UNDER_LIMIT，整理機構は左側)，
// 青チームは可動域上限(*_DEG_OVER_LIMIT，整理機構は右側)を原点にする。
// LOWER/UPPER_ARM_DEG_HOME_DEGはこの原点に対応する偏角(度)。DEG_MIN側を原点にする場合は
// そのままDEG_MIN，DEG_MIN+RANGE側(可動域の反対側の端)を原点にする場合はDEG_MIN+RANGEを
// 使う(360度になっても0度に正規化しない。coordinate.hのto_polar()は[0,2π)にラップして
// 返すため，そのまま比較・減算する側で360度をまたぐ場合の連続性を考慮する必要がある。
// STM32/robomas_controllerのpolar_deg_unwrapped()，STM32/upper_arm_servoの
// 偏角正規化処理を参照)。
#if ROBOT_TEAM == ROBOT_TEAM_BLUE
#define LOWER_ARM_DEG_HOME_DEG (LOWER_ARM_DEG_MIN + LOWER_ARM_DEG_RANGE)
#define UPPER_ARM_DEG_HOME_DEG (UPPER_ARM_DEG_MIN + UPPER_ARM_DEG_RANGE)
#else
#define LOWER_ARM_DEG_HOME_DEG (LOWER_ARM_DEG_MIN)
#define UPPER_ARM_DEG_HOME_DEG (UPPER_ARM_DEG_MIN)
#endif

// ホーミング原点(LOWER/UPPER_ARM_DEG_HOME_DEG)に対応する座標。
#if ROBOT_TEAM == ROBOT_TEAM_BLUE
#define LOWER_ARM_HOME_COORDINATE {\
  .x = -LOWER_ARM_R_MIN,\
  .y = 0,\
}
#define UPPER_ARM_HOME_COORDINATE {\
  .x = UPPER_ARM_R_MIN,\
  .y = 0,\
}
#else
#define LOWER_ARM_HOME_COORDINATE {\
  .x = LOWER_ARM_R_MIN,\
  .y = 0,\
}
#define UPPER_ARM_HOME_COORDINATE {\
  .x = -UPPER_ARM_R_MIN,\
  .y = 0,\
}
#endif

//HOMING
#define HOMING_UPPER_ARM_TIMEOUT_MS 10000
#define HOMING_LOWER_ARM_TIMEOUT_MS 10000

#define HOMING_TIMEOUT_MARGIN_RATE 0.6

//                                                                  rotation   per  second
#define HOMING_UPPER_DEG_RPM  ((UPPER_ARM_DEG_RANGE / 360 * POLAR_RATIO)        /   (HOMING_UPPER_ARM_TIMEOUT_MS/1000.0/60.0 * HOMING_TIMEOUT_MARGIN_RATE))
#define HOMING_LOWER_DEG_RPM  ((LOWER_ARM_DEG_RANGE / 360 * POLAR_RATIO)        /   (HOMING_LOWER_ARM_TIMEOUT_MS/1000.0/60.0 * HOMING_TIMEOUT_MARGIN_RATE))
#define HOMING_UPPER_R_RPM    ((UPPER_ARM_R_RANGE/(R_ROBOMAS_DIAMETER * M_PI))  /   (HOMING_UPPER_ARM_TIMEOUT_MS/1000.0/60.0 * HOMING_TIMEOUT_MARGIN_RATE))
#define HOMING_LOWER_R_RPM    ((LOWER_ARM_R_RANGE/(R_ROBOMAS_DIAMETER * M_PI))  /   (HOMING_LOWER_ARM_TIMEOUT_MS/1000.0/60.0 * HOMING_TIMEOUT_MARGIN_RATE))
