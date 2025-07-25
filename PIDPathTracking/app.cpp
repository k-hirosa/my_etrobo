#include "app.h"
#include <stdio.h>
#include <inttypes.h> 

//センサー向けドライバー読み込み
#include "UltrasonicSensorWrapper.h"
#include "ColorSensorWrapper.h"
#include "IMUWrapper.h"
#include "EncoderWrapper.h"
#include "TwinMotorWrapper.h"

//自己位置推定ライブラリ読み込み
#include "RobotPose.h"
#include "SelfLocalizer.h"

//難所切り替え読み込み
#include "SwitchSceneManager.h"


//センサークラスインスタンス
UltrasonicSensorWrapper ultrasonicSensor(EPort::PORT_F); //超音波センサ
ColorSensorWrapper colorSensor(EPort::PORT_E); //カラーセンサ
IMUWrapper imu; //IMU
TwinMotorWrapper twinMotor(EPort::PORT_A, EPort::PORT_B); //両輪制御
EncoderWrapper leftEncoder(twinMotor.getLeftMotorPtr());
EncoderWrapper rightEncoder(twinMotor.getRightMotorPtr()); //エンコーダー

//自己位置推定インスタンス
float wheel_radius = 0.027;
RobotPose initial_pose(0.0, 0.0, 0.0);
SelfLocalizer selfLocalizer(wheel_radius, initial_pose, twinMotor.getRightMotorPtr(), twinMotor.getLeftMotorPtr());

//難所切り替えインスタンス
SwitchSceneManager switchSceneManager;


//一定周期で呼ばれる割り込みタスク 基本的に処理はここに書く
void timer_interrupt_task(intptr_t unused) {
    //自己位置推定更新
    selfLocalizer.updateLocalization();
    //printf("intterupt\n");
    //超音波センサ読み取り
    int32_t distance = ultrasonicSensor.measureDistance();
    //printf("Distance: %d\n", distance);
    //カラーセンサ読み取り
    colorSensor.update();
    const ColorSensor::RGB& rgb = colorSensor.getRGB();
    const ColorSensor::HSV& hsv = colorSensor.getHSV();
    //printf("RGB: R=%d, G=%d, B=%d\n", rgb.r, rgb.g, rgb.b);
    //printf("HSV: H=%d, S=%d, V=%d\n", hsv.h, hsv.s, hsv.v);
    //IMU読み取り
    imu.calcAngle();
    float yaw = imu.getAngle();
    //printf("yaw: %f\n", yaw);
    //モーター駆動
    //twinMotor.setPower(30,30);
    //エンコーダー読み取り
    int angleL = leftEncoder.getCount();
    int speedL = leftEncoder.getSpeed();
    int powerL = leftEncoder.getPower();
    //printf("left angle:%d , left speed:%d: , left power:%d \n",angleL,speedL ,powerL);
    int angleR = rightEncoder.getCount();
    int speedR = rightEncoder.getSpeed();
    int powerR = rightEncoder.getPower();
    //printf("left angle:%d , left speed:%d: , left power:%d \n",angleR,speedR ,powerR);

    //難所切り替え
    switchSceneManager.runSceneManager();

}

//起動時に呼ばれるメインタスク
void main_task(intptr_t unused) {
    //割り込みタスクの起動
    sta_cyc(TIMER_INTERRUPT_TASK_CYC);

    //タスク終了
    ext_tsk();
}
