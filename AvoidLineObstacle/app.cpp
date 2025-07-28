#include "app.h"
#include <stdio.h>
#include <inttypes.h>
#include <csignal>

//センサー向けドライバー読み込み
#include "UltrasonicSensorWrapper.h"
#include "ColorSensorWrapper.h"
#include "IMUWrapper.h"
#include "EncoderWrapper.h"
#include "TwinMotorWrapper.h"

//自己位置推定ライブラリ読み込み
#include "RobotPose.h"
#include "SelfLocalizer.h"

#include "PIDPathTracking.h"
#include "LocalGoalCreator.h"
#include "Logger.h"


//センサークラスインスタンス
UltrasonicSensorWrapper ultrasonicSensor(EPort::PORT_F); //超音波センサ
ColorSensorWrapper colorSensor(EPort::PORT_E); //カラーセンサ
IMUWrapper imu; //IMU
TwinMotorWrapper twinMotor(EPort::PORT_A, EPort::PORT_B); //両輪制御

// 初期化後に代入するポインタ
EncoderWrapper* leftEncoder = nullptr;
EncoderWrapper* rightEncoder = nullptr;
SelfLocalizer* selfLocalizer = nullptr;


PIDPathTracking pidPathTracking;
LocalGoalCreator localGoalCreator;
PathPoint local_goal;
PWM pwm;
Logger logger;
static int count = 0;
static int save_log_cycle = 1000; 

// 一定周期で呼ばれる割り込みタスク
void timer_interrupt_task(intptr_t unused) {
    if (!selfLocalizer) return;

    selfLocalizer->updateLocalization();
    // printf("Localization updated: x=%.2f, y=%.2f, theta=%.2f\n",
        //    selfLocalizer->getLocalization().m_x,
        //    selfLocalizer->getLocalization().m_y,
        //    selfLocalizer->getLocalization().m_yaw);

    local_goal = localGoalCreator.createLocalGoalRobotFrame(
        selfLocalizer->getLocalization().m_x,
        selfLocalizer->getLocalization().m_y,
        selfLocalizer->getLocalization().m_yaw,
        0.1 // 距離閾値
    );
    // printf("Local goal created: x=%.2f, y=%.2f\n", local_goal.x, local_goal.y);
    pwm = pidPathTracking.computePWM(local_goal.x, local_goal.y);
    // printf("Computed PWM: left=%.2f, right=%.2f\n", pwm.left, pwm.right);

    twinMotor.setPower(pwm.left, pwm.right);
    count++;

    logger.addLog(0.0, 1.0, 2.0, 3.14);
    /* printf("%.2f,%.2f,%.2f,%.2f\n", 0.0, 1.0, 2.0, 3.14); */
    if (count % save_log_cycle == 0) 
    {
        /* logger.saveCsv("./log.csv"); */
        syslog(LOG_NOTICE, "CSV saved.");
    }

    // logger.addLog(0.0, 1.0, 2.0, 3.14);

    // logger.saveCsv("/log/log.csv");
    // センサ読み取り例（コメントアウトで調整）
    // int32_t distance = ultrasonicSensor.measureDistance();
    // colorSensor.update();
    // imu.calcAngle();

    // モータ・エンコーダ使用例
    // if (leftEncoder && rightEncoder) {
    //     int angleL = leftEncoder->getCount();
    //     int angleR = rightEncoder->getCount();
    // }
}

// 初期化専用周期タスク
void init_task(intptr_t unused) {
    if (twinMotor.getLeftMotorPtr() && twinMotor.getRightMotorPtr()) {
        // printf("Motor initialized!\n");

        // 動的確保してグローバルポインタに設定
        leftEncoder = new EncoderWrapper(twinMotor.getLeftMotorPtr());
        rightEncoder = new EncoderWrapper(twinMotor.getRightMotorPtr());

        RobotPose initial_pose{0, 0, 0};
        float wheel_radius = 0.026; // 例：半径26mm
        selfLocalizer = new SelfLocalizer(wheel_radius, initial_pose,
                                          twinMotor.getRightMotorPtr(),
                                          twinMotor.getLeftMotorPtr());

        // 割り込み周期タスク起動
        sta_cyc(TIMER_INTERRUPT_TASK_CYC);

        // 自分（init_task）はもう不要なので終了
        ext_tsk();
    } else {
        // printf("Waiting for motor...\n");
        // 次回周期で再度チェック
    }
}

// 起動時に呼ばれるメインタスク
void main_task(intptr_t unused) {
    // init_taskを周期起動にしておけば、ここでは何もしなくてOK
    sta_cyc(INIT_TASK_CYC);    
    // logger.saveCsv("/log/log.csv");
    // logger.addLog(0.0, 1.0, 2.0, 3.14);
    // logger.saveCsv("/log/log.csv");
    // tslp_tsk(100);  // 書き込み処理完了待機（適宜調整）
    ext_tsk();
}
