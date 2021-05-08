#include "main.h"
#include <array>

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

pros::ADIEncoder rightEnc(RightEncTop, RightEncBot);
pros::ADIEncoder leftEnc(LeftEncTop, LeftEncBot);
pros::ADIEncoder horEnc(HorEncTop, HorEncBot, true);

Chassis newChassis{ 2.75, 12.75, 0.5 };
Sensor_vals valStorage{ 0, 0, 0, true };

ThreeTrackerOdom odomSys(newChassis);

PIDConsts straight{ 17, 0, 0, 0 };
PIDConsts turn{ 160, 0, 0, 0 };
PIDController driveCont(straight);
PIDController turnCont(turn);

XDrive newX({ FrontRightWheelPort, BackRightWheelPort }, { -FrontLeftWheelPort, -BackLeftWheelPort });

PursuitController chassisController(&newX, &odomSys, &driveCont, &turnCont);

PathFollower Rohith(&chassisController);

pros::Vision camera(visionPort, pros::E_VISION_ZERO_CENTER);
pros::Imu inertial(12);


double convertToRadians(double input) {

    return input / 180 * 3.14159;
}

void robotTask(void* p) {
}

void odomTask(void* p) {

    /*
    lv_obj_t* chart = lv_chart_create(lv_scr_act(), NULL);

    lv_obj_set_size(chart, 200, 150);
    lv_obj_align(chart, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);

    lv_chart_series_t* ser1 = lv_chart_add_series(chart, LV_COLOR_RED);
    lv_chart_series_t* ser2 = lv_chart_add_series(chart, LV_COLOR_GREEN);
    lv_chart_series_t* ser3 = lv_chart_add_series(chart, LV_COLOR_YELLOW);
    */

    lv_obj_t* label = lv_label_create(lv_scr_act(), NULL);
    lv_obj_t* label2 = lv_label_create(lv_scr_act(), label);
    lv_obj_t* label3 = lv_label_create(lv_scr_act(), label);

    lv_obj_set_pos(label2, 0, 50);
    lv_obj_set_pos(label3, 0, 100);

    //Assuming pros::delay(20);
    Matrix F({
        {1, 0, 0.02, 0},
        {0, 1, 0, 0.02},
        {0, 0, 0.7, 0},
        {0, 0, 0, 0.7}
        });
    Matrix G({
        {0, 0},
        {0, 0},
        {7, 0},
        {0, -7}
        });
    Matrix H({
        {1, 0, 0, 0},
        {0, 1, 0, 0}
        });
    Matrix Q({
        {0, 0, 1, 0},
        {0, 0, 0, 1},
        {1, 0, 3, 0},
        {0, 1, 0, 3}
        });
    Matrix R({
        {1000, 0},
        {0, 1000}
        });
    Matrix Pzero({
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
        });

    KalmanFilter odomFilter(F, G, H, Q, R, Pzero, 4);

    Matrix results;

    odomFilter.setState({ 0, 0, 0, 0 });

    pros::delay(2500);
    double curAx = 0;
    double curAy = 0;
    
    double lastax = 0;
    double lastay = 0;

    double loop = 0;

    while (true) {
        pros::c::imu_accel_s_t accelData = inertial.get_accel();
        
        double differenceX = accelData.x - lastax;
        double differenceY = accelData.y - lastay;

        if (loop > 2) {
            curAx += differenceX;
            curAx *= 0.97;
            curAy += differenceY;
            curAy *= 0.97;
        }
        loop++;
        lastax = accelData.x;
        lastay = accelData.y;
       
        results = odomFilter.step({ curAx, curAy }, { 0, 0 });

        
        //std::string text3 = "last: " + std::to_string(lastax);
        //std::string text2 = "Delta ax: " + std::to_string(difference);
        //std::string text = "measured: " + std::to_string(curAx);
        
        std::string text = "x: " + std::to_string(results(0, 0)) + " y: " + std::to_string(results(1, 0));

        lv_label_set_text(label, text.c_str());

        pros::delay(20);
    }

    /*
    while (true) {
        pros::c::imu_accel_s_t accelData = inertial.get_accel();
        std::string text = "ax: " + std::to_string(accelData.x) + " ay: " + std::to_string(accelData.y) +
            " az: " + std::to_string(accelData.z);

        lv_label_set_text(label, text.c_str());

        pros::delay(20);
    }
    */
    /*
    std::array<int, 3> tickDiffs;
    OdomDebug display(lv_scr_act(), LV_COLOR_ORANGE);

    //odomSys.setState({0, 0, convertToRadians(90)});

    while (true) {
        int LVal = leftEnc.get_value(); int RVal = rightEnc.get_value(); int HVal = horEnc.get_value();
        int LDiff = LVal - valStorage.left;
        int RDiff = RVal - valStorage.right;
        int HDiff = HVal - valStorage.middle;
        tickDiffs = { LDiff, RDiff, HDiff };
        valStorage.setVals(RVal, LVal, HVal);

        odomSys.odomStep(tickDiffs);
        display.setData(odomSys.getState(), valStorage);

        pros::delay(20);
    }
    */
}

void opcontrol() {
    pros::Task dispOdom(odomTask);
    pros::Task runRobot(robotTask);
}
