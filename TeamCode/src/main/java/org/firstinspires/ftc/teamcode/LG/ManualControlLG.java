package org.firstinspires.ftc.teamcode.LG;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.LG.LG;

@TeleOp(name = "1 Manual Control LG")
public class ManualControlLG extends LinearOpMode {
    DcMotor slidesMotor;
    double ticks = 103.6;
    double newTarget;
    double motorpower =0.0;
    public void Init(HardwareMap hardwareMap) {
        slidesMotor = hardwareMap.get(DcMotor.class, "leftFront_leftOdometry");
        telemetry.addData("Hardware: ", "Initialized");
        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runOpMode() {


        LG theRobot;
        theRobot = new LG(true, true, this);
        TouchSensor touchSensor;
        theRobot.Init(this.hardwareMap);
        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
        Init(this.hardwareMap);




        telemetry.update();
        waitForStart();
        resetRuntime();

        try {
            while (opModeIsActive()) {

                theRobot.EndgameBuzzer();

                /* *************************************************
                 *************************************************
                 * Driver Controls (gamepad1)
                 *************************************************
                 *************************************************
                 */
                // Drive Controls uses left_stick_y, left_stick_x, and right_stick_x
                theRobot.driveControlsFieldCentric();

                if (gamepad1.triangle) {
                    theRobot.imu.resetYaw();
                }
                if (gamepad1.square) {
                    slides(-1);
                }
                else if (gamepad1.circle) {
                    slides(1);
                }
                else if (gamepad2.circle) {
                    //slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                   motorpower = 0.3;
                    telemetry.addData("Touch Sensor", "starting motor");
                }
                else if (touchSensor.isPressed()) {
                    telemetry.addData("Touch Sensor", "Is Pressed");
                    motorpower = 0.0;
                } else {
                    telemetry.addData("Touch Sensor", "Is Not Pressed");
                }
                slidesMotor.setPower(motorpower);
                telemetry.addData("Motor Ticks: ", slidesMotor.getCurrentPosition());

                telemetry.update();
            } // end while (opModeIsActive())


        } catch (Exception e) {

            // throw the exception higher for other handlers to run
            throw e;
        }

    }
    public void slides( double x){
        slidesMotor.getCurrentPosition();
        newTarget = 103.6*x;
        slidesMotor.setTargetPosition((int) newTarget);
        slidesMotor.setPower(motorpower);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


}
