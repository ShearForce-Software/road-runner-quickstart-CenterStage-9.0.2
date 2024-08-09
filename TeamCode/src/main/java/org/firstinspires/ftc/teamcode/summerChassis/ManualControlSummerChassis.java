package org.firstinspires.ftc.teamcode.summerChassis;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.summerChassis.SummerChassis;

@TeleOp(name = "1 Manual Control Summer Chassis")
public class ManualControlSummerChassis extends LinearOpMode {

    double ticks = 103.6;


    public void runOpMode() {


        SummerChassis theRobot;
        theRobot = new SummerChassis(true, true, this);
        TouchSensor touchSensor;
        theRobot.Init(this.hardwareMap);
        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");





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
                    theRobot.setslidePower(1);
                }
                else if (gamepad1.circle) {
                    theRobot.setslidePower(-1);
                }
                else if (gamepad2.circle) {
                    //slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    theRobot.slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    theRobot.motorpower = 0.1;
                    telemetry.addData("Touch Sensor", "starting motor");
                }
                else if (touchSensor.isPressed()) {
                    telemetry.addData("Touch Sensor", "Is Pressed");
                    theRobot.motorpower = 0.0;
                } else {
                    telemetry.addData("Touch Sensor", "Is Not Pressed");
                    theRobot.setslidePower(0);
                }
               // theRobot.slidesMotor.setPower(theRobot.motorpower);
                telemetry.addData("Motor Ticks: ", theRobot.slidesMotor.getCurrentPosition());

                telemetry.update();
            } // end while (opModeIsActive())


        } catch (Exception e) {

            // throw the exception higher for other handlers to run
            throw e;
        }

    }



}
