package org.firstinspires.ftc.teamcode.LG;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.LG.LG;

@TeleOp(name = "LG Manual Control")
public class ManualControlLG extends LinearOpMode {

  /*  public void Init(HardwareMap hardwareMap) {
        slidesMotor = hardwareMap.get(DcMotor.class, "leftFront_leftOdometry");
        telemetry.addData("Hardware: ", "Initialized");
        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
*/
    public void runOpMode() {


        LG theRobot;
        theRobot = new LG(true, true, this);
        TouchSensor touchSensor;
        theRobot.Init(this.hardwareMap);
        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
        //Init(this.hardwareMap);




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
                    theRobot.slides(-1);
                }
                else if (gamepad1.circle) {
                    theRobot.slides(1);
                }
                else if (gamepad2.circle) {
                    //slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    theRobot.slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                   theRobot.motorpower = 0.3;
                    telemetry.addData("Touch Sensor", "starting motor");
                }
                else if (touchSensor.isPressed()) {
                    telemetry.addData("Touch Sensor", "Is Pressed");
                    theRobot.motorpower = 0.0;
                } else {
                    telemetry.addData("Touch Sensor", "Is Not Pressed");
                }
                theRobot.slidesMotor.setPower(theRobot.motorpower);
                telemetry.addData("Motor Ticks: ", theRobot.slidesMotor.getCurrentPosition());

                telemetry.update();
            } // end while (opModeIsActive())


        } catch (Exception e) {

            // throw the exception higher for other handlers to run
            throw e;
        }

    }



}
