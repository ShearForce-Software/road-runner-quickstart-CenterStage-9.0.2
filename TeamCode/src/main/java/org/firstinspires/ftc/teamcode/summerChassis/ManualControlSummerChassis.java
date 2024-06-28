package org.firstinspires.ftc.teamcode.summerChassis;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.summerChassis.SummerChassis;

@TeleOp(name = "1 Manual Control Summer Chassis")
public class ManualControlSummerChassis extends LinearOpMode {
    DcMotor slidesMotor;
    double ticks = 103.6;
    double newTarget;

    public void Init(HardwareMap hardwareMap) {
        slidesMotor = hardwareMap.get(DcMotor.class, "slides");
        telemetry.addData("Hardware: ", "Initialized");
        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runOpMode() {


        SummerChassis theRobot;
        theRobot = new SummerChassis(true, true, this);

        theRobot.Init(this.hardwareMap);





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
                telemetry.addData("Motor Ticks: ", slidesMotor.getCurrentPosition());
                if (gamepad1.circle) {
                    slides(1);
                }

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
        slidesMotor.setPower(0.3);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
