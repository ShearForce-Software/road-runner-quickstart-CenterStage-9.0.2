package org.firstinspires.ftc.teamcode.summerChassis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.summerChassis.SummerChassis;

@TeleOp(name = "1 Manual Control Summer Chassis")
public class ManualControlSummerChassis extends LinearOpMode {
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

                if(gamepad1.y){
                    theRobot.imu.resetYaw();
                }

                telemetry.update();
            } // end while (opModeIsActive())


        } catch (Exception e) {

            // throw the exception higher for other handlers to run
            throw e;
        }
    }
}
