package org.firstinspires.ftc.teamcode.Gertrude;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Gertrude.Gertrude_KidFriendly;

//@Disabled
@TeleOp(name = "Gertrude Kid Friendly - 1 Operator")
public class KidFriendly_Gertrude_1Operator extends LinearOpMode {
    Gertrude_KidFriendly theRobot;
    static final double SCALE = 0.001;
    public void runOpMode() {
        theRobot = new Gertrude_KidFriendly(true, true, this);

        double armRotationLeftPosition = 0.07;
        double armRotationRightPosition = 0.07;
        final double MAX_POS     =  1.0;     // Maximum rotational position
        final double MIN_POS     =  0.0;     // Minimum rotational position
        final double MIN_POS_ARM     =  0.04;     // Minimum rotational position

        theRobot.Init(this.hardwareMap);
        theRobot.ManualStartPos();
        theRobot.ShowSlideTelemetry();

        telemetry.update();
        waitForStart();
        resetRuntime();

        try {
            while (opModeIsActive()) {
                if (isStopRequested()){
                    theRobot.ServoStop();
                    theRobot.DisableAutoIntake();
                    sleep(100);
                }
                theRobot.EndgameBuzzer();
                theRobot.PickupRoutine();

                /* *************************************************
                   *************************************************
                   * Driver Controls (gamepad1)
                   *************************************************
                   *************************************************
                 */
                // Drive Controls uses left_stick_y, left_stick_x, and right_stick_x
                theRobot.driveControlsFieldCentric();
                if (gamepad1.a && gamepad1.back) {
                    theRobot.LaunchAirplane();
                }
                if (gamepad1.b && gamepad1.back){
                    // Auto drives to the board using the arm distance sensor
                    theRobot.StopNearBoard();
                }
                if(gamepad1.y && gamepad1.back){
                    theRobot.imu.resetYaw();
                }
                if (gamepad1.left_trigger != 0 && gamepad1.back ) {
                    theRobot.SetScissorLiftPower(gamepad1.left_trigger);
                } else if (gamepad1.right_trigger != 0 && gamepad1.back) {
                    theRobot.SetScissorLiftPower(-gamepad1.right_trigger);
                } else {
                    theRobot.SetScissorLiftPower(0);
                }

                /* *************************************************
                 *************************************************
                 * Gunner / Arm Controls (gamepad2)
                 *
                 *************************************************
                 *************************************************
                 */
                if (gamepad1.right_bumper) { // intake in
                    theRobot.EnableAutoIntake();
                }
                if (gamepad1.right_bumper && gamepad1.back ) {
                    theRobot.ServoStop();
                    theRobot.DisableAutoIntake();
                }
                if (gamepad1.left_bumper) { // release pixels
                    theRobot.ReleaseRight();
                    theRobot.ReleaseLeft();
                }
                if (gamepad1.left_trigger != 0){
                    theRobot.ReleaseRight();
                }
                if (gamepad1.right_trigger != 0){
                    theRobot.ReleaseLeft();
                }
                // Slides HIGH
                if (gamepad1.y && !gamepad1.back ) {
                    theRobot.ServoStop();
                    theRobot.SlidesHigh();
                    theRobot.SpecialSleep(500);
                    theRobot.DeliverPixelToBoardPos();
                }
                // Slides MEDIUM
                if (gamepad1.x) {
                    theRobot.ServoStop();
                    theRobot.SlidesMedium();
                    theRobot.SpecialSleep(500);
                    theRobot.DeliverPixelToBoardPos();
                }
                // Slides LOW
                if (gamepad1.a && !gamepad1.start) {
                    theRobot.ServoStop();
                    theRobot.SlidesToAuto();
                    theRobot.SpecialSleep(500);
                    theRobot.DeliverPixelToBoardPos();
                }
                if (gamepad1.left_stick_button){
                    theRobot.PickupOne();
                }
                // RESET Slides, ARM, and Wrist
                if (gamepad1.b && !gamepad1.start) {
                    theRobot.ResetArm();
                }

                // Manual control of the slides
                if (gamepad1.dpad_up)
                {
                    // slowly raise the slides up
                    theRobot.leftSlide.setTargetPosition(-1800);
                    theRobot.rightSlide.setTargetPosition(-1800);
                    theRobot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    theRobot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    theRobot.SetSlidePower(.25);
                }
                if (gamepad1.dpad_down)
                {
                    // slowly lower the slides
                    theRobot.SlidesDown();
                }

                theRobot.ShowSlideTelemetry();
                telemetry.update();
            } // end while (opModeIsActive())

            // Stop must have been pressed to get here, make sure the continuous rotation servos are stopped
            theRobot.ServoStop();
            theRobot.DisableAutoIntake();
            sleep(150);

        } catch (Exception e) {
            // something went wrong and has crashed
            // Make sure the continuous rotation servos are stopped
            theRobot.ServoStop();
            theRobot.DisableAutoIntake();
            sleep(150);

            // throw the exception higher for other handlers to run
            throw e;
        }
    }
}
