/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Geraldine;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

/**
 *
 * This OpMode executes a basic Tank Drive Teleop for a two wheeled robot using two REV SPARKminis.
 * To use this example, connect two REV SPARKminis into servo ports on the Expansion Hub. On the
 * robot configuration, use the drop down list under 'Servos' to select 'REV SPARKmini Controller'
 * and name them 'left_drive' and 'right_drive'.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Disabled
@TeleOp(name="Geraldine_TeleOp", group="Geraldine")
public class Geraldine_TeleOp extends LinearOpMode {

    // Declare OpMode members.
    Geraldine Geraldine;
    double scale_arm = .5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // Servos below

        //Touch sensors below

        //Motors below

        //twist  = hardwareMap.get(DcMotorSimple.class, "twist");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backward when connected directly to the battery

        //twist.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetRuntime();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double max;
            double liftPower;
            //double twistPower;
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            // gamepad 1= gunner
            // gamepad 2= driver
            if (gamepad1.dpad_up)
            {
                liftPower = 1.0;
            }
            else if (gamepad1.dpad_down)
            {
                liftPower = -1.0;
            }
            else
            {
                liftPower = 0;
            }

            //lift.setPower(liftPower*scale);
            //twist.setPower(twistPower*scale);

            if (Geraldine.right_swivel.getState() == true)
            {
            }
            else
            {

            }

            if (Geraldine.left_swivel.getState() == true)
            {

            }
            else
            {

            }

            if ((Geraldine.low_arm.getState() == false) && (liftPower < 0))
            {

                Geraldine.lift.setPower(0);

            }
            else if ((Geraldine.high_arm.getState() == false) && (liftPower > 0))
            {
                Geraldine.lift.setPower(0);
            }
            else
            {
                Geraldine.lift.setPower(liftPower*scale_arm);


            }

            if (gamepad2.left_bumper)
            if (gamepad1.left_bumper == true)
            {
                Geraldine.left_mouth.setPower(-1);
                Geraldine.right_mouth.setPower(1);
            }

            else if (gamepad2.right_bumper);

            else if (gamepad1.right_bumper == true)
            {
                Geraldine.left_mouth.setPower(1);
                Geraldine.right_mouth.setPower(-1);
            }
            else
            {
                Geraldine.left_mouth.setPower(0);
                Geraldine.right_mouth.setPower(0);
            }

            telemetry.addData("Front left/Right: ", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right: ", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Touch Left Swivel: ", Geraldine.left_swivel.getState());
            telemetry.addData("Touch Right Swivel: ", Geraldine.right_swivel.getState());
            telemetry.addData("Touch High Arm: ", Geraldine.high_arm.getState());
            telemetry.addData("Touch Low Arm: ", Geraldine.low_arm.getState());
            //telemetry.addData("Servos", "left (%.2f), right (%.2f)", left_mouth.getPosition(), right_mouth.getPosition());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", liftPower); //Twist power is temporarily deleted
            telemetry.update();
        }
    }
}
