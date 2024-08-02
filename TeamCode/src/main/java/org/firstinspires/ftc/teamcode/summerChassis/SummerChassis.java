package org.firstinspires.ftc.teamcode.summerChassis;

import static org.firstinspires.ftc.teamcode.summerChassis.MecanumDrive_summerChassis.PARAMS;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class SummerChassis {
    LinearOpMode opMode;
    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor rightRear;
  //  DcMotor slidesMotor;
  // slidesMotor was commented out to allow SummerChassis to be calibrated
    IMU imu;
    public double imuOffsetInDegrees;

    boolean IsDriverControl;
    boolean IsFieldCentric;
    int autoPosition;
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    public static boolean allianceColorIsBlue = false;
    public static double autoTimeLeft = 0.0;

    public double newTarget;
    public double motorpower = 0.0;

    public SummerChassis(boolean isDriverControl, boolean isFieldCentric, LinearOpMode opMode) {
        this.IsDriverControl = isDriverControl;
        this.IsFieldCentric = isFieldCentric;
        this.opMode = opMode;
    }
    public void Init (HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront_leftOdometry");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront_rightOdometry");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        //  slidesMotor = hardwareMap.get(DcMotor.class, "slidesMotor");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    public void moveRobot(double x, double y, double yaw) {
       // opMode.telemetry.addData("Claw Distance: ", clawDistanceSensor.getDistance(DistanceUnit.MM));
      //  opMode.telemetry.update();
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFront.setPower(leftFrontPower *.5);
        rightFront.setPower(rightFrontPower *.5);
        leftRear.setPower(leftBackPower *.5);
        rightRear.setPower(rightBackPower *.5);

      //  opMode.telemetry.addData("Claw Distance: ", clawDistanceSensor.getDistance(DistanceUnit.MM));
      //  opMode.telemetry.update();
    }
    public void EndgameBuzzer(){
        if(opMode.getRuntime() < 109.5 && opMode.getRuntime() > 109.0){
            opMode.gamepad1.rumble(1000);
            opMode.gamepad2.rumble(1000);
        }
    }

    public void driveControlsRobotCentric() {
        double y = -opMode.gamepad1.left_stick_y;
        double x = opMode.gamepad1.left_stick_x * 1.1;
        double rx = opMode.gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }
    public void driveControlsRobotCentricKID() {
        double y = -opMode.gamepad2.left_stick_y;
        double x = opMode.gamepad2.left_stick_x * 1.1;
        double rx = opMode.gamepad2.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower*.25);
        leftRear.setPower(backLeftPower*.25);
        rightFront.setPower(frontRightPower*.25);
        rightRear.setPower(backRightPower*.25);
    }

    public double GetIMU_HeadingInDegrees()
    {
        double botHeading = AngleUnit.normalizeDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + imuOffsetInDegrees);

        return botHeading;

    }
    public void driveControlsFieldCentric() {
        double y = -opMode.gamepad1.left_stick_y;
        double x = opMode.gamepad1.left_stick_x;
        double rx = opMode.gamepad1.right_stick_x;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower*1.0);
        leftRear.setPower(backLeftPower*1.0);
        rightFront.setPower(frontRightPower*1.0);
        rightRear.setPower(backRightPower*1.0);
    }
    public void RunDriveControls() {
        if (IsFieldCentric) {
            driveControlsFieldCentric();
        }
        else {
            driveControlsRobotCentric();
        }
    }
    public void SetFieldCentricMode(boolean fieldCentricEnabled) {
        IsFieldCentric = fieldCentricEnabled;
    }
    public void SpecialSleep(long milliseconds) {
        for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(milliseconds);
             stop > System.nanoTime(); ) {
            if (!opMode.opModeIsActive() || opMode.isStopRequested()) return;
            if (IsDriverControl) {
                if (IsFieldCentric) driveControlsFieldCentric();
                if (!IsFieldCentric) driveControlsRobotCentric();
            }
        }
    }
   /* public void slides( double x){
        slidesMotor.getCurrentPosition();
        newTarget = 103.6*x;
        slidesMotor.setTargetPosition((int) newTarget);
        slidesMotor.setPower(motorpower);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setslidePower (double power) {
        slidesMotor.setPower(power);
        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
*/
}