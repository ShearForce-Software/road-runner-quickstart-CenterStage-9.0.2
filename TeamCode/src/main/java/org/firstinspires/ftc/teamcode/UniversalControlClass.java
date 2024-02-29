package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.MecanumDrive.PARAMS;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.lang.reflect.Field;
import java.util.List;
import java.util.concurrent.TimeUnit;
@Config
public class  UniversalControlClass {
    LinearOpMode opMode;
    //TODO: Create Motors, sensors, and servos
    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor rightRear;
    DcMotor rightSlide;
    DcMotor leftSlide;
    DcMotor leftScissor;
    DcMotor rightScissor;
    TouchSensor leftSlideLimit;
    TouchSensor rightSlideLimit;
    CRServo intakeLeft;
    CRServo intakeRight;
    RevColorSensorV3 leftColorSensor;
    RevColorSensorV3 rightColorSensor;
    RevColorSensorV3 clawDistanceSensor;
    HuskyLens huskyLens;
    HuskyLens huskyLens2;
    IMU imu;
    Servo   grabberLeft;
    Servo   grabberRight;
    Servo armRotRight;
    Servo armRotLeft;
    Servo wristRight;
    Servo wristLeft;
    Servo droneLauncher;
    RevBlinkinLedDriver.BlinkinPattern Blinken_left_pattern;
    RevBlinkinLedDriver.BlinkinPattern Blinken_right_pattern;
    RevBlinkinLedDriver blinkinLedDriverLeft;
    RevBlinkinLedDriver blinkinLedDriverRight;

    //TODO: set universal variables (public static to make available in dashboard
    boolean IsDriverControl;
    boolean IsFieldCentric;
    int hopperDistance = 30;
    double  grabberPosition = 0; // Start at minimum rotational position
    int leftSpikeBound = 100;
    int rightSpikeBound = 200;
    int autoPosition;
    double pixelCorrectionAmountLR = 0.0;
    double pixelWidth_HL = 0.0;
    double hl_rangeToBoard = 0.0;
    double distanceCorrectionLR_HL = 0.0;
    double hl_halfScreenWidth = 0.0;
    double stackCorrectionLR = 0.0;
    double stackCorrection = 0.0;
    double stackWidth = 0.0;
    public static double grabPosition = 0.5;
    public static double dropPosition = 0;
    private TfodProcessor tfod;
    boolean AutoIntake = false;
    public static final double SLIDE_POWER   = 0.75;
    public static final int SLIDE_MAX_HEIGHT = -2850;
    public static final int SLIDE_MIN_HEIGHT = 0;
    public static final int SLIDE_AUTO_HEIGHT = -450;
    public static final int SLIDE_LOW_HEIGHT = -400;
    public static final int SLIDE_MEDIUM_HEIGHT = -1500;
    public double wristPosition = 0.0;
    public double wholeArmPosition = 0.04;
    static final double MAX_WRIST_POS = 1.0;
    static final double MIN_WRIST_POS = 0.0;
    public double WRIST_GRAB_PIXEL_POS = 0.45;
    public double WRIST_DELIVER_TO_BOARD_POS = 0.85;
    static final double MAX_WHOLE_ARM_POS = 1.0;
    static final double MIN_WHOLE_ARM_POS = 0.04;
    //NAV TO TAG VARIABLES
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    public static int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    double rangeError = 0;
    double yawError = 0;
    private double slidePower = 0.0;
    int redLeft = 0;
    int greenLeft = 0;
    int blueLeft = 0;
    int redRight = 0;
    int greenRight = 0;
    int blueRight = 0;
    public static boolean allianceColorIsBlue = false;

    //TODO: Add any other specification variables
    public UniversalControlClass(boolean isDriverControl, boolean isFieldCentric, LinearOpMode opMode) {
        this.IsDriverControl = isDriverControl;
        this.IsFieldCentric = isFieldCentric;
        this.opMode = opMode;
    }
    public void Init (HardwareMap hardwareMap) {
        //TODO: hardware map all servos, motors, sensors, and cameras
        leftFront = hardwareMap.get(DcMotor.class, "leftFront_leftOdometry");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront_rightOdometry");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        grabberLeft = hardwareMap.get(Servo.class, "pixelGrabberLeft");
        grabberRight = hardwareMap.get(Servo.class, "pixelGrabberRight");
        armRotLeft = hardwareMap.get(Servo.class, "armRotateLeft");
        armRotRight = hardwareMap.get(Servo.class, "armRotateRight");
        wristLeft = hardwareMap.get(Servo.class, "pixelRotateLeft");
        wristRight = hardwareMap.get(Servo.class, "pixelRotateRight");
        droneLauncher = hardwareMap.get(Servo.class, "droneLaunch");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        leftSlideLimit = hardwareMap.get(TouchSensor.class, "leftSlideLimit");
        rightSlideLimit = hardwareMap.get(TouchSensor.class, "rightSlideLimit");

        leftScissor = hardwareMap.get(DcMotor.class, "centerOdometry");
        rightScissor = hardwareMap.get(DcMotor.class, "rightScissor");
        leftScissor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftScissor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightScissor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftScissor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightScissor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensorLeft");
        rightColorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensorRight");
        clawDistanceSensor = hardwareMap.get(RevColorSensorV3.class, "ClawSensor");
        InitBlinkin(hardwareMap);
        leftColorSensor.enableLed(false);
        rightColorSensor.enableLed(false);
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens1");
        huskyLens2 = hardwareMap.get(HuskyLens.class, "huskyLens2");

        //TODO: set motor direction, zero power brake behavior, stop and reset encoders, etc
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        armRotLeft.setDirection(Servo.Direction.FORWARD);  // Updated for Axon: To FORWARD
        armRotRight.setDirection(Servo.Direction.REVERSE); // Updated for Axon: To REVERSE
        wristLeft.setDirection(Servo.Direction.REVERSE);
        wristRight.setDirection(Servo.Direction.FORWARD);
        grabberLeft.setDirection(Servo.Direction.REVERSE);
        grabberRight.setDirection(Servo.Direction.FORWARD);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));
        imu.initialize(parameters);
        imu.resetYaw();
        droneLauncher.setPosition(.1);
    }

    public void WebcamInit (HardwareMap hardwareMap){
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        /*aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();*/
        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.

            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
    }
    public void NavToTag(){
        desiredTag  = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) && (detection.id == DESIRED_TAG_ID) ){
                desiredTag = detection;
                rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                yawError        = desiredTag.ftcPose.yaw;
                break;  // don't look any further.
            } else {
                opMode.telemetry.addData("Unknown Target - ", "No Tag Detected");
                // set some range and yaw error
            }
        }
    }
    public void DriveToTag() {
        double drive = 0.0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0.0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0.0;        // Desired turning power/speed (-1 to +1)

        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        NavToTag();
        double headingError = desiredTag.ftcPose.bearing;

        double timeout = opMode.getRuntime() + 5;

        while ((rangeError > DESIRED_DISTANCE) &&
                ((headingError > 2.0) || (headingError < -2.0)) &&
                ((yawError > 2.0) || (yawError < -2.0)) && (opMode.getRuntime() < timeout)) {
            // Determine heading, range and Yaw (tag image rotation) errors
            NavToTag();
            headingError = desiredTag.ftcPose.bearing;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            opMode.telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            opMode.telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            opMode.telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            opMode.telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            opMode.telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            opMode.telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            opMode.sleep(10);
        }
    }
    public void AutoStartPos(){
        grabberLeft.setPosition(0);
        grabberRight.setPosition(0);
        armRotLeft.setPosition(.16);
        armRotRight.setPosition(.16);
        wristLeft.setPosition(.95);
        wristRight.setPosition(.95);
    }
    public void GrabPixels(){
        grabberRight.setPosition(.72);
        grabberLeft.setPosition(.72);
    }
    public void DropOnLine(){
        // move the arm to just off of the floor -- candidate part to do in parallel
        armRotLeft.setPosition(.72);
        armRotRight.setPosition(.72);
        SpecialSleep(200);

        // move the wrist to put the purple pixel on the floor
        wristLeft.setPosition(.85);
        wristRight.setPosition(.85);
        SpecialSleep(200);

        // release the purple pixel
        grabberRight.setPosition(0);
        SpecialSleep(200);

        // Move the arm and wrist slightly up so the grabber servo is clear of the pixel, so doesn't fly out when the arm is reset
        armRotLeft.setPosition(.7);
        armRotRight.setPosition(.7);
        wristLeft.setPosition(.83);
        wristRight.setPosition(.83);
        SpecialSleep(200);
    }
    public void SafeStow(){
        grabberLeft.setPosition(.72);
        grabberRight.setPosition(.72);
        armRotLeft.setPosition(.16);
        armRotRight.setPosition(.16);
        wristLeft.setPosition(.95);
        wristRight.setPosition(.95);
    }
    public void ManualStartPos(){
        armRotLeft.setPosition(.07);
        armRotRight.setPosition(.07);
        wristLeft.setPosition(WRIST_GRAB_PIXEL_POS);
        wristRight.setPosition(WRIST_GRAB_PIXEL_POS);
        grabberLeft.setPosition(0);
        grabberRight.setPosition(0);
    }
    public void GrabPixelPos(){ // in center of pixels
        wristLeft.setPosition(WRIST_GRAB_PIXEL_POS);
        wristRight.setPosition(WRIST_GRAB_PIXEL_POS);
        armRotLeft.setPosition(.04);
        armRotRight.setPosition(.04);
    }
    public void ReadyToLiftSlides(){ // slight move before lifting slides
        armRotLeft.setPosition(.08);
        armRotRight.setPosition(.08);
        //SpecialSleep(150);
        wristLeft.setPosition(WRIST_GRAB_PIXEL_POS-.02);
        wristRight.setPosition(WRIST_GRAB_PIXEL_POS-.02);
        //SpecialSleep(150);
    }

    public void SlidesToAuto(){
        leftSlide.setTargetPosition(SLIDE_AUTO_HEIGHT);
        rightSlide.setTargetPosition(SLIDE_AUTO_HEIGHT);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SetSlidePower(SLIDE_POWER);
    }
    public void DeliverPixelToBoardPos(){
        armRotLeft.setPosition(.57);
        armRotRight.setPosition(.57);
        SpecialSleep(150);
        wristLeft.setPosition(WRIST_DELIVER_TO_BOARD_POS);
        wristRight.setPosition(WRIST_DELIVER_TO_BOARD_POS);
    }
    public void DeliverPixelToBoardPosPart1(){
        armRotLeft.setPosition(.57);
        armRotRight.setPosition(.57);
    }
    public void DeliverPixelToBoardPosPart2(){
        wristLeft.setPosition(WRIST_DELIVER_TO_BOARD_POS);
        wristRight.setPosition(WRIST_DELIVER_TO_BOARD_POS);
    }
    public void ResetArmAuto(){
        armRotLeft.setPosition(.07);
        armRotRight.setPosition(.07);
        wristLeft.setPosition(WRIST_GRAB_PIXEL_POS);
        wristRight.setPosition(WRIST_GRAB_PIXEL_POS);
        //SlidesDownInParallel();
    }
    public void ResetArmAutoNoSlides(){
        armRotLeft.setPosition(.07);
        armRotRight.setPosition(.07);
        wristLeft.setPosition(WRIST_GRAB_PIXEL_POS);
        wristRight.setPosition(WRIST_GRAB_PIXEL_POS);
    }
    public void ResetArm(){
        ManualStartPos();
        SpecialSleep(250);
        SlidesDown();
    }
    public void PickupRoutine(){

        if (AutoIntake){
            ServoIntake();
            if((leftColorSensor.getDistance(DistanceUnit.MM) < hopperDistance) && (rightColorSensor.getDistance(DistanceUnit.MM) < hopperDistance)){
                opMode.gamepad1.rumble(1000);
                ServoStop();
                GrabPixelPos();
                SpecialSleep(500);
                GrabPixels();
                SpecialSleep(750);
                SetBlinkinToPixelColor();
                ServoOuttake();
                ReadyToLiftSlides();
                AutoIntake = false;
            }
        }
    }
    public void AutoPickupRoutineDrive(double wait){
        double timeout = opMode.getRuntime() + wait;
        boolean forward = true;
        ServoIntake();
        while(opMode.getRuntime() <= timeout){
            if((leftColorSensor.getDistance(DistanceUnit.MM) < hopperDistance) && (rightColorSensor.getDistance(DistanceUnit.MM) < hopperDistance)) {
                break;
            }
            else if((leftColorSensor.getDistance(DistanceUnit.MM) < hopperDistance) || (rightColorSensor.getDistance(DistanceUnit.MM) < hopperDistance)) {
                if (forward) {
                    moveRobot(-.1, 0, 0);
                    forward = false;
                }
                else{
                    moveRobot(.75, 0, 0);
                    forward = true;
                }
            }
            else{
                moveRobot(.65, 0, 0);
            }
            opMode.sleep(100);
        }
    }
    public void AutoPickupRoutineStopAndLower(){
        // stop the spinners
        ServoStop();
        // Move arm and wrist down to grab the pixels
        GrabPixelPos();
        GrabPixels();
    }
    public void AutoPickupRoutineGrabAndUp(){
        // Turn spinners back on the other way, in case a pixel is in a bad spot
        // make a slight move of the arm to prevent the pixels getting hit on the sensors when the slides start going up has a (150ms sleep)
        //ReadyToLiftSlides();
        armRotLeft.setPosition(.08);
        armRotRight.setPosition(.08);

        //ServoOuttake();
    }
    public void AutoPickupRoutine(){
        double timeout = opMode.getRuntime() + 1.5;
        EnableAutoIntake();
        ServoIntake();
        while((AutoIntake) && (opMode.getRuntime() <= timeout)){
            moveRobot(.5, 0, 0);
            if((leftColorSensor.getDistance(DistanceUnit.MM) < hopperDistance) && (rightColorSensor.getDistance(DistanceUnit.MM) < hopperDistance)) {
                break;
            }
            opMode.sleep(100);
        }
        // stop the spinners
        ServoStop();

        // Move arm and wrist down to grab the pixels
        GrabPixelPos();
        SpecialSleep(400);

        // lock on to the pixels
        GrabPixels();
        SpecialSleep(500);

        // Turn spinners back on the other way, in case a pixel is in a bad spot
        ServoOuttake();

        // make a slight move of the arm to prevent the pixels getting hit on the sensors when the slides start going up has a (150ms sleep)
        ReadyToLiftSlides();
        AutoIntake = false;
    }
    public void EnableAutoIntake(){
        AutoIntake = true;
    }
    public void DisableAutoIntake(){
        AutoIntake = false;
    }
    public void InitBlinkin(HardwareMap hardwareMap) {
        blinkinLedDriverLeft = hardwareMap.get(RevBlinkinLedDriver.class,"leftBlinkin");
        blinkinLedDriverRight = hardwareMap.get(RevBlinkinLedDriver.class,"rightBlinkin");
        //leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensorLeft");

        if(allianceColorIsBlue){
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
        }else {
            Blinken_left_pattern  = RevBlinkinLedDriver.BlinkinPattern.RED;
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
        }
    }
    public void ArmWrist(double position){
        if (position > MAX_WRIST_POS){
            wristPosition = MAX_WRIST_POS;
        }else if(position < MIN_WRIST_POS){
            wristPosition = MIN_WRIST_POS;
        }else{
            wristPosition = position;
        }
        wristLeft.setPosition(wristPosition);
        wristRight.setPosition(wristPosition);
    }

    public void ResetWristGrabPixelPos() {
        // SPECIAL routine to change the wrist position that equals the right spot
        // to grab the pixels, if the servo gets shifted out of position
        WRIST_GRAB_PIXEL_POS = wristLeft.getPosition();
        WRIST_DELIVER_TO_BOARD_POS = WRIST_GRAB_PIXEL_POS + 0.4;
    }
    public double getWristPosition(){
        return wristLeft.getPosition();
    }
    public void WholeArmRot(double position){
    if (position > MAX_WHOLE_ARM_POS){
            wholeArmPosition = MAX_WHOLE_ARM_POS;
        }else if(position < MIN_WHOLE_ARM_POS){
            wholeArmPosition = MIN_WHOLE_ARM_POS;
        }else{
            wholeArmPosition = position;
        }
        armRotLeft.setPosition(wholeArmPosition);
        armRotRight.setPosition(wholeArmPosition);
    }
    public double getWholeArmPosition(){
        return wholeArmPosition;
    }
    public void ReleaseLeft(){
        grabberLeft.setPosition(0);
        if(allianceColorIsBlue){
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
        }else {
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
        }
    }
    public void ReleaseRight(){
        grabberRight.setPosition(0);
        if(allianceColorIsBlue){
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }else {
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }
    }
    public void ServoIntake() {
        intakeRight.setPower(-1.0);
        intakeLeft.setPower(-1.0);
    }
    public void ServoOuttake() {
        intakeRight.setPower(1);
        intakeLeft.setPower(1);
    }
    public void ServoStop(){
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }
    public void SlidesLow(){
        leftSlide.setTargetPosition(SLIDE_LOW_HEIGHT);
        rightSlide.setTargetPosition(SLIDE_LOW_HEIGHT);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SetSlidePower(SLIDE_POWER);
    }
    public void SlidesMedium(){
        leftSlide.setTargetPosition(SLIDE_MEDIUM_HEIGHT);
        rightSlide.setTargetPosition(SLIDE_MEDIUM_HEIGHT);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SetSlidePower(SLIDE_POWER);
    }
    public void SlidesHigh(){
        leftSlide.setTargetPosition(SLIDE_MAX_HEIGHT);
        rightSlide.setTargetPosition(SLIDE_MAX_HEIGHT);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SetSlidePower(SLIDE_POWER);
    }
    public void SlidesDown() {
        double timeout = opMode.getRuntime() + 3;
        // while one of the slide limit switches are not pressed
        while ((!(leftSlideLimit.isPressed() || rightSlideLimit.isPressed()))&&(opMode.getRuntime() < timeout))
        {
            leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (leftSlideLimit.isPressed()){
                leftSlide.setPower(0);
                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }else{
                leftSlide.setPower(slidePower);
            }

            if (rightSlideLimit.isPressed()){
                rightSlide.setPower(0);
                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }else{
                rightSlide.setPower(slidePower);
            }
            if (opMode.gamepad1.left_trigger != 0) {
                SetScissorLiftPower(opMode.gamepad1.left_trigger);
            } else if (opMode.gamepad1.right_trigger != 0) {
                SetScissorLiftPower(-opMode.gamepad1.right_trigger);
            } else {
                SetScissorLiftPower(0);
            }
            SpecialSleep(150);
        }

        // at least one of the slide limits has been pressed when we get to here, stop both slide motors, and then reset encoders to zero
        leftSlide.setPower(0);
        rightSlide.setPower(0);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public boolean SlidesDownInParallel() {
        boolean slidesAllDown = false;
        // if one of the slide limit switches are not pressed
        if ((!(leftSlideLimit.isPressed() || rightSlideLimit.isPressed())))
        {
            leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (leftSlideLimit.isPressed()){
                leftSlide.setPower(0);
                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }else{
                leftSlide.setPower(slidePower);
            }

            if (rightSlideLimit.isPressed()){
                rightSlide.setPower(0);
                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }else{
                rightSlide.setPower(slidePower);
            }
        }
        else {
            // at least one of the slide limits has been pressed when we get to here, stop both slide motors, and then reset encoders to zero
            leftSlide.setPower(0);
            rightSlide.setPower(0);
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slidesAllDown = true;
        }
        return slidesAllDown;
    }
    public void SetSlidePower(double power){
        //TODO: CLAIRE slides w/ limit switch
        if ((leftSlideLimit.isPressed() && power < 0) || (rightSlideLimit.isPressed() && power < 0))
        {
            slidePower = 0;
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else
        {
            slidePower = power;
        }
        leftSlide.setPower(slidePower);
        rightSlide.setPower(slidePower);
    }
    public void ShowSlideTelemetry(){
        opMode.telemetry.addData("Left Slide: ", leftSlide.getCurrentPosition());
        opMode.telemetry.addData("Right Slide: ", rightSlide.getCurrentPosition());
        opMode.telemetry.addData("LeftLimitPushed: ", leftSlideLimit.isPressed());
        opMode.telemetry.addData("RightLimitPushed: ", rightSlideLimit.isPressed());
        opMode.telemetry.addData("Arm Servo Left: ", armRotLeft.getPosition());
        opMode.telemetry.addData("Arm Servo Right: ", armRotRight.getPosition());
        opMode.telemetry.addData("Wrist Position: ", wristLeft.getPosition());
        opMode.telemetry.addData("Left Hopper: ", leftColorSensor.getDistance(DistanceUnit.MM));
        opMode.telemetry.addData("Right Hopper: ", rightColorSensor.getDistance(DistanceUnit.MM));
        opMode.telemetry.addData("LeftScissor: ", leftScissor.getPower());
        opMode.telemetry.addData("RightScissor: ", rightScissor.getPower());
        opMode.telemetry.addData("Claw Distance: ", clawDistanceSensor.getDistance(DistanceUnit.MM));
        showColorSensorTelemetry();
        opMode.telemetry.update();
    }

    public void StopNearBoard(){
        double timeout = opMode.getRuntime() + 3;
        while((clawDistanceSensor.getDistance(DistanceUnit.MM) > 78) && (opMode.getRuntime() < timeout)){
            opMode.telemetry.addData("Claw Distance: ", clawDistanceSensor.getDistance(DistanceUnit.MM));
            opMode.telemetry.update();
            moveRobot(-.55,0.0,0);
            opMode.sleep(100);
            if(clawDistanceSensor.getDistance(DistanceUnit.MM) <= 78){
                moveRobot(0.0,0.0,0);
                opMode.sleep(100);
                ReleaseLeft();
                ReleaseRight();
                break;
            }
        }
    }
    public void StopNearBoardAuto(boolean releaseBoth){
        double timeout = opMode.getRuntime() + 3;
        while((clawDistanceSensor.getDistance(DistanceUnit.MM) > 50) && (opMode.getRuntime() < timeout)){
            moveRobot(-.55,0.0,0);
            ShowSlideTelemetry();
            opMode.sleep(100);
            if(clawDistanceSensor.getDistance(DistanceUnit.MM) <= 50){
                moveRobot(0.0,0.0,0);
                opMode.sleep(100);
                ReleaseLeft();
                if (releaseBoth) ReleaseRight();
                break;
            }
            ReleaseLeft();
            if (releaseBoth) ReleaseRight();
        }
    }
    public void moveRobot(double x, double y, double yaw) {
        opMode.telemetry.addData("Claw Distance: ", clawDistanceSensor.getDistance(DistanceUnit.MM));
        opMode.telemetry.update();
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

        opMode.telemetry.addData("Claw Distance: ", clawDistanceSensor.getDistance(DistanceUnit.MM));
        opMode.telemetry.update();
    }
    public void SetScissorLiftPower(double power){
        leftScissor.setPower(power);
        rightScissor.setPower(power);
    }

    public void EndgameBuzzer(){
        if(opMode.getRuntime() < 109.5 && opMode.getRuntime() > 109.0){
            opMode.gamepad1.rumble(1000);
            opMode.gamepad2.rumble(1000);
        }
    }
    public void ColorDetect(){
        //double rightColor = rightColorSensor.getLightDetected();
    }
    public void showColorSensorTelemetry(){
        //int leftColor = leftColorSensor.getNormalizedColors().toColor();
        //opMode.telemetry.addData("leftColorNorm: ", leftColor);
        opMode.telemetry.addData("leftColor(red): ", redLeft);
        opMode.telemetry.addData("leftColor(green): ", greenLeft);
        opMode.telemetry.addData("leftColor(blue): ", blueLeft);
        opMode.telemetry.addData("rightColor(red): ", redRight);
        opMode.telemetry.addData("rightColor(green): ", greenRight);
        opMode.telemetry.addData("rightColor(blue): ", blueRight);
        //opMode.telemetry.addData("rightColor: ", rightColor);
        //opMode.telemetry.addData("leftColorNorm(red): ", leftColorSensor.getNormalizedColors().red);
        //opMode.telemetry.addData("leftColorNorm(green): ", leftColorSensor.getNormalizedColors().green);
        //opMode.telemetry.addData("leftColorNorm(blue): ", leftColorSensor.getNormalizedColors().blue);
        /*
        int red = leftColorSensor.red();
        int green = leftColorSensor.green();
        int blue = leftColorSensor.blue();
        // Check for White Pixel
        if(red < 4000 && red > 1000 && green < 6000 && green > 3000 && blue < 7000 && blue > 3000) {
            opMode.telemetry.addData("Left: ", "is white");
        }
        // Check for yellow pixel
        else if(red < 2500 && red > 1000 && green < 3500 && green > 1500 && blue < 1000 && blue >0 )
        {
            opMode.telemetry.addData("Left: ", "is yellow");
        }
        // Check for green pixel
        else if(red < 1000 && red > 0 && green < 6000 && green > 1500 && blue < 1000 && blue >0 )
        {
            opMode.telemetry.addData("Left: ", "is green");
        }
        // Check for purple pixel
        else if(red < 3500 && red > 1000 && green < 4000 && green > 2000 && blue < 7000 && blue > 3500 )
        {
            opMode.telemetry.addData("Left: ", "is purple");
        }
        else {
            opMode.telemetry.addData("Left: ", "unknown");
        }

         */
    }
    public void SetBlinkinToPixelColor() {
        redLeft = leftColorSensor.red();
        greenLeft = leftColorSensor.green();
        blueLeft = leftColorSensor.blue();
        redRight = rightColorSensor.red();
        greenRight = rightColorSensor.green();
        blueRight = rightColorSensor.blue();

        // Left sensor left blinkin
        if(redLeft > (blueLeft / 2) && greenLeft > redLeft && blueLeft > redLeft) {
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
        }
        else if(greenLeft > redLeft && redLeft > blueLeft) {
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
        }
        else if(greenLeft > (redLeft * 2) && greenLeft > (blueLeft * 2)) {
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
        }
        else if(blueLeft > greenLeft && greenLeft > redLeft) {
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
        }
        else if(allianceColorIsBlue){
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
        }else {
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
        }

        //Right sensor right blinkin
        if(redRight > (blueRight / 2) && greenRight > redRight && blueRight > redRight) {
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }
        else if(greenRight > redRight && redRight > blueRight) {
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }
        else if(greenRight > (redRight * 2) && greenRight > (blueRight * 2)) {
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }
        else if(blueRight > greenRight && greenRight > redRight) {
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }
        else if(allianceColorIsBlue){
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }else {
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }
    }
    public void ShowBlinkinTelemetry() {
        opMode.telemetry.addData("Blinkin Left: ", Blinken_left_pattern.toString());
        opMode.telemetry.addData("Blinkin Right: ", Blinken_right_pattern.toString());
    }
    public void HuskyLensInit(){
        if (!huskyLens.knock()) {
            opMode.telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            opMode.telemetry.addData(">>", "Press start to continue");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION
        );
        opMode.telemetry.update();
    }
    public void HuskyLensInit2(){
        if (!huskyLens2.knock()) {
            opMode.telemetry.addData(">>", "Problem communicating with " + huskyLens2.getDeviceName());
        } else {
            opMode.telemetry.addData(">>", "Press start to continue");
        }
        huskyLens2.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION
        );
        opMode.telemetry.update();
    }
    public void StackCorrection(){
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        double timeout = opMode.getRuntime() + 1;
        while ((currentRecognitions.size() < 1) && (opMode.getRuntime() < timeout))
        {
            opMode.sleep(50);
            currentRecognitions = tfod.getRecognitions();
        }

        if (currentRecognitions.size() < 1)
        {
            opMode.telemetry.addData("WARNING **** - No WHITE Pixels in view - ***** ", currentRecognitions.size());
            stackCorrection = -1;
        }
        else {
            opMode.telemetry.addData("# Objects Detected", currentRecognitions.size());
        }

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            stackWidth = 3/recognition.getWidth(); // pixels are 3 inches in diameter
            // I think the webcam resolution is 640x480
            stackCorrectionLR = x - 320; // x distance from center of the screen
            stackCorrection = stackCorrectionLR * stackWidth;

            //opMode.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            opMode.telemetry.addData("stack image Size:", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            opMode.telemetry.addData("stack Position:  ", "%.0f / %.0f", x, y);
            opMode.telemetry.addData("stackCorrection: ", stackCorrection);
            break;
        }
        opMode.telemetry.update();

    }
    public void TagCorrection(){
        HuskyLens.Block[] blocks = huskyLens2.blocks();
        double timeout = opMode.getRuntime() + 0.15;

        while ((blocks.length < 1) && (opMode.getRuntime() < timeout)) {
            opMode.sleep(50);
            blocks = huskyLens2.blocks();
        }
        if (blocks.length > 0){
            double xVal = blocks[0].x;
            pixelCorrectionAmountLR = xVal - 160;

            double xWidth = blocks[0].width;
            pixelWidth_HL = 2/xWidth;
            distanceCorrectionLR_HL = pixelCorrectionAmountLR * pixelWidth_HL;

            opMode.telemetry.addData("AprilTag width: ", xWidth);
            opMode.telemetry.addData("pixel width HL: ", "%.04f", pixelWidth_HL);

            hl_halfScreenWidth = pixelWidth_HL * 160;
            hl_rangeToBoard = (pixelWidth_HL * 160) / Math.tan(Math.toRadians(30));

            opMode.telemetry.addData("HL range to board", "%.01f in", hl_rangeToBoard);
            opMode.telemetry.addData("HL Half Screen Width", "%.01f in", hl_halfScreenWidth);
            opMode.telemetry.addData("Correction LR: ","%.01f in", distanceCorrectionLR_HL);
            opMode.telemetry.update();

        }
        else{
            opMode.telemetry.addData("WARNING **** - No April Tags in view - *****",0 );
            opMode.telemetry.update();
            distanceCorrectionLR_HL = 0.0;
        }
    }
    public void StackCorrectionHL(){
        HuskyLens.Block[] blocks = huskyLens.blocks();
        double timeout = opMode.getRuntime() + 0.15;

        while ((blocks.length < 1) && (opMode.getRuntime() < timeout)) {
            opMode.sleep(50);
            blocks = huskyLens.blocks();
        }
        if (blocks.length > 0){
            double xVal = blocks[0].x;
            pixelCorrectionAmountLR = xVal - 160;

            double xWidth = blocks[0].width;
            pixelWidth_HL = 3/xWidth;
            distanceCorrectionLR_HL = pixelCorrectionAmountLR * pixelWidth_HL;

            opMode.telemetry.addData("Stack width: ", xWidth);
            opMode.telemetry.addData("pixel width HL: ", "%.04f", pixelWidth_HL);

            hl_halfScreenWidth = pixelWidth_HL * 160;
            hl_rangeToBoard = (pixelWidth_HL * 160) / Math.tan(Math.toRadians(30));

            opMode.telemetry.addData("HL range to board", "%.01f in", hl_rangeToBoard);
            opMode.telemetry.addData("HL Half Screen Width", "%.01f in", hl_halfScreenWidth);
            opMode.telemetry.addData("Correction LR: ","%.01f in", distanceCorrectionLR_HL);
            opMode.telemetry.update();

        }
        else{
            opMode.telemetry.addData("WARNING **** - No Stack in view - *****",0 );
            opMode.telemetry.update();
            distanceCorrectionLR_HL = 0.0;
        }

    }
    public void DetectTeamArtBlue() {
        allianceColorIsBlue = true;
        if(allianceColorIsBlue){
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }else {
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if (blocks.length > 0){
            int xVal = blocks[0].x;
            opMode.telemetry.addData("Team Art Detected: ", true);
            opMode.telemetry.addData("Team Art X position: ", xVal);
            if (xVal <= leftSpikeBound){
                autoPosition = 3;
                DESIRED_TAG_ID = 3;
            }
            else if ((xVal > leftSpikeBound) && (xVal < rightSpikeBound)){
                autoPosition = 2;
                DESIRED_TAG_ID = 2;
            }
            else if (xVal >= rightSpikeBound)
            {
                autoPosition = 1;
                DESIRED_TAG_ID = 1;
            }
            opMode.telemetry.addData("Auto position: ", autoPosition);
        }
        else{
            //pick a spot
            opMode.telemetry.addData("!!Team Art NOT DETECTED!! ", "DEFAULT TO POSITION 1");
            autoPosition = 1;
            DESIRED_TAG_ID = 1;
        }
    }
    public void DetectTeamArtBlueBoard() {
        allianceColorIsBlue = true;
        if(allianceColorIsBlue){
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }else {
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if (blocks.length > 0){
            int xVal = blocks[0].x;
            opMode.telemetry.addData("Team Art Detected: ", true);
            opMode.telemetry.addData("Team Art X position: ", xVal);
            if (xVal <= leftSpikeBound){
                autoPosition = 3;
                DESIRED_TAG_ID = 3;
            }
            else if ((xVal > leftSpikeBound) && (xVal < rightSpikeBound)){
                autoPosition = 2;
                DESIRED_TAG_ID = 2;
            }
            else if (xVal >= rightSpikeBound)
            {
                autoPosition = 1;
                DESIRED_TAG_ID = 1;
            }
            opMode.telemetry.addData("Auto position: ", autoPosition);
        }
        else{
            //pick a spot
            opMode.telemetry.addData("!!Team Art NOT DETECTED!! ", "DEFAULT TO POSITION 1");
            autoPosition = 3;
            DESIRED_TAG_ID = 3;
        }
    }
    public void DetectTeamArtRed() {
        allianceColorIsBlue = false;
        if(allianceColorIsBlue){
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }else {
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if (blocks.length > 0){
            int xVal = blocks[0].x;
            opMode.telemetry.addData("Team Art Detected: ", true);
            opMode.telemetry.addData("Team Art X position: ", xVal);
            if (xVal <= leftSpikeBound){
                autoPosition = 3;
                DESIRED_TAG_ID = 6;
            }
            else if ((xVal > leftSpikeBound) && (xVal < rightSpikeBound)){
                autoPosition = 2;
                DESIRED_TAG_ID = 5;
            }
            else if (xVal >= rightSpikeBound)
            {
                autoPosition = 1;
                DESIRED_TAG_ID = 4;
            }
            opMode.telemetry.addData("Auto position: ", autoPosition);
        }
        else{
            //pick a spot
            opMode.telemetry.addData("!!Team Art NOT DETECTED!! ", "DEFAULT TO RIGHT");
            autoPosition = 3;
            DESIRED_TAG_ID = 6;
        }
    }
    public void DetectTeamArtRedBoard() {
        allianceColorIsBlue = false;
        if(allianceColorIsBlue){
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }else {
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if (blocks.length > 0){
            int xVal = blocks[0].x;
            opMode.telemetry.addData("Team Art Detected: ", true);
            opMode.telemetry.addData("Team Art X position: ", xVal);
            if (xVal <= leftSpikeBound){
                autoPosition = 3;
                DESIRED_TAG_ID = 6;
            }
            else if ((xVal > leftSpikeBound) && (xVal < rightSpikeBound)){
                autoPosition = 2;
                DESIRED_TAG_ID = 5;
            }
            else if (xVal >= rightSpikeBound)
            {
                autoPosition = 1;
                DESIRED_TAG_ID = 4;
            }
            opMode.telemetry.addData("Auto position: ", autoPosition);
        }
        else{
            //pick a spot
            opMode.telemetry.addData("!!Team Art NOT DETECTED!! ", "DEFAULT TO RIGHT");
            autoPosition = 1;
            DESIRED_TAG_ID = 4;
        }
    }
    public void LaunchAirplane() {
        if (droneLauncher.getPosition() < 0.5) {
            droneLauncher.setPosition(1);
            SpecialSleep(150);
        } else {
            droneLauncher.setPosition(0.1);
            SpecialSleep(150);
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
        for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(milliseconds); stop > System.nanoTime(); ) {
            if (!opMode.opModeIsActive() || opMode.isStopRequested()) return;
            if (IsDriverControl) {
                if (IsFieldCentric) driveControlsFieldCentric();
                if (!IsFieldCentric) driveControlsRobotCentric();
            }
        }
    }
}