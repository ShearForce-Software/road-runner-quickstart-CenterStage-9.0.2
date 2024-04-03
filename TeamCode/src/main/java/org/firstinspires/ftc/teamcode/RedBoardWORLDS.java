package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Disabled
@Autonomous(name="Red Board WORLDS", preselectTeleOp = "1 Manual Control")
public class RedBoardWORLDS extends LinearOpMode {
    UniversalControlClass control = new UniversalControlClass(true, false,this);
    MecanumDrive drive;
    Pose2d startPose;
    Pose2d deliverToFloorPose;
    Pose2d deliverToBoardPose;
    Pose2d stackPose;
    Action FloorTraj;
    Action DriveToStack;
    Action BoardTraj2;
    Action Park;
    Action DriveBackToStack;
    VelConstraint speedUpVelocityConstraint;
    AccelConstraint speedUpAccelerationConstraint;
    VelConstraint slowDownVelocityConstraint;
    AccelConstraint slowDownAccelerationConstraint;
    double stackY = -36.0;
    double stackX = -57.0;

    public void runOpMode(){
        startPose = new Pose2d(12,-62.5,Math.toRadians(90));
        stackPose = new Pose2d(stackX, stackY, Math.toRadians(180)); //-54.5,-11.5

        // Define some custom constraints to use when wanting to go faster than defaults
        speedUpVelocityConstraint = new TranslationalVelConstraint(75.0);
        speedUpAccelerationConstraint = new ProfileAccelConstraint(-40.0, 60.0);
        slowDownVelocityConstraint = new TranslationalVelConstraint(5); 
        slowDownAccelerationConstraint = new ProfileAccelConstraint(-20, 50);

        /* Initialize the Robot */
        drive = new MecanumDrive(hardwareMap, startPose);
        control.Init(hardwareMap);
        control.HuskyLensInit();
        //control.WebcamInit(hardwareMap);
        control.AutoStartPos();
        telemetry.update();
        control.imuOffsetInDegrees = 90; // Math.toDegrees(startPose.heading.toDouble());

        while(!isStarted()){
            control.DetectTeamArtRedBoard();
            telemetry.update();
        }
        resetRuntime();
        control.autoTimeLeft = 0.0;

        // ***************************************************
        // ****  START DRIVING    ****************************
        // ***************************************************

        RedBoardDecision();
        /* Drive to the Board */
        Actions.runBlocking(
                new SequentialAction(
                        lockPixels(),
                        /* Drive to the board while moving arm up to scoring position after crossing the half-way point */
                        new ParallelAction(
                                BoardTraj2,
                                new SequentialAction(
                                        halfwayTrigger1(),
                                        new SleepAction(.15),
                                        halfwayTrigger2(),
                                        new SleepAction(.15),
                                        halfwayTrigger3()
                                )
                        )
                )
        );

        /* Use AprilTags to Align Perfectly to the Board */
        control.TagCorrection();
        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + .5, drive.pose.position.y - control.distanceCorrectionLR_HL), Math.toRadians(180))
                        .build());

        /* release pixels on the board using the distance sensor to know when to stop */
        control.StopNearBoardAuto(false);

        /* BACK UP FROM BOARD slightly so that the pixels fall off cleanly */
        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToX(46)
                        .build());
        drive.updatePoseEstimate();

        /* Determine path to Floor Position */
        RedRightPurplePixelDecision();

        // Calculate the path from the floor Position to the Stack
        // Build up the Floor to Stack Trajectory
        if (control.autoPosition == 1)
        {
            DriveToStack = drive.actionBuilder(deliverToFloorPose)
                    .splineToLinearHeading(new Pose2d(12,-56, Math.toRadians(180)), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(-36,-56), Math.toRadians(180), speedUpVelocityConstraint)
                    .strafeToLinearHeading(new Vector2d(-55.5,-56), Math.toRadians(180), speedUpVelocityConstraint)
                    .strafeToLinearHeading(new Vector2d(stackX, stackY), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(stackX - 2.0, stackY), Math.toRadians(180), slowDownVelocityConstraint)
                    .build();

        }
        else
        {
            DriveToStack = drive.actionBuilder(deliverToFloorPose)
                    .strafeToLinearHeading(new Vector2d(12,-56), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(-55.5,-56), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(stackX, stackY), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(stackX - 2.0, stackY), Math.toRadians(180), slowDownVelocityConstraint)
                    .build();
        }

        drive.useExtraCorrectionLogic = true;
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                FloorTraj,
                                new SequentialAction(
                                        resetArmPurple(),
                                        new SleepAction(.15),
                                        slidesDown()
                                )
                        ),
                        dropOnLine(),
                        new ParallelAction(
                                resetArm(),
                                servoIntake(),
                                DriveToStack)
                )
        );
        drive.useExtraCorrectionLogic = false;


        /* Use camera to make a minor adjustment to position if needed */
        control.StackCorrectionHL();
        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(stackX,drive.pose.position.y - control.distanceCorrectionLR_HL), Math.toRadians(180))
                        .build()
        );
        drive.updatePoseEstimate();

        //grab 2 more white pixels
        control.AutoPickupRoutineDrive(2.2);
        drive.updatePoseEstimate();

        // Build up the Stack to Board Position 3 Trajectory
        BoardTraj2 = drive.actionBuilder(drive.pose)
                //.setTangent(0)
                //.splineToLinearHeading(new Pose2d(-30, -11.5, Math.toRadians(180)), Math.toRadians(0))
                //.splineToLinearHeading(new Pose2d(47.5, -11.5, Math.toRadians(180)), Math.toRadians(0))
                //.setTangent(Math.toRadians(270))5
                //.splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(stackX + 1.0, -62), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(47,-62), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(47,-40), Math.toRadians(180))
                .build();
        //drive to position 3
        Actions.runBlocking(new SequentialAction(
                        autoGrab1(),
                        new SleepAction(.25),
                        new ParallelAction(
                                new SequentialAction(
                                        autoGrab2(),
                                        new SleepAction(.15),
                                        servoOuttake()
                                ),
                                BoardTraj2,
                                new SequentialAction(
                                        halfwayTrigger1(),
                                        new SleepAction(.15),
                                        halfwayTrigger2(),
                                        new SleepAction(.15),
                                        halfwayTrigger3()
                                )
                        )
                )
        );

        //deliver two white pixels
        control.StopNearBoardAuto(true);
        drive.updatePoseEstimate();
        //sleep(150);

        /* Park the Robot, and Reset the Arm and slides */
        Park = drive.actionBuilder(drive.pose)
                .lineToX(45, slowDownVelocityConstraint)
                .splineToLinearHeading(new Pose2d(48, -56, Math.toRadians(90)), Math.toRadians(90))
                .build();
        Actions.runBlocking(
                new ParallelAction(
                        Park,
                        new SequentialAction(
                                new SleepAction(.1),
                                resetArm(),
                                new SleepAction(.15),
                                slidesDown()
                        ),
                        servoStop()
                )
        );

        control.autoTimeLeft = 30-getRuntime();
        telemetry.addData("Time left", control.autoTimeLeft);
        telemetry.update();

    }
    public void RedBoardDecision() {
        // Look for potential errors
        //***POSITION 1***
        if (control.autoPosition == 1) {
            deliverToBoardPose = new Pose2d(46,-30,Math.toRadians(180));
        }
        //***POSITION 3***
        else if (control.autoPosition == 3) {
            deliverToBoardPose = new Pose2d(46,-42,Math.toRadians(180));
        }
        //***POSITION 2***
        else {
            deliverToBoardPose = new Pose2d(46,-36,Math.toRadians(180));
        }
        BoardTraj2 = drive.actionBuilder(startPose)
                .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                .build();
    }
    public void RedRightPurplePixelDecision() {
        if (control.autoPosition == 1) {
            deliverToFloorPose = new Pose2d(12, -33, Math.toRadians(0));
            FloorTraj = drive.actionBuilder(deliverToBoardPose)
                    .splineToLinearHeading(new Pose2d(27,-33, Math.toRadians(0)), Math.toRadians(180))
                    //.setTangent(Math.toRadians(180))
                    .lineToX(0)
                    .strafeToLinearHeading(new Vector2d(deliverToFloorPose.position.x, deliverToFloorPose.position.y), Math.toRadians(0))
                    //.splineToLinearHeading(new Pose2d(0,-33, Math.toRadians(0)), Math.toRadians(0))
                    //.splineToLinearHeading(deliverToFloorPose, Math.toRadians(0))
                    .build();
        }
        else if (control.autoPosition == 3) {
            deliverToFloorPose = new Pose2d(12, -36, Math.toRadians(180));
            FloorTraj = drive.actionBuilder(deliverToBoardPose)
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading (deliverToFloorPose, Math.toRadians(180))
                    .build();
        }
        else {
            deliverToFloorPose = new Pose2d(12, -36, Math.toRadians(270));
            FloorTraj = drive.actionBuilder(deliverToBoardPose)
                    .splineToLinearHeading(new Pose2d(12, -30, Math.toRadians(-90)), Math.toRadians(180))
                    .splineToLinearHeading(deliverToFloorPose, Math.toRadians(270))
                    .build();
        }
    }

    public Action lockPixels(){return new LockPixels();}
    public class LockPixels implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.GrabPixels();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action dropOnLine(){return new DropOnLine();}
    public class DropOnLine implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.DropOnLine();
                initialized = true;
            }
            packet.put("drop purple pixel on line", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action resetArm(){return new ResetArm();}
    public class ResetArm implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.ResetArmAuto();
                initialized = true;
            }
            packet.put("ResetArm", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action resetArmPurple(){return new ResetArmPurple();}
    public class ResetArmPurple implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.ResetArmBoard();
                initialized = true;
            }
            packet.put("ResetArm", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action slidesDown(){return new SlidesDown();}
    public class SlidesDown implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            packet.put("Slides Down", 0);
            boolean slidesAllDown = control.SlidesDownInParallel();
            return !slidesAllDown;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action autoGrab1(){return new AutoGrab1();}
    public class AutoGrab1 implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.AutoPickupRoutineStopAndLower();
                initialized = true;
            }
            packet.put("servoIntake", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action autoGrab2(){return new AutoGrab2();}
    public class AutoGrab2 implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.AutoPickupRoutineGrabAndUp();
                initialized = true;
            }
            packet.put("servoIntake", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action servoOuttake(){return new ServoOuttake();}
    public class ServoOuttake implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.ServoOuttake();
                initialized = true;
            }
            packet.put("ServoOuttake", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action servoIntake(){return new ServoIntake();}
    public class ServoIntake implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.ServoIntake();
                initialized = true;
            }
            packet.put("servoIntake", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action halfwayTrigger1(){return new HalfwayTrigger1();}
    public class HalfwayTrigger1 implements Action{
        public boolean run(@NonNull TelemetryPacket packet) {
            boolean moveArm = false;
            //drive.updatePoseEstimate();
            if (drive.pose.position.x >= 12) {
                moveArm = true;
                control.SlidesToAutoLow();
            }
            packet.put("move arm trigger", 0);
            return !moveArm;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action halfwayTrigger2(){return new HalfwayTrigger2();}
    public class HalfwayTrigger2 implements Action{
        public boolean run(@NonNull TelemetryPacket packet) {
            boolean moveArm = false;
            //drive.updatePoseEstimate();
            if (drive.pose.position.x >= 12) {
                moveArm = true;
                control.DeliverPixelToBoardPosPart1();
            }
            packet.put("move arm trigger", 0);
            return !moveArm;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action halfwayTrigger3(){return new HalfwayTrigger3();}
    public class HalfwayTrigger3 implements Action{
        public boolean run(@NonNull TelemetryPacket packet) {
            boolean moveArm = false;
            //drive.updatePoseEstimate();
            if (drive.pose.position.x >= 12) {
                moveArm = true;
                control.DeliverPixelToBoardPosPart2();
            }
            packet.put("move arm trigger", 0);
            return !moveArm;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action servoStop(){return new ServoStop();}
    public class ServoStop implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.ServoStop();
                initialized = true;
            }
            packet.put("drop purple pixel on line", 0);
            return false;
            }
    }

    public Action prepareToDropPurplePixel() {
        return new PrepareToDropPurplePixel();
    }

    public class PrepareToDropPurplePixel implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.ArmRotationsPurplePixelDelivery();
                control.WristRotationsPurplePixelDelivery();
                initialized = true;
            }
            packet.put("prepare to drop purple pixel on line", 0);
            return false;
        }
    }

    public Action armRotationsPurplePixelDelivery() {
        return new ArmRotationsPurplePixelDelivery();
    }

        public class ArmRotationsPurplePixelDelivery implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    control.ArmRotationsPurplePixelDelivery();
                    initialized = true;
                }
                packet.put("Rotate Arm to deliver purple pixel on line", 0);
                return false;
            }
        }
        public Action wristRotationsPurplePixelDelivery() {
            return new WristRotationsPurplePixelDelivery();
        }

        public class WristRotationsPurplePixelDelivery implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    control.WristRotationsPurplePixelDelivery();
                    initialized = true;
                }
                packet.put("Adjust wrist to deliver the purple pixel on line", 0);
                return false;
            }
        }
        public Action releasePurplePixel() {
            return new ReleasePurplePixel();
        }

        public class ReleasePurplePixel implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    control.ReleasePurplePixel();
                    initialized = true;
                }
                packet.put("Release purple pixel on line", 0);
                return false;
            }
        }

        public Action clearanceAfterPurpleDelivery() {
            return new ClearanceAfterPurpleDelivery();
        }

        public class ClearanceAfterPurpleDelivery implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    control.ClearanceAfterPurpleDelivery();
                    initialized = true;
                }
                packet.put("Clearance of arm mechanism after purple pixel delivery", 0);
                return false;
            }
        }
}

