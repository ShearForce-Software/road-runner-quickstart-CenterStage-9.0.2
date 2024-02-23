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
@Autonomous(name="Red Far Multiple Cycles Actions"/*, preselectTeleOp = "1 Manual Control"*/)
public class RedFarMultipleCyclesActions extends LinearOpMode {
    UniversalControlClass control = new UniversalControlClass(true, false,this);
    MecanumDrive drive;
    Pose2d startPose;
    Pose2d deliverToFloorPose;
    Pose2d deliverToBoardPose;
    Pose2d stackPose;
    Action FloorTraj;
    Action DriveToStack;
    Action BoardTraj2;
    Action BoardTrajFinal;
    Action Park;
    Action DriveBackToStack;
    Action DriveBackToStack2;
    VelConstraint speedUpVelocityConstraint;
    AccelConstraint speedUpAccelerationConstraint;
    public void runOpMode(){
        startPose = new Pose2d(-35.5,-62.5,Math.toRadians(90));
        stackPose = new Pose2d(-54.5, -11.5, Math.toRadians(180));

        speedUpVelocityConstraint = new TranslationalVelConstraint(90.0); //TODO Need to add a speed-up Velocity constraint to some of the trajectories
        speedUpAccelerationConstraint = new ProfileAccelConstraint(-70.0, 70.0);    //TODO need to determine is an acceleration constraint on some trajectories would be useful

        /* Initialize the Robot */
        drive = new MecanumDrive(hardwareMap, startPose);
        control.Init(hardwareMap);
        control.HuskyLensInit();
        control.AutoStartPos();
        telemetry.update();

        while(!isStarted()){
            control.DetectTeamArtRed();
            telemetry.update();//make decisions
            RedLeftPurplePixelDecision();
        }
        resetRuntime();

        DriveToStack = drive.actionBuilder(deliverToFloorPose)
                .splineToLinearHeading(stackPose, Math.toRadians(180))
                .build();

        // ***************************************************
        // ****  START DRIVING    ****************************
        // ***************************************************
        Actions.runBlocking(
                new SequentialAction(
                        /* Drive to Floor Position */
                        new ParallelAction(
                                lockPixels(),
                                FloorTraj),
                        /* Deliver the Purple Pixel */
                        dropOnLine(), //TODO -- takes too long, need to see if can split up and make parts of it parallel
                        new ParallelAction(
                                resetArm(),
                                servoIntake(),
                                DriveToStack)
                )
        );

        /* Pick up a White Pixel from the stack */
        control.AutoPickupRoutineDrive();
        drive.updatePoseEstimate();

        /* Drive to the board while moving arm up to scoring position after crossing the half-way point */
        RedBoardDecision(); // updates BoardTraj2
        Actions.runBlocking(new SequentialAction(
                autoGrab1(),
                new SleepAction(.5),
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
                                halfwayTrigger2()
                                )
                        )
                )
        );

        /* Use AprilTags to Align Perfectly to the Board */
        control.TagCorrection();
        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + .5, drive.pose.position.y + control.distanceCorrectionLR_HL), Math.toRadians(180))
                        .build());

        /* release pixels on the board using the distance sensor to know when to stop */
        control.StopNearBoardAuto(true);

        /* BACK UP FROM BOARD slightly so that the pixels fall off cleanly */
        drive.updatePoseEstimate();
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .lineToX(46)
                .build());

        // **********************************************************
        // ******    Begin Logic to get an extra 2 White Pixels *****
        // **********************************************************
        /* move arm to reset position while strafing to the side, before driving back to the stack of white pixels */
        drive.updatePoseEstimate();
        DriveBackToStack = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(45, -11.5), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(stackPose.position.x, stackPose.position.y), Math.toRadians(180))
                .build();
        Actions.runBlocking(
                new ParallelAction(
                        DriveBackToStack,
                        new SequentialAction(
                                resetArm(),
                                new SleepAction(.15),
                                slidesDown()
                        ),
                        servoIntake()
                )
        );

        /* move slides down and drive back to stack */
        drive.updatePoseEstimate();

        //grab 2 more white pixels
        control.AutoPickupRoutineDrive();
        drive.updatePoseEstimate();

        //drive to position 1
        BoardTraj2 = drive.actionBuilder(drive.pose)
                .setTangent(0)
                //.splineToLinearHeading(new Pose2d(-30, -11.5, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(47.5, -11.5, Math.toRadians(180)), Math.toRadians(0))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(47.5, -28, Math.toRadians(180)), Math.toRadians(270))
                .build();

        Actions.runBlocking(new SequentialAction(
                        autoGrab1(),
                        new SleepAction(.5),
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
                                        halfwayTrigger2()
                                )
                        )
                )
        );

        //deliver two white pixels
        control.StopNearBoardAuto(true);
        drive.updatePoseEstimate();

        /* Park the Robot, and Reset the Arm and slides */
        Park = drive.actionBuilder(drive.pose)
                .lineToX(46)
                //.splineToLinearHeading(new Pose2d(48, -6, Math.toRadians(90)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(48, -10), Math.toRadians(90))
                .build();
        Actions.runBlocking(
                new ParallelAction(
                        Park,
                        new SequentialAction(
                                resetArm(),
                                new SleepAction(.15),
                                slidesDown()
                        ),
                        servoStop()
                )
        );
    }

    public void RedBoardDecision() {
        // Look for potential errors
        //***POSITION 1***
        if (control.autoPosition == 1) {
            deliverToBoardPose = new Pose2d(47.5,-28,Math.toRadians(180));
            BoardTraj2 = drive.actionBuilder(drive.pose)
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(47.5, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .setTangent(Math.toRadians(270))
                    .splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                    .build();
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))

        }
        //***POSITION 3***
        else if (control.autoPosition == 3) {
            deliverToBoardPose = new Pose2d(47.5,-38,Math.toRadians(180));
            BoardTraj2 = drive.actionBuilder(drive.pose)
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(47.5, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .setTangent(Math.toRadians(270))
                    .splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                    .build();
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
        }
        //***POSITION 2***
        else {
            deliverToBoardPose = new Pose2d(47.5,-33,Math.toRadians(180));
            BoardTraj2 = drive.actionBuilder(drive.pose)
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(47.5, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .setTangent(Math.toRadians(270))
                    .splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                    .build();
        }
    }
    public void RedLeftPurplePixelDecision() {
        //***POSITION 1***
        if (control.autoPosition == 1) {
            deliverToFloorPose = new Pose2d(-39.5, -20.5, Math.toRadians(45));
            FloorTraj = drive.actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                    .splineToLinearHeading (deliverToFloorPose, Math.toRadians(45))
                    //.strafeToLinearHeading(new Vector2d(-38.5, -33), Math.toRadians(90))
                    //.strafeToLinearHeading(new Vector2d(deliverToFloorPose.position.x, deliverToFloorPose.position.y), Math.toRadians(45))
                    .build();
        }
        //***POSITION 3***
        else if (control.autoPosition == 3) {
            deliverToFloorPose = new Pose2d(-34.5, -32, Math.toRadians(180));
            FloorTraj = drive.actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                    //.splineToLinearHeading(new Pose2d(-27, -33, Math.toRadians(180)), Math.toRadians(180))
                    //.splineToLinearHeading(deliverToFloorPose, Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(-27, -33), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(deliverToFloorPose.position.x, deliverToFloorPose.position.y), Math.toRadians(180))
                    .build();
        }
        //***POSITION 2***
        else {
            deliverToFloorPose = new Pose2d(-38.5, -12.5, Math.toRadians(90));
            FloorTraj = drive.actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                    .splineToLinearHeading(deliverToFloorPose, Math.toRadians(90))
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
                control.ReleaseLeft();
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
                control.DropOnLine(); //TODO split up this logic, it has a bunch of sleeps in it, should do some of this in parallel with driving
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
                control.SlidesToAuto();
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
                control.DeliverPixelToBoardPosTest();
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
}

