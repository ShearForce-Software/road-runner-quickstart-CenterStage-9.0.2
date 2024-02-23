package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Far Stack SMOOTH", preselectTeleOp = "1 Manual Control")
public class RedFarStackSmooth extends LinearOpMode {
    UniversalControlClass control = new UniversalControlClass(true, false, this);
    MecanumDrive drive;
    Pose2d startPose;
    Pose2d deliverToFloorPose;
    Pose2d deliverToBoardPose;
    Pose2d stackPose = new Pose2d(-56, -12, Math.toRadians(180));
    public void runOpMode() {
        startPose = new Pose2d(-35.5, -62.5, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, startPose);
        control.Init(hardwareMap);
        control.HuskyLensInit();
        control.AutoStartPos();

        telemetry.update();

        while (!isStarted()) {
            control.DetectTeamArtRed();
            telemetry.update();
        }
        waitForStart();

        // lock the pixels
        control.GrabPixels();
        control.ReleaseLeft();

        // Drives to left, center, or right positions based on team art location.
        RedLeftTeamArtPixelDelivery();

        // Drop the LEFT pixel (put PURPLE on LEFT, YELLOW on RIGHT) on the line
        control.DropOnLine();
        // put the arm back in a safe to travel position
        control.ResetArmAuto();
        sleep(150);
        control.SlidesDown();

        if(control.autoPosition==2){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(-38.5, -14))
                    .build());
        }
        drive.updatePoseEstimate();
        //control.SpecialSleep(10000);
        control.SpecialSleep(6000);
        control.ServoIntake();
        // drive to stack
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(stackPose, Math.toRadians(180))
                        .build()
        );
        control.AutoPickupRoutine();

        // drive to the correct backboard spot based on the team art
        drive.updatePoseEstimate();
        RedBoardDelivery();

        // Return Arm to Ready position
        control.SlidesToAuto();
        sleep(150);
        control.DeliverPixelToBoardPos();
        control.StopNearBoardAuto(true);
        sleep(200);

        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToX(47)
                        .build());

        // Return Arm to Ready position
        control.ResetArm();
        sleep(300);

        /*//TEST MULTIPLE CYCLES
        drive.updatePoseEstimate();
        //drive back to stack
        Actions.runBlocking(
                //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                drive.actionBuilder(drive.pose)
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(30,9, Math.toRadians(180)), Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-50,12, Math.toRadians(180)), Math.toRadians(180))
                        .build());
        control.AutoPickupRoutine();
        //drive back to board
        drive.updatePoseEstimate();
        BlueBoardDelivery();

        // Return Arm to Ready position
        control.SlidesToAuto();
        sleep(150);
        control.DeliverPixelToBoardPos();
        control.StopNearBoardAuto(true);
        sleep(200);

        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(180)))
                        .lineToX(47)
                        .build());

        control.ResetArmAuto();
        sleep(150);
        control.SlidesDown();

        sleep(300);
        //END OF TEST MULTIPLE CYCLES*/

        // Move the robot to the parking position
        drive.updatePoseEstimate();
        Actions.runBlocking(
                //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(new Pose2d(48, -9, Math.toRadians(90)), Math.toRadians(90))
                        .build());
        control.ServoStop();
        sleep(100);
        telemetry.update();
    }
    public void RedBoardDelivery() {
        // Look for potential errors
        //***POSITION 1***
        if (control.autoPosition == 1) {
            deliverToBoardPose = new Pose2d(49,-27,Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.
                    // yawError, Math.toRadians(180)))
                    drive.actionBuilder(drive.pose)
                            .setTangent(0)
                            .splineToLinearHeading(new Pose2d(-30, -7, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(49, -7, Math.toRadians(180)), Math.toRadians(0))
                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                            .build());
            drive.updatePoseEstimate();
        }
        //***POSITION 3***
        else if (control.autoPosition == 3) {
            deliverToBoardPose = new Pose2d(49,-34,Math.toRadians(180));
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .setTangent(0)
                            .splineToLinearHeading(new Pose2d(-30, -7, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(49, -7, Math.toRadians(180)), Math.toRadians(0))
                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                            .build());
            drive.updatePoseEstimate();
        }
        //***POSITION 2***
        else {
            deliverToBoardPose = new Pose2d(49,-30,Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(drive.pose)
                            .setTangent(0)
                            .splineToLinearHeading(new Pose2d(-30, -7, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(49, -7, Math.toRadians(180)), Math.toRadians(0))
                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                            .build());
            drive.updatePoseEstimate();
        }
    }
    public void RedLeftTeamArtPixelDelivery() {
        //***POSITION 1***
        if (control.autoPosition == 1) {
            deliverToFloorPose = new Pose2d(-39.5, -20.5, Math.toRadians(45));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  newa Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                            .splineToLinearHeading (deliverToFloorPose, Math.toRadians(45))
                            .build());

        }
        //***POSITION 3***
        else if (control.autoPosition == 3) {
            deliverToFloorPose = new Pose2d(-34.5, -32, Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(-27, -33, Math.toRadians(180)), Math.toRadians(180))
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(180))
                            .build());
        }
        //***POSITION 2***
        else {
            deliverToFloorPose = new Pose2d(-38.5, -12.5, Math.toRadians(90));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(90))
                            .build());
        }
    }
    public Action stopSpinners() {
        return new StopSpinners();
    }
    public class StopSpinners implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.ServoStop();
                initialized = true;
            }


            packet.put("disable Intakes", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }

    }
    public Action driveAcrossField()
    {
        return new DriveAcrossField();
    }
    public class DriveAcrossField implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                drive.updatePoseEstimate();
                drive.actionBuilder(drive.pose)
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(-30, 9, Math.toRadians(180)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(30, 9, Math.toRadians(180)), Math.toRadians(0))
                        .build();
            }
            packet.put("drive across blue field", 0);
            boolean returnValue = true;

            drive.updatePoseEstimate();
            if ((drive.pose.position.x == 30) && (drive.pose.position.y == 9))
            {
                returnValue = false;
            }
            return returnValue;
        }

    }
}
