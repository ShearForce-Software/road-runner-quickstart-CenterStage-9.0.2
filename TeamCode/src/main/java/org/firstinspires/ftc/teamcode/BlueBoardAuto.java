package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Board", preselectTeleOp="1 Manual Control")
public class BlueBoardAuto extends LinearOpMode {
    UniversalControlClass control = new UniversalControlClass(true, false, this);
    MecanumDrive drive;
    Pose2d startPose;
    Pose2d deliverToFloorPose;
    Pose2d deliverToBoardPose;
    public void runOpMode() {
        startPose = new Pose2d(12, 62.5, Math.toRadians(270));
        drive = new MecanumDrive(hardwareMap, startPose);
        control.Init(hardwareMap);
        control.HuskyLensInit2();
        control.HuskyLensInit();
        control.AutoStartPos();

        drive.localizer.update();
        telemetry.update();

        while (!isStarted()) {
            control.DetectTeamArtBlueBoard();
            telemetry.update();
        }
        resetRuntime();

        // lock the pixels
        control.GrabPixels();

        // ***************************************************
        // ****  START DRIVING    ****************************
        // ***************************************************

        /* Drive to the Board */
        BlueBoardDelivery();

        /* Use AprilTags to Align Perfectly to the Board */
        control.TagCorrection();
        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + .5, drive.pose.position.y + control.distanceCorrectionLR_HL), Math.toRadians(180))
                        .build());

        /* release pixels on the board using the distance sensor to know when to stop */
        control.SlidesToAuto();
        sleep(150);
        control.DeliverPixelToBoardPos();
        control.StopNearBoardAuto(false);

        sleep(200);

        /* BACK UP FROM BOARD slightly so that the pixels fall off cleanly */
        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToX(47)
                        .build());
        drive.updatePoseEstimate();

        control.ResetArmAuto();
        sleep(150);
        control.SlidesDown();

        /* Drive to Floor Position */
        BlueLeftTeamArtPixelDelivery();
        drive.updatePoseEstimate();

        /* Deliver the Purple Pixel */
        control.SlidesToAuto(); //TODO TEMP - should be able to remove this once Husky Lens moved
        control.DropOnLine();

        control.ResetArm();
        sleep(400);
        control.SlidesDown();

        if(control.autoPosition == 2){
            Actions.runBlocking(
                    drive.actionBuilder(deliverToFloorPose)
                            .splineToLinearHeading(new Pose2d(12,42, Math.toRadians(270)), Math.toRadians(90))
                            .build());
        }
        drive.updatePoseEstimate();

        // Move the robot to the parking position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(55,55, Math.toRadians(270)), Math.toRadians(0))
                        .build());

        telemetry.update();
    }
    public void BlueBoardDelivery() {
        //***POSITION 1***
        if (control.autoPosition == 1) {
            deliverToBoardPose = new Pose2d(47.5,38,Math.toRadians(180));
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                            .build());
            drive.updatePoseEstimate();
        }
        //***POSITION 3***
        else if (control.autoPosition == 3) {
            deliverToBoardPose = new Pose2d(47.5,26,Math.toRadians(180));
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                            .build());
            drive.updatePoseEstimate();
        }
        //***POSITION 2***
        else {
            deliverToBoardPose = new Pose2d(47.5,33,Math.toRadians(180));
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                            .build());
            drive.updatePoseEstimate();
        }
    }

    public void BlueLeftTeamArtPixelDelivery() {
        if (control.autoPosition == 1) {
            deliverToFloorPose = new Pose2d(12, 32, Math.toRadians(180));
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .setTangent(Math.toRadians(180))
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(180))
                            .build());
            drive.updatePoseEstimate();
        }
        else if (control.autoPosition == 3) {
            deliverToFloorPose = new Pose2d(12, 36, Math.toRadians(0));
            Actions.runBlocking(
                    drive.actionBuilder(deliverToBoardPose)
                            .splineToLinearHeading(new Pose2d(27, 39, Math.toRadians(0)), Math.toRadians(180))
                            .lineToX(0)
                            .lineToX(12)
                            .strafeTo(new Vector2d(12,36))
                            .build());
            drive.updatePoseEstimate();
        }
        else {
            deliverToFloorPose = new Pose2d(12, 36, Math.toRadians(90));
            Actions.runBlocking(
                    drive.actionBuilder(deliverToBoardPose)
                            .splineToLinearHeading(new Pose2d(12, 30, Math.toRadians(90)), Math.toRadians(180))
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(90))
                            .build());
            drive.updatePoseEstimate();
        }
    }
}
