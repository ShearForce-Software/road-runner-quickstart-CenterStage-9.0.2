package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name="Red Far", preselectTeleOp = "1 Manual Control")
public class RedFarAuto extends LinearOpMode {
    UniversalControlClass control = new UniversalControlClass(true, false,this);
    MecanumDrive drive;
    Pose2d startPose;
    Pose2d deliverToFloorPose;
    Pose2d deliverToBoardPose;

    public void runOpMode(){
        startPose = new Pose2d(-35.5,-62.5,Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, startPose);
        control.Init(hardwareMap);
        control.HuskyLensInit();
        control.AutoStartPos();

        telemetry.update();

        while(!isStarted()){
            control.DetectTeamArtRed();
            telemetry.update();
        }
        waitForStart();

        // lock the pixels
        control.GrabPixels();

        //Drives to LCR drop on line
        RedLeftTeamArtPixelDelivery();

        // drive to the middle floor delivery position
        /*Actions.runBlocking(
            drive.actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, -12.5, Math.toRadians(90)), Math.toRadians(90))
                .build());*/

        // Drop the LEFT pixel (put PURPLE on LEFT, YELLOW on RIGHT) on the line
        control.DropOnLine();
        // put the arm back in a safe to travel position
        control.SafeStow();
        control.SpecialSleep(8000);

        // drive to the backboard area
        Actions.runBlocking(
            drive.actionBuilder(deliverToFloorPose)
                    .splineToLinearHeading(new Pose2d(-38,-9, Math.toRadians(90)), Math.toRadians(90))
        //            .setTangent(0)
                    .splineToLinearHeading(new Pose2d(-30,-9, Math.toRadians(180)), Math.toRadians(0))
            //.setTangent(0)
                    .splineToLinearHeading(new Pose2d(30,-9, Math.toRadians(180)), Math.toRadians(0))
                    //.splineToLinearHeading(new Pose2d(52.2,-28, Math.toRadians(180)), Math.toRadians(0))
            .build());

        // Rotate Arm slightly so can flip arm over or raise safely
        //control.ReadyToLiftSlides();
        //sleep(150);

        //move arm, slides, drive, and release pixels to board
        RedBoardDelivery();

        // Return Arm to Ready position
        control.SlidesToAuto();
        sleep(150);
        control.DeliverPixelToBoardPos();
        control.StopNearBoardAuto(true);
        sleep(200);

        //Drop the pixel on the board
        control.ReleaseRight();
        control.ReleaseLeft();
        sleep(750);

        //return the arm to ready position
        Actions.runBlocking(
                drive.actionBuilder(deliverToBoardPose)
                        .lineToX(47)
                        .build());

        control.ResetArm();
        sleep(400);

        // Move the robot to the parking position
        Actions.runBlocking(
                //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                drive.actionBuilder(new Pose2d(50, -28, Math.toRadians(180)))
                        .splineToLinearHeading(new Pose2d(48,-15, Math.toRadians(90)), Math.toRadians(90))
                        .build());

        telemetry.update();
    }
    public void RedBoardDelivery() {
        // Look for potential errors
        if (control.autoPosition == 1) {
            deliverToBoardPose = new Pose2d(50,-22,Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(new Pose2d(30,-9, Math.toRadians(180)))
                            .setTangent(0)
                            .splineToLinearHeading(new Pose2d(50, -22, Math.toRadians(180)), Math.toRadians(0))
                            .build());
        } else if (control.autoPosition == 2) {
            deliverToBoardPose = new Pose2d(50,-28,Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(new Pose2d(30, -9, Math.toRadians(180)))
                            .setTangent(0)
                            .splineToLinearHeading(new Pose2d(50, -28, Math.toRadians(180)), Math.toRadians(0))
                            .build());
        } else if (control.autoPosition == 3) {
            deliverToBoardPose = new Pose2d(50,-34,Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(new Pose2d(30, -9, Math.toRadians(180)))
                            .setTangent(0)
                            .splineToLinearHeading(new Pose2d(50, -34, Math.toRadians(180)), Math.toRadians(0))
                            .build());
        } else {
            deliverToBoardPose = new Pose2d(50,-28,Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(new Pose2d(30, -9, Math.toRadians(180)))
                            .setTangent(0)
                            .splineToLinearHeading(new Pose2d(50, -28, Math.toRadians(180)), Math.toRadians(0))
                            .build());
        }
    }
    public void RedLeftTeamArtPixelDelivery() {

        Pose2d aTempPose = new Pose2d(-38.5, -33, Math.toRadians(90));

        Actions.runBlocking(
                //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                drive.actionBuilder(startPose)
                        .splineToLinearHeading(aTempPose, Math.toRadians(90))
                        .build());

        if (control.autoPosition == 3) {
            deliverToFloorPose = new Pose2d(-34.5, -32, Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(aTempPose)
                            .splineToLinearHeading (new Pose2d(-27, -33, Math.toRadians(180)), Math.toRadians(180))
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(180))
                            .build());
        } else if (control.autoPosition == 2) {
            deliverToFloorPose = new Pose2d(-38.5, -12.5, Math.toRadians(90));

            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(aTempPose)
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(90))
                            .build());
        } else if (control.autoPosition == 1) {
            deliverToFloorPose = new Pose2d(-38.5, -20.5, Math.toRadians(45));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  newa Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(aTempPose)
                            .splineToLinearHeading (deliverToFloorPose, Math.toRadians(45))
                            .build());
        } else {
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(-38.5, -12.5, Math.toRadians(90)), Math.toRadians(90))
                            .build());
        }
    }
}