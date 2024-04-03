package com.example.meepmeep;


import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep_BlueBoardChris {
    static Pose2d startPose;
    static Pose2d stackPose;
    static Pose2d deliverToFloorPose;
    static Pose2d deliverToBoardPose;
    static Action FloorTraj;
    static Action BoardTraj2;
    static Action DriveToStack;
    static Action DriveBackToStack2;
    static Action BoardTrajFinal;
    static Action Park;
    static RoadRunnerBotEntity myBot;
    static int autoPosition = 1;
    static VelConstraint speedUpVelocityConstraint;
    static AccelConstraint speedUpAccelerationConstraint;
    static VelConstraint slowDownVelocityConstraint;
    static AccelConstraint slowDownAccelerationConstraint;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(450);

        startPose = new Pose2d(12, 62.5, Math.toRadians(270));
        stackPose = new Pose2d(-58, 36, Math.toRadians(180));

        // Define some custom constraints to use when wanting to go faster than defaults
        speedUpVelocityConstraint = new TranslationalVelConstraint(75.0);
        speedUpAccelerationConstraint = new ProfileAccelConstraint(-70.0, 70.0);
        slowDownVelocityConstraint = new TranslationalVelConstraint(30);
        slowDownAccelerationConstraint = new ProfileAccelConstraint(-30, 30);

        // Define the standard constraints to use for this robot
        myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.PI * .8, Math.PI, 15)
                .setDimensions(18, 15)
                .build();

        // ********************************************
        /* Specify which Pixel Position will be run  */
        // ********************************************
        autoPosition = 2;

        // Build up the start to board delivery trajectory
        //*****************************************************
        BlueBoardDecision();

        // Build up the Board to Floor Trajectory
        //*****************************************************
        BlueRightPurplePixelDecision();

        // Build up the Floor to Stack Trajectory (First Stack Run)
        //*****************************************************
        if (autoPosition == 1)
        {
            DriveToStack = myBot.getDrive().actionBuilder(deliverToFloorPose)
                    .strafeToLinearHeading(new Vector2d(12,60), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(-53,60), Math.toRadians(180),speedUpVelocityConstraint)
                    .strafeToLinearHeading(new Vector2d(stackPose.position.x, stackPose.position.y), Math.toRadians(180),slowDownVelocityConstraint)
                    .build();
        }
        else if (autoPosition == 2) {
            DriveToStack = myBot.getDrive().actionBuilder(deliverToFloorPose)
                    // turn and go through center rigging
                    .turnTo(Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(stackPose.position.x, stackPose.position.y), Math.toRadians(180),speedUpVelocityConstraint)
                    .build();

                    // strafe to wall and go around and through wall rigging
                    //.strafeToLinearHeading(new Vector2d(12,60), Math.toRadians(180))
                    //.strafeToLinearHeading(new Vector2d(-53,60), Math.toRadians(180))
                    //.strafeToLinearHeading(new Vector2d(stackPose.position.x, stackPose.position.y), Math.toRadians(180))
                    //.build();
        }

        else
        {
            DriveToStack = myBot.getDrive().actionBuilder(deliverToFloorPose)
                    .splineToLinearHeading(new Pose2d(12,60, Math.toRadians(180)), Math.toRadians(180),speedUpVelocityConstraint)
                    .strafeToLinearHeading(new Vector2d(-53,60), Math.toRadians(180),speedUpVelocityConstraint)
                    .strafeToLinearHeading(new Vector2d(stackPose.position.x, stackPose.position.y), Math.toRadians(180),slowDownVelocityConstraint)
                    .build();

        }

        // Build up the Board to Stack Trajectory (Second Stack Run)
        //*****************************************************
        if (autoPosition == 1) {
            DriveBackToStack2 = myBot.getDrive().actionBuilder(new Pose2d(47, 36, Math.toRadians(180)))
                    .strafeToLinearHeading(new Vector2d(24, 60), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(-53, 60), Math.toRadians(180),speedUpVelocityConstraint)
                    .strafeToLinearHeading(new Vector2d(stackPose.position.x, stackPose.position.y), Math.toRadians(180),slowDownVelocityConstraint)
                    .build();
        }
        else if (autoPosition == 2)
        {
            DriveBackToStack2 = myBot.getDrive().actionBuilder(new Pose2d(47, 36, Math.toRadians(180)))
                    .strafeToLinearHeading(new Vector2d(stackPose.position.x, stackPose.position.y), Math.toRadians(180),speedUpVelocityConstraint)
                    .build();
        }
        else
        {
            DriveBackToStack2 = myBot.getDrive().actionBuilder(new Pose2d(47, 36, Math.toRadians(180)))
                    .strafeToLinearHeading(new Vector2d(24, 60), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(-53, 60), Math.toRadians(180),speedUpVelocityConstraint)
                    .strafeToLinearHeading(new Vector2d(stackPose.position.x, stackPose.position.y), Math.toRadians(180),slowDownVelocityConstraint)
                    .build();
        }
        // Build up the Stack to Board Position 2 Trajectory
        //*****************************************************
        if (autoPosition == 1)
        {
            BoardTrajFinal = myBot.getDrive().actionBuilder(stackPose)
                    //.setTangent(0)
                    //.splineToLinearHeading(new Pose2d(-30, 11.5, Math.toRadians(180)), Math.toRadians(0))
                    //.splineToLinearHeading(new Pose2d(47.5, 11.5, Math.toRadians(180)), Math.toRadians(0))
                    //.setTangent(Math.toRadians(270))5
                    //.splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                    .strafeToLinearHeading(new Vector2d(-53, 60), Math.toRadians(180),slowDownVelocityConstraint)
                    .strafeToLinearHeading(new Vector2d(24,60), Math.toRadians(180),speedUpVelocityConstraint)
                    .strafeToLinearHeading(new Vector2d(47,36), Math.toRadians(180),slowDownVelocityConstraint)
                    .build();
        }
        else if (autoPosition == 2)
        {
            BoardTrajFinal = myBot.getDrive().actionBuilder(stackPose)
                    .strafeToLinearHeading(new Vector2d(47,36), Math.toRadians(180))
                    .build();
        }
        else
        {
            BoardTrajFinal = myBot.getDrive().actionBuilder(stackPose)
                    .strafeToLinearHeading(new Vector2d(-53, 60), Math.toRadians(180),slowDownVelocityConstraint)
                    .strafeToLinearHeading(new Vector2d(24,60), Math.toRadians(180),speedUpVelocityConstraint)
                    .strafeToLinearHeading(new Vector2d(47,36), Math.toRadians(180),slowDownVelocityConstraint)
                    .build();
        }

        // Build up the Board position 2 to Parking Trajectory
        //*****************************************************
        //Action Park = myBot.getDrive().actionBuilder(new Pose2d(46, deliverToBoardPose.position.y, Math.toRadians(180)))
        Park = myBot.getDrive().actionBuilder(new Pose2d(47, 36, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(48, 60, Math.toRadians(270)), Math.toRadians(90),speedUpVelocityConstraint)
                //.strafeToLinearHeading(new Vector2d(48, 10), Math.toRadians(90))
                //.turnTo(Math.toRadians(90))
                .build();


        // Assemble Action Sequence
        //*****************************************************

        myBot.runAction(new SequentialAction(
                // Drive to Board Position
                BoardTraj2,
                // simulate waiting for board delivery and april tag correction
                new SleepAction(0.6),
                FloorTraj,
                // simulate waiting for purple pixel delivery & resetArm
                new SleepAction(.6),
                DriveToStack,
                // simulate waiting to get to the white pixel pickup location
                new SleepAction(0.7),
                // simulate waiting to pick up the pixels
                new SleepAction(.75),
                // Drive to Board Position 3

                BoardTrajFinal,
                // simulate waiting for board delivery
                new SleepAction(0.6),
                // Drive to the parking spot

                DriveBackToStack2,
                // simulate waiting to get to the white pixel pickup location
                new SleepAction(0.7),
                // simulate waiting to pick up the pixels
                new SleepAction(.75),
                // Drive to Board Position 3

                BoardTrajFinal,
                // simulate waiting for board delivery
                new SleepAction(0.6),
                // Drive to the parking spot

                Park
                // simulate waiting for slides down, and servo stop
                //, new SleepAction(0.8)
                ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

        static public void BlueRightPurplePixelDecision() {
            //***POSITION 1***
            if (autoPosition == 1) {
                deliverToFloorPose = new Pose2d(12, 33, Math.toRadians(180));
                FloorTraj = myBot.getDrive().actionBuilder(deliverToBoardPose)
                        //.splineToLinearHeading(new Pose2d(36,33, Math.toRadians(0)), Math.toRadians(180))
                        // lower arm to floor, then push to line and release
                        //.lineToX(0) // push team out of the way then drop pixel on line
                        .strafeTo(new Vector2d(deliverToFloorPose.position.x,deliverToFloorPose.position.y))
                        //.strafeToLinearHeading(new Vector2d(deliverToFloorPose.position.x, deliverToFloorPose.position.y), Math.toRadians(0))
                        //.splineToLinearHeading(new Pose2d(0,33, Math.toRadians(0)), Math.toRadians(0))
                        //.splineToLinearHeading(deliverToFloorPose, Math.toRadians(0))
                        .build();
            }
            //***POSITION 3***
            else if (autoPosition == 3) {
                deliverToFloorPose = new Pose2d(12, 36, Math.toRadians(0));
                FloorTraj = myBot.getDrive().actionBuilder(deliverToBoardPose)
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(24, 36, Math.toRadians(0)), Math.toRadians(180))
                        .strafeTo(new Vector2d(deliverToFloorPose.position.x,deliverToFloorPose.position.y))
                        .build();
            }
            //***POSITION 2***
            else {
                deliverToFloorPose = new Pose2d(12, 36, Math.toRadians(90));
                FloorTraj = myBot.getDrive().actionBuilder(deliverToBoardPose)
                        .splineToLinearHeading(new Pose2d(12, 30, Math.toRadians(90)), Math.toRadians(180))
                        .strafeTo(new Vector2d(deliverToFloorPose.position.x,deliverToFloorPose.position.y))
                        .build();
            }
        }

        static public void BlueBoardDecision() {
            // Look for potential errors
            //***POSITION 1***
            if (autoPosition == 1) {
                deliverToBoardPose = new Pose2d(47,42,Math.toRadians(180));
            }
            //***POSITION 3***
            else if (autoPosition == 3) {
                deliverToBoardPose = new Pose2d(47,30,Math.toRadians(180));
            }
            //***POSITION 2***
            else {
                deliverToBoardPose = new Pose2d(47,36,Math.toRadians(180));
            }
            BoardTraj2 = myBot.getDrive().actionBuilder(startPose)
                    .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                    .build();
        }
    }

