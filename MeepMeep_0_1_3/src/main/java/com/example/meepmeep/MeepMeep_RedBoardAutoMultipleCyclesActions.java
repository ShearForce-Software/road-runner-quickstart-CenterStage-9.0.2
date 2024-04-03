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

public class MeepMeep_RedBoardAutoMultipleCyclesActions {
    static Pose2d startPose;
    static Pose2d stackPose;
    static Pose2d deliverToFloorPose;
    static Pose2d deliverToBoardPose;
    static Action FloorTraj;
    static Action BoardTraj2;
    static Action DriveToStack;
    static RoadRunnerBotEntity myBot;
    static int autoPosition = 1;
    static VelConstraint speedUpVelocityConstraint;
    static AccelConstraint speedUpAccelerationConstraint;
    static VelConstraint slowDownVelocityConstraint;
    static AccelConstraint slowDownAccelerationConstraint;

    static double stackY = -36.0;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        startPose = new Pose2d(12, -62.5, Math.toRadians(90));
        stackPose = new Pose2d(-55.5, stackY, Math.toRadians(180));

        speedUpVelocityConstraint = new TranslationalVelConstraint(90.0); //TODO Need to add a speed-up Velocity constraint to some of the trajectories
        speedUpAccelerationConstraint = new ProfileAccelConstraint(-70.0, 70.0);    //TODO need to determine is an acceleration constraint on some trajectories would be useful
        slowDownVelocityConstraint = new TranslationalVelConstraint(5); //TODO Need to add a slow-down Velocity constraint to some of the trajectories
        slowDownAccelerationConstraint = new ProfileAccelConstraint(-30, 30);    //TODO need to determine is an acceleration constraint on some trajectories would be useful

        // Define the standard constraints to use for this robot
        myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.PI*.6, Math.PI*.8, 18.959)
                .setDimensions(18, 15)
                .build();

        // ******************************************
        /* Specify which Position will be run */
        // ******************************************
        autoPosition = 3;

        // Build up the start to board delivery trajectory
        RedBoardDecision();

        // Build up the Board to Floor Trajectory
        RedRightPurplePixelDecision();

        // Build up the Floor to Stack Trajectory
        // if autoPosition = 1, then need to take extra care to avoid hitting alliance purple pixel
        //    and have to spline out from dropping our pixel instead of just strafing
        if (autoPosition == 1)
        {
            DriveToStack = myBot.getDrive().actionBuilder(deliverToFloorPose)
                    .splineToLinearHeading(new Pose2d(12,-56, Math.toRadians(180)), Math.toRadians(180))
                    .strafeToConstantHeading(new Vector2d(-40, -56), speedUpVelocityConstraint)
                    .strafeToConstantHeading(new Vector2d(-54, -56), speedUpVelocityConstraint)
                    .strafeToConstantHeading(new Vector2d(-54, stackY), speedUpVelocityConstraint)
                    .strafeToLinearHeading(new Vector2d(stackPose.position.x, stackY), Math.toRadians(180))
                    .build();

        }
        else
        {
            DriveToStack = myBot.getDrive().actionBuilder(deliverToFloorPose)
                    .strafeToLinearHeading(new Vector2d(12,-56), Math.toRadians(180))
                    .strafeToConstantHeading(new Vector2d(-36, -56), speedUpVelocityConstraint)
                    //.strafeToConstantHeading(new Vector2d(-54, -56), speedUpVelocityConstraint)
                    //.strafeToConstantHeading(new Vector2d(-54, stackY), speedUpVelocityConstraint)
                    //.splineToLinearHeading(new Pose2d(-54, stackY, Math.toRadians(180)), Math.toRadians(180), speedUpVelocityConstraint)
                    .splineToLinearHeading(new Pose2d(-54, stackY, Math.toRadians(180)), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(stackPose.position.x, stackY), Math.toRadians(180))
                    .build();
        }
        /////
       // Action DriveBackToStack2 = myBot.getDrive().actionBuilder(new Pose2d(48,-11.5,Math.toRadians(180)))
        //        .strafeToLinearHeading(new Vector2d(stackPose.position.x, stackPose.position.y), Math.toRadians(180), speedUpVelocityConstraint)
         //       .build();

        Action BoardTrajFinal;
        // Build up the Stack to Board Position 3 Trajectory
        // if position 1 then need to do extra care to avoid hitting alliance purple pixel
        if (autoPosition == 1) {
            BoardTrajFinal = myBot.getDrive().actionBuilder(stackPose)
                    //.setTangent(0)
                    //.splineToLinearHeading(new Pose2d(-30, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    //.splineToLinearHeading(new Pose2d(47.5, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    //.setTangent(Math.toRadians(270))5
                    //.splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                    .strafeToConstantHeading(new Vector2d(-54, stackY))
                    .strafeToLinearHeading(new Vector2d(-54, -56), Math.toRadians(180), speedUpVelocityConstraint)
                    .strafeToLinearHeading(new Vector2d(-40, -56), Math.toRadians(180), speedUpVelocityConstraint)
                    .strafeToLinearHeading(new Vector2d(12, -56), Math.toRadians(180), speedUpVelocityConstraint)
                    .strafeToLinearHeading(new Vector2d(30, -56), Math.toRadians(180), speedUpVelocityConstraint)
                    //.strafeToLinearHeading(new Vector2d(47,-40), Math.toRadians(180), speedUpVelocityConstraint)
                    .splineToLinearHeading(new Pose2d(47, -40, Math.toRadians(180)), Math.toRadians(0))
                    .build();
        }
        else {
            BoardTrajFinal = myBot.getDrive().actionBuilder(stackPose)
                    //.setTangent(0)
                    //.splineToLinearHeading(new Pose2d(-30, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    //.splineToLinearHeading(new Pose2d(47.5, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    //.setTangent(Math.toRadians(270))5
                    //.splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                    .strafeToConstantHeading(new Vector2d(-54, stackY))
                    .splineToLinearHeading(new Pose2d(-36, -56, Math.toRadians(180)), Math.toRadians(0))
//                    .strafeToLinearHeading(new Vector2d(-54, -56), Math.toRadians(180), speedUpVelocityConstraint)
//                    .strafeToLinearHeading(new Vector2d(-40, -56), Math.toRadians(180), speedUpVelocityConstraint)
                    .strafeToLinearHeading(new Vector2d(12, -56), Math.toRadians(180), speedUpVelocityConstraint)
                    .strafeToLinearHeading(new Vector2d(30, -56), Math.toRadians(180), speedUpVelocityConstraint)
                    //.strafeToLinearHeading(new Vector2d(47,-40), Math.toRadians(180), speedUpVelocityConstraint)
                    .splineToLinearHeading(new Pose2d(47, -40, Math.toRadians(180)), Math.toRadians(0))
                    .build();
        }
        // Build up the Board position 3 to Parking Trajectory
        //Action Park = myBot.getDrive().actionBuilder(new Pose2d(46, deliverToBoardPose.position.y, Math.toRadians(180)))
        Action Park = myBot.getDrive().actionBuilder(new Pose2d(47, -40, Math.toRadians(180)))
                .lineToX(45, slowDownVelocityConstraint)
                //.splineToLinearHeading(new Pose2d(48, -56, Math.toRadians(90)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(48, -56), Math.toRadians(90))
                .build();

        myBot.runAction(new SequentialAction(
                // Drive to Board Position
                BoardTraj2,
                // simulate waiting for board delivery and april tag correction
                new SleepAction(0.6),
                FloorTraj,
                // simulate waiting for purple pixel delivery & resetArm
                new SleepAction(1.0),
                DriveToStack,
                // simulate waiting to get to the white pixel pickup location
                new SleepAction(0.7),
                // simulate waiting to pick up the pixels
                new SleepAction(1.05),
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


    static public void RedBoardDecision() {
        // Look for potential errors
        //***POSITION 1***
        if (autoPosition == 1) {
            deliverToBoardPose = new Pose2d(47,-30,Math.toRadians(180));
        }
        //***POSITION 3***
        else if (autoPosition == 3) {
            deliverToBoardPose = new Pose2d(47,-42,Math.toRadians(180));
        }
        //***POSITION 2***
        else {
            deliverToBoardPose = new Pose2d(47,-36,Math.toRadians(180));
        }
        BoardTraj2 = myBot.getDrive().actionBuilder(startPose)
                .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                .build();
    }

    static public void RedRightPurplePixelDecision() {
        //***POSITION 1***
        if (autoPosition == 1) {
            deliverToFloorPose = new Pose2d(12, -33, Math.toRadians(0));
            FloorTraj = myBot.getDrive().actionBuilder(deliverToBoardPose)
                    .splineToLinearHeading(new Pose2d(27,-33, Math.toRadians(0)), Math.toRadians(180))
                    //.setTangent(Math.toRadians(180))
                    .lineToX(0)
                    .strafeToLinearHeading(new Vector2d(deliverToFloorPose.position.x, deliverToFloorPose.position.y), Math.toRadians(0))
                    //.splineToLinearHeading(new Pose2d(0,-33, Math.toRadians(0)), Math.toRadians(0))
                    //.splineToLinearHeading(deliverToFloorPose, Math.toRadians(0))
                    .build();
        }
        //***POSITION 3***
        else if (autoPosition == 3) {
            deliverToFloorPose = new Pose2d(12, -36, Math.toRadians(180));
            FloorTraj = myBot.getDrive().actionBuilder(deliverToBoardPose)
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading (deliverToFloorPose, Math.toRadians(180))
                    .build();
        }
        //***POSITION 2***
        else {
            deliverToFloorPose = new Pose2d(12, -36, Math.toRadians(270));
            FloorTraj = myBot.getDrive().actionBuilder(deliverToBoardPose)
                    .splineToLinearHeading(new Pose2d(12, -30, Math.toRadians(-90)), Math.toRadians(180))
                    .splineToLinearHeading(deliverToFloorPose, Math.toRadians(270))
                    .build();
        }
    }

    }

