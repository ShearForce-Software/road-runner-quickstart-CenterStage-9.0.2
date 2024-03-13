package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AidanBlueFarStackAutoTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);
       // Vector2d stackVec = new Vector2d(-56, 12);
       // Pose2d startPose = new Pose2d(-35.5, 62.5, Math.toRadians(270));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(18, 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-35.5, 62.5, 0))
                .setTangent(0)
                .splineTo(new Vector2d(-35.5, 11.6),0)
                .turn(Math.toRadians(180))
                .lineToX(-52)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }
}
