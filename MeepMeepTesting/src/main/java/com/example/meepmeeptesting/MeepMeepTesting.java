package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(driveTrain ->
                        driveTrain.trajectorySequenceBuilder(new Pose2d(new Vector2d(36,-66),Math.PI/2))
                                .splineTo(new Vector2d(36,-36),Math.PI/2)
                                .splineTo(new Vector2d(42,-13),0)
                                .setReversed(true)
                                .splineTo(new Vector2d(16,-13),Math.PI)
                                .setReversed(false)
                                .splineTo(new Vector2d(60,-12),0)
                                .waitSeconds(0.25)
                                .setReversed(true)
                                .splineTo(new Vector2d(16,-13),Math.PI)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}