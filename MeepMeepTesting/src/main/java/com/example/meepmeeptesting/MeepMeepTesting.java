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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), toInches(35))
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(toInches(-90), toInches(-159), Math.toRadians(90)))
                                .splineTo(new Vector2d(toInches(-81),toInches(-81)), Math.toRadians(45))
                                .turn(Math.toRadians(60))
                                .splineTo(new Vector2d(toInches(-140), toInches(-30)), Math.toRadians(180))
                                .splineTo(new Vector2d(toInches(-81), toInches(-39)), Math.toRadians(135))
                                .splineTo(new Vector2d(toInches(-140), toInches(-30)), Math.toRadians(180))
                                .splineTo(new Vector2d(toInches(-81), toInches(-39)), Math.toRadians(135))
                                .splineTo(new Vector2d(toInches(-140), toInches(-30)), Math.toRadians(180))
                                .splineTo(new Vector2d(toInches(-81), toInches(-39)), Math.toRadians(135))
                                .splineTo(new Vector2d(toInches(-140), toInches(-30)), Math.toRadians(180))
                                .splineTo(new Vector2d(toInches(-81), toInches(-39)), Math.toRadians(135))
                                .splineTo(new Vector2d(toInches(-140), toInches(-30)), Math.toRadians(180))
                                .splineTo(new Vector2d(toInches(-81), toInches(-39)), Math.toRadians(135))
                                .splineTo(new Vector2d(toInches(-150), toInches(-30)), Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }



    public static double toInches(double dist){
        return dist/2.54;
    }
}