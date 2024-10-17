package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double north = Math.toRadians(90);
        double south = Math.toRadians(-90);
        double east = Math.toRadians(0);
        double west = Math.toRadians(180);
        Pose2d startingPose = new Pose2d(-12, 60, north);
        double scoringY = 30;
        Vector2d intakeLocation = new Vector2d(-42, 62);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(60), Math.toRadians(60), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingPose)
                        .lineTo(new Vector2d(0, scoringY))
                        .setTangent(north)
                        .splineToConstantHeading(intakeLocation, west)
                        .setTangent(east)
                        .splineToConstantHeading(new Vector2d(-2, scoringY), south)
                        .setTangent(north)
                        .splineToLinearHeading(new Pose2d(-38, 24, west), south)
                        .setTangent(south)
                        .splineToConstantHeading(new Vector2d(-48, 14), north)
                        .splineToConstantHeading(new Vector2d(-48, 60), north)
                        .setTangent(south)
                        .splineToConstantHeading(new Vector2d(-50, 14), west)
                        .splineToConstantHeading(new Vector2d(-55, 60), north)
                        .splineToLinearHeading(
                                new Pose2d(intakeLocation.getX(), intakeLocation.getY(), north)
                                , north)
                        .setTangent(east)
                        .splineToConstantHeading(new Vector2d(-4, scoringY), south)
                        .setTangent(north)
                        .splineToConstantHeading(intakeLocation, west)
                        .setTangent(east)
                        .splineToConstantHeading(new Vector2d(-6, scoringY), south)
                        .setTangent(north)
                        .splineToConstantHeading(intakeLocation, west)
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}