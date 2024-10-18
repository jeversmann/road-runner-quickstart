package org.firstinspires.ftc.teamcode.zippy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class BlueSpecimenAuto extends LinearOpMode {

    private static final double north = Math.toRadians(90);
    private static final double south = Math.toRadians(-90);
    private static final double east = Math.toRadians(0);
    private static final double west = Math.toRadians(180);
    private static final double scoringY = 30;

    public static Pose2d robotInBlueObservationCorner = new Pose2d(-72.0 + 7.25, 72.0 - 8.75, east);
    public static Pose2d intakeFirstBlueLine = new Pose2d(-57, 45, north);
    public static Pose2d depositFirstBlueLine = new Pose2d(-54, 42, -2);
    public static Pose2d blueSpecimenIntake = new Pose2d(-45, 62, north);
    public static Pose2d blueLowRung = new Pose2d(2, 30, 2);
    public static Pose2d blueSpecimenAutoStart = new Pose2d(-13, 62.5, west);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        StatusLights lights = new StatusLights(hardwareMap);

        arm.close();
        lights.green();
        drive.setPoseEstimate(blueSpecimenAutoStart);
        drive.update();

        waitForStart();

        if (isStopRequested()) return;

        lights.off();

        TrajectorySequence seq = drive.trajectorySequenceBuilder(blueSpecimenAutoStart)
                .setTangent(south)
                .splineToLinearHeading(new Pose2d(-10, scoringY, north), south)
                .setTangent(north)
                .splineToLinearHeading(blueSpecimenIntake, west)
                .setTangent(east)
                .splineToConstantHeading(new Vector2d(-8, scoringY), south)
                .setTangent(north)
                .splineToLinearHeading(intakeFirstBlueLine, west)
                .lineToLinearHeading(depositFirstBlueLine)
                .waitSeconds(3)
                .setTangent(east)
                .splineToLinearHeading(blueSpecimenIntake, north)
                .setTangent(east)
                .splineToConstantHeading(new Vector2d(-4, scoringY), south)
                .setTangent(north)
                .lineToLinearHeading(blueLowRung)
                .waitSeconds(10)
                // Raise arm for preload
                .addTemporalMarker(0, () -> arm.setPose(Arm.Pose.HighChamber))
                // Score preload
                .addTemporalMarker(2, () -> {
                    arm.outtake();
                    lights.red();
                })
                .addTemporalMarker(2.5, () -> {
                    arm.open();
                    lights.green();
                })
                .addTemporalMarker(4, () -> {
                    arm.setPose(Arm.Pose.WallIntake);
                    lights.off();
                })
                // Intake 1
                .addTemporalMarker(5, () -> {
                    arm.close();
                    lights.red();
                })
                .addTemporalMarker(5.2, () -> {
                    arm.outtake();
                    lights.green();
                })
                .addTemporalMarker(5.9, () -> {
                    arm.setPose(Arm.Pose.HighChamber);
                    lights.off();
                })
                // Score 1
                .addTemporalMarker(7.7, () -> {
                    arm.outtake();
                    lights.red();
                })
                .addTemporalMarker(8.2, () -> {
                    arm.open();
                    lights.green();
                })
                // Ground intake 2
                .addTemporalMarker(9.5, () -> {
                    arm.setPose(Arm.Pose.Intake);
                    lights.off();
                })
                .addTemporalMarker(10.5, () -> {
                    arm.close();
                    lights.orange();
                })
                .addTemporalMarker(11, () -> {
                    arm.setPose(Arm.Pose.Submersible);
                    lights.orange();
                })
                .addTemporalMarker(12.5, () -> {
                    arm.open();
                    arm.setPose(Arm.Pose.WallIntake);
                    lights.off();
                })
                // Wall intake 2
                .addTemporalMarker(18, () -> {
                    arm.close();
                    lights.red();
                })
                .addTemporalMarker(18.5, () -> {
                    arm.outtake();
                    lights.green();
                })
                .addTemporalMarker(19, () -> {
                    arm.setPose(Arm.Pose.HighChamber);
                    lights.off();
                })
                // Score 2
                .addTemporalMarker(20.5, () -> {
                    arm.outtake();
                    lights.red();
                })
                .addTemporalMarker(20.9, () -> {
                    arm.open();
                    lights.green();
                })
                .addTemporalMarker(22, () -> {
                    arm.outtake();
                    lights.orange();
                })
                .build();

        drive.followTrajectorySequence(seq);
    }
}
