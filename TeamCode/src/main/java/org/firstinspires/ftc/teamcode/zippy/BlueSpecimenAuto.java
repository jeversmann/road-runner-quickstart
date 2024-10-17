package org.firstinspires.ftc.teamcode.zippy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class BlueSpecimenAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        double angle = Math.toRadians(90);
        double tangent = Math.toRadians(-90);

        int clawTime = 500;
        int outtakeTime = 200;

        Pose2d startingPose = new Pose2d(-12, 60, angle);
        double scoringY = 30;
        Vector2d intakeLocation = new Vector2d(-39, 62);

        arm.close();
        drive.setPoseEstimate(startingPose);
        drive.update();

        waitForStart();

        if (isStopRequested()) return;

        Trajectory scorePreload = drive.trajectoryBuilder(startingPose)
                .lineTo(new Vector2d(0, scoringY))
                .addTemporalMarker(0, () -> { arm.setPose(Arm.Pose.HighChamber);})
                .build();
        Trajectory intakeOne = drive.trajectoryBuilder(scorePreload.end())
                .splineToConstantHeading(intakeLocation, angle)
                .addTemporalMarker(1, () -> { arm.setPose(Arm.Pose.WallIntake);})
                .build();
        Trajectory scoreOne = drive.trajectoryBuilder(intakeOne.end(), true)
                .splineToConstantHeading(new Vector2d(-2, scoringY), tangent)
                .addTemporalMarker(0, () -> { arm.setPose(Arm.Pose.HighChamber);})
                .build();
        Trajectory intakeTwo = drive.trajectoryBuilder(scoreOne.end())
                .splineToConstantHeading(intakeLocation, angle)
                .addTemporalMarker(1, () -> { arm.setPose(Arm.Pose.WallIntake);})
                .build();
        Trajectory scoreTwo = drive.trajectoryBuilder(intakeTwo.end(), true)
                .splineToConstantHeading(new Vector2d(-4, scoringY), tangent)
                .addTemporalMarker(0, () -> { arm.setPose(Arm.Pose.HighChamber);})
                .build();

        drive.followTrajectory(scorePreload);
        arm.outtake();
        sleep(outtakeTime);
        arm.open();
        sleep(clawTime);

        drive.followTrajectory(intakeOne);
        arm.close();
        sleep(clawTime);
        arm.outtake();
        sleep(outtakeTime);

        drive.followTrajectory(scoreOne);
        arm.outtake();
        sleep(outtakeTime);
        arm.open();
        sleep(clawTime);

        drive.followTrajectory(intakeTwo);
        arm.close();
        sleep(clawTime);
        arm.outtake();
        sleep(outtakeTime);

        drive.followTrajectory(scoreTwo);
        arm.outtake();
        sleep(200);
        arm.open();
        sleep(clawTime);
    }
}
