package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PrototypeHardware;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@Autonomous(group = "drive")
public class UltimateGoalAutoCloseRings extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        PrototypeHardware hardware = new PrototypeHardware(hardwareMap, false);

        waitForStart();

        if (isStopRequested()) return;

        // This is the pose of the robot centered on the right starting line against the wall
        Pose2d startPose = new Pose2d(-60, -48, 0);
        drive.setPoseEstimate(startPose);


        hardware.flywheel.setVelocity(hardware.FLYWHEEL_LAUNCH_LINE_VELOCITY);
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(0, -54), Math.toRadians(0))
                .build());

        int rings = 3;
        while (rings > 0) {
            if (hardware.flywheel.getVelocity() < hardware.FLYWHEEL_LAUNCH_LINE_VELOCITY) {
                continue;
            }

            hardware.pusher.setPosition(hardware.PUSHER_FORWARD);
            sleep(hardware.PUSHER_DELAY);
            hardware.pusher.setPosition(hardware.PUSHER_BACK);
            sleep(hardware.PUSHER_DELAY);
            rings--;
        }
        hardware.flywheel.setVelocity(0);

        drive.turn(Math.toRadians(-25));

        hardware.intake.setPower(1);

        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(40)
                .build());

        hardware.flywheel.setVelocity(hardware.FLYWHEEL_LAUNCH_LINE_VELOCITY);

        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(0, -54), Math.toRadians(5))
                .build());

        hardware.intake.setPower(0);

        rings = 3;
        while (rings > 0) {
            if (hardware.flywheel.getVelocity() < hardware.FLYWHEEL_LAUNCH_LINE_VELOCITY) {
                continue;
            }

            hardware.pusher.setPosition(hardware.PUSHER_FORWARD);
            sleep(hardware.PUSHER_DELAY);
            hardware.pusher.setPosition(hardware.PUSHER_BACK);
            sleep(hardware.PUSHER_DELAY);
            rings--;
        }
        hardware.flywheel.setVelocity(0);

        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(10)
                .build());
    }
}
