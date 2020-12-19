package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@TeleOp
public class PrototypeAutoAim extends OpMode {
    private SampleTankDrive drive;
    private PrototypeHardware hardware;

    @Override
    public void init() {
        drive = new SampleTankDrive(hardwareMap);
        hardware = new PrototypeHardware(hardwareMap, false);
    }

    @Override
    public void start() {
        // This is the pose of the robot centered on the right starting line against the wall
        Pose2d startPose = new Pose2d(-60, -48, 0);
        drive.setPoseEstimate(startPose);
    }

    boolean autoAiming = false;
    boolean pusherMoving = false;
    int rings = 0;

    @Override
    public void loop() {
        if (gamepad1.dpad_down) {
            autoAiming = true;
            rings = 3;
            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineTo(new Vector2d(0, -54), Math.toRadians(0))
                    .build());
        }

        if (autoAiming) {
            hardware.flywheel.setVelocity(hardware.FLYWHEEL_LAUNCH_LINE_VELOCITY);

            if (!drive.isBusy() && hardware.flywheel.getVelocity() < hardware.FLYWHEEL_LAUNCH_LINE_VELOCITY) {
                if (rings > 0) {
                    hardware.pusher.setPosition(hardware.PUSHER_FORWARD);
                    sleep(hardware.PUSHER_DELAY);
                    hardware.pusher.setPosition(hardware.PUSHER_BACK);
                    sleep(hardware.PUSHER_DELAY);
                    rings--;
                } else {
                    hardware.flywheel.setVelocity(0);
                    autoAiming = false;
                }
            }
        } else {
            Pose2d control = new Pose2d(
                    signSquare(gamepad1.left_stick_y) / 2,
                    0,
                    -signSquare(gamepad1.left_stick_x) / 2
            );
            drive.setWeightedDrivePower(control);
            telemetry.addLine(String.format("%.2f : %.2f", control.component1(), control.component3()));
        }

        drive.update();

        hardware.intake.setPower(gamepad1.y ? 1 : 0);
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private double signSquare(double d) {
        return Math.pow(d, 2) * (d < 0 ? -1 : 1);
    }
}
