package org.firstinspires.ftc.teamcode.zippy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "DriveV3")
public class DriveV3 extends OpMode {
    private Arm arm;
    private SampleMecanumDrive drive;
    private StatusLights lights;
    private boolean clawClosed;
    private boolean clawDebounce;
    private boolean outtakeDebounce;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        lights = new StatusLights(hardwareMap);

        clawDebounce = false;
        clawClosed = false;
        outtakeDebounce = false;
        arm.open();
        drive.update();
    }

    @Override
    public void init_loop(){
        drive.update();
    }

    @Override
    public void loop() {
        lights.off(0);
        lights.green(1);
        lights.red(2);
        lights.orange(3);

        double heading = drive.getPoseEstimate().getHeading();
        double fieldX = -gamepad1.left_stick_y * Math.cos(heading) + -gamepad1.left_stick_x * Math.sin(heading);
        double fieldY = -gamepad1.left_stick_y * -Math.sin(heading) + -gamepad1.left_stick_x * Math.cos(heading);

        drive.setWeightedDrivePower(
                new Pose2d(
                        fieldX,
                        fieldY,
                        -gamepad1.right_stick_x
                )
        );

        drive.update();

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());

        if (gamepad1.right_bumper) {
            if (!clawDebounce) {
                if (clawClosed) {
                    arm.open();
                } else {
                    arm.close();
                }
                clawClosed = !clawClosed;
            }
            clawDebounce = true;
        } else {
            clawDebounce = false;
        }

        if (gamepad1.a) {
            arm.setPose(Arm.Pose.Zero);
        } else if (gamepad1.b && !outtakeDebounce) {
            arm.outtake();
            outtakeDebounce = true;
        } else if (gamepad1.x) {
            arm.setPose(Arm.Pose.LowChamber);
        } else if (gamepad1.y) {
            arm.setPose(Arm.Pose.HighChamber);
        } else if (gamepad1.dpad_up) {
            arm.setPose(Arm.Pose.Submersible);
        } else if (gamepad1.dpad_down) {
            arm.setPose(Arm.Pose.Intake);
        } else if (gamepad1.dpad_left) {
            arm.zeroEncoders();
        } else {
            outtakeDebounce = false;
        }

        arm.adjustPosition(gamepad1.right_stick_y);

        telemetry.addData("Shoulder Position", arm.getShoulderTicks());

        telemetry.update();
    }
}
