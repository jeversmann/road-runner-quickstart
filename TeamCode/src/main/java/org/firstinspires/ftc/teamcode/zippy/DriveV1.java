package org.firstinspires.ftc.teamcode.zippy;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "DriveV1")
public class DriveV1 extends OpMode {
    private Arm arm;
    private MecanumDrive drive;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        arm = new Arm(hardwareMap);
    }

    @Override
    public void loop() {
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));

        drive.updatePoseEstimate();

        if (gamepad1.right_trigger > 0.5) {
            arm.intake();
        } else if (gamepad1.left_trigger > 0.5) {
            arm.outtake();
        } else {
            arm.hold();
        }

        if (gamepad1.right_bumper) {
            arm.close();
        } else if (gamepad1.left_bumper) {
            arm.open();
        }

        if (gamepad1.a) {
            arm.setPose(Arm.Pose.Zero);
        } else if (gamepad1.b) {
            arm.setPose(Arm.Pose.Submersible);
        } else if (gamepad1.x) {
            arm.setPose(Arm.Pose.Intake);
        } else if (gamepad1.y) {
            arm.setPose(Arm.Pose.WallIntake);
        } else if (gamepad1.dpad_down) {
            arm.setPose(Arm.Pose.LowBasket);
        } else if (gamepad1.dpad_up) {
            arm.setPose(Arm.Pose.HighBasket);
        } else if (gamepad1.dpad_left) {
            arm.setPose(Arm.Pose.LowChamber);
        } else if (gamepad1.dpad_right) {
            arm.setPose(Arm.Pose.HighChamber);
        }
    }
}
