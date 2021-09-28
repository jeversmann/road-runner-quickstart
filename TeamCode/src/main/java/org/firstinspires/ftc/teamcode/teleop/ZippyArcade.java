package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.SquareCurve;
import org.firstinspires.ftc.teamcode.util.ZippyArm;

@TeleOp
public class ZippyArcade extends OpMode {
    private ZippyArm arm;
    private SampleMecanumDrive drive;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        arm = new ZippyArm(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -SquareCurve.from(gamepad1.left_stick_y),
                        -SquareCurve.from(gamepad1.left_stick_x),
                        -SquareCurve.from(gamepad1.right_stick_x)
                )
        );

        drive.update();

        if (gamepad1.left_trigger > .5) {
            arm.down();
        } else if (gamepad1.dpad_up) {
            arm.top();
        } else if (gamepad1.dpad_right) {
            arm.middle();
        } else if (gamepad1.dpad_down) {
            arm.low();
        } else if (gamepad1.left_bumper) {
            arm.flip();
        }
    }

}
