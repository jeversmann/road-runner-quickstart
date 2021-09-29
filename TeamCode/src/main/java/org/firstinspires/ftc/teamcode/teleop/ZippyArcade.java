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
        // DRIVETRAIN
        drive.setWeightedDrivePower(
                new Pose2d(
                        -SquareCurve.from(gamepad1.left_stick_y),
                        -SquareCurve.from(gamepad1.left_stick_x),
                        -SquareCurve.from(gamepad1.right_stick_x)
                )
        );
        drive.update();

        // ARM
        int stick = (int) Math.round(-SquareCurve.from(gamepad1.right_stick_y) * 50);
        if (gamepad1.left_trigger > .5) {
            arm.adjustTarget(-100);
        } else if (gamepad1.left_bumper) {
            arm.setTarget(650);
        } else {
            arm.adjustTarget(stick);
        }
        telemetry.addLine("Arm T:" + arm.getArmTarget() + " P:" + arm.getArmTicks());

        // CLAW
        if (gamepad1.right_bumper) {
            arm.clawOpen();
        } else if (gamepad1.right_trigger > .5) {
            arm.clawClose();
        } else {
            arm.clawStop();
        }

        // CAROUSEL
        if (gamepad1.x) {
            arm.setCarousel(1);
        } else if (gamepad1.b) {
            arm.setCarousel(-1);
        } else {
            arm.setCarousel(0);
        }
    }
}
