package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "drive")
public class ZippyDiagnostics extends OpMode {

    private DcMotorEx arm, carousel, claw;

    private DcMotorEx back_left_drive;
    private DcMotorEx back_right_drive;
    private DcMotorEx front_left_drive;
    private DcMotorEx front_right_drive;

    @Override
    public void init() {
        back_left_drive = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        back_right_drive = hardwareMap.get(DcMotorEx.class, "back_right_drive");
        front_left_drive = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        front_right_drive = hardwareMap.get(DcMotorEx.class, "front_right_drive");

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        claw = hardwareMap.get(DcMotorEx.class, "claw");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        arm.setPower(gamepad1.y ? .5 : 0);
        carousel.setPower(gamepad1.b ? .5 : 0);
        claw.setPower(gamepad1.x ? .5 : 0);

        setDriveMotorPowers(
                gamepad1.dpad_up ? .5 : 0,
                gamepad1.dpad_right ? .5 : 0,
                gamepad1.dpad_left ? .5 : 0,
                gamepad1.dpad_down ? .5 : 0);
    }

    private void setDriveMotorPowers(double fR, double bR, double fL, double bL) {
        front_left_drive.setPower(fR);
        front_left_drive.setPower(fL);
        back_right_drive.setPower(bR);
        back_left_drive.setPower(bL);
    }
}
