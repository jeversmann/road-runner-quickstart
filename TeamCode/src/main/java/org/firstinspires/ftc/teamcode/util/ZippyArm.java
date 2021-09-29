package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.nio.channels.FileLockInterruptionException;

public class ZippyArm {
    private DcMotorEx arm, carousel, claw;

    private static final int LOWER_BOUND = 0;
    private static final int UPPER_BOUND = 3500;

    private int armTarget;

    public ZippyArm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        claw = hardwareMap.get(DcMotorEx.class, "claw");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        claw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getArmTarget() {
        return armTarget;
    }

    public int getArmTicks() {
        return arm.getCurrentPosition();
    }

    private int applyBounds(int target) {
        if (target < LOWER_BOUND) {
            return LOWER_BOUND;
        } else if (target > UPPER_BOUND) {
            return UPPER_BOUND;
        } else {
            return target;
        }
    }

    public void setTarget(int pos) {
        armTarget = applyBounds(pos);

        arm.setTargetPosition(armTarget);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
    }

    public void adjustTarget(int ticks) {
        armTarget = applyBounds(armTarget + ticks);

        arm.setTargetPosition(armTarget);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(.8);
    }

    public void clawClose() {
        claw.setPower(1);
    }

    public void clawOpen() {
        claw.setPower(-.5);
    }

    public void clawStop() {
        claw.setPower(0);
    }

    public void setCarousel(double power) {
        carousel.setPower(power);
    }
}
