package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.nio.channels.FileLockInterruptionException;

public class ZippyArm {
    public static final int ARM_FLIPPED = 1100;
    public static final int ARM_TOP = 500;
    public static final int ARM_MIDDLE = 350;
    public static final int ARM_LOW = 200;
    public static final int ARM_DOWN = 0;

    private DcMotorEx arm, carousel;

    public ZippyArm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void set(int pos) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(.5);
    }

    public void down() {
        set(ARM_DOWN);
    }

    public void low() {
        set(ARM_LOW);
    }

    public void middle() {
        set(ARM_MIDDLE);
    }

    public void top() {
        set(ARM_TOP);
    }

    public void flip() {
        set(ARM_FLIPPED);
    }
}
