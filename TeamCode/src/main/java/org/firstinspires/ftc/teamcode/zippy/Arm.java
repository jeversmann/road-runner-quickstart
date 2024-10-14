package org.firstinspires.ftc.teamcode.zippy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

public class Arm {
    public enum Pose {
        Zero,
        Submersible,
        LowChamber,
        LowRung,
        HighChamber,
        LowBasket,
        HighBasket,
        WallIntake,
        Intake
    }

    private int shoulderTicks(Pose pose) {
        switch (pose) {
            case LowChamber:
                return -1850;
            case LowRung:
                return 0;
            case HighChamber:
                return -1250;
            case LowBasket:
                return 0;
            case HighBasket:
                return 0;
            case WallIntake:
                return 0;
            case Intake:
                return -2300;
            case Submersible:
                return -2100;
            case Zero:
            default:
                return 0;
        }
    }

    private static int UP = -1000;

    private DcMotorEx shoulder;
    private Servo wrist;
    private Servo claw;

    private Pose currentPose;

    public Arm(HardwareMap hardwareMap) {
        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        wristUp();
        close();

        shoulder.setDirection(DcMotorSimple.Direction.REVERSE);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setPose(Pose.Zero);
    }

    public void zeroEncoders() {
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        currentPose = Pose.Zero;
        shoulder.setTargetPosition(0);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getShoulderTicks() {
        return shoulder.getCurrentPosition();
    }

    public void setPose(Pose pose) {
        int shoulderTicks = shoulderTicks(pose);

        shoulder.setPower(0);

        shoulder.setTargetPosition(shoulderTicks);

        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shoulder.setPower(.8);

        currentPose = pose;

        if (pose == Pose.HighChamber || pose == Pose.LowChamber) {
            wristDown();
        } else {
            wristUp();
        }
    }

    public void adjustPosition(double direction) {
        int target = shoulder.getTargetPosition();
        int current = shoulder.getCurrentPosition();
        if (current < UP) {
            direction *= -1;
        }
        int adjustment = (int) (direction * 10);
        shoulder.setTargetPosition(target + adjustment);
    }

    public void open() {
        claw.setPosition(0);
    }

    public void close() {
        claw.setPosition(1);
    }

    public void outtake() {
        shoulder.setTargetPosition(shoulder.getTargetPosition() - 100);
    }

    public void wristUp() {
        wrist.setPosition(1);
    }

    public void wristDown() {
        wrist.setPosition(0);
    }
}
