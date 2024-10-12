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
                return -1300;
            case LowRung:
                return -2100;
            case HighChamber:
                return -2250;
            case LowBasket:
                return -1400;
            case HighBasket:
                return -3000;
            case WallIntake:
                return -1070;
            case Intake:
            // Submersible is the same as intake so we don't have to move up through the barrier
            case Submersible:
                return -550;
            case Zero:
            default:
                return 0;
        }
    }

    private int elbowTicks(Pose pose) {
        switch (pose ){
            case Submersible:
                return 300;
            case LowBasket:
                return 180;
            case HighBasket:
                return 500;
            case Intake:
                return 450;
            case WallIntake:
            case LowChamber:
            case LowRung:
            case HighChamber:
            case Zero:
            default:
                return 0;
        }
    }

    private DcMotorEx shoulder;
    private DcMotorEx elbow;
    private Servo intake;
    private Servo claw;

    private Pose currentPose;

    public Arm(HardwareMap hardwareMap) {
        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        intake = hardwareMap.get(Servo.class, "intake");
        claw = hardwareMap.get(Servo.class, "claw");

        shoulder.setDirection(DcMotorSimple.Direction.REVERSE);

        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void zeroEncoders() {
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        currentPose = Pose.Zero;
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void manualControl(Double shoulderPower, Double elbowPower, Double intakePower, Double clawPosition) {
        if(shoulderPower != null) {
            shoulder.setPower(shoulderPower);
        }
        if(elbowPower != null) {
            elbow.setPower(elbowPower);
        }
        if(intakePower != null) {
            intake.setPosition(intakePower);
        }
        if (clawPosition != null) {
            claw.setPosition(clawPosition);
        }
    }

    public int getShoulderTicks() {
        return shoulder.getCurrentPosition();
    }

    public int getElbowTicks() {
        return elbow.getCurrentPosition();
    }

    public void setPose(Pose pose) {
        int shoulderTicks = shoulderTicks(pose);
        int elbowTicks = elbowTicks(pose);

        shoulder.setPower(0);
        elbow.setPower(0);

        shoulder.setTargetPosition(shoulderTicks);
        elbow.setTargetPosition(elbowTicks);

        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shoulder.setPower(.8);
        elbow.setPower(.8);

        currentPose = pose;
    }

    private Pose[] ADJUST_SHOULDER = new Pose[]{Pose.Zero, Pose.WallIntake, Pose.HighChamber, Pose.LowChamber, Pose.LowRung};
    private Pose[] ADJUST_ELBOW = new Pose[]{Pose.Intake, Pose.Submersible, Pose.HighBasket, Pose.LowBasket};

    public void adjustPosition(double direction) {
        if (Arrays.stream(ADJUST_SHOULDER).anyMatch(pose -> pose == currentPose)) {
            shoulder.setTargetPosition(shoulder.getTargetPosition() + (int) (direction * 10));
        } else if (Arrays.stream(ADJUST_ELBOW).anyMatch(pose -> pose == currentPose)) {
            elbow.setTargetPosition(elbow.getTargetPosition() + (int) (direction * 3));
        }
    }

    public void intake() {
        intake.setPosition(1);
    }

    public void hold() {
        intake.setPosition(.5);
    }

    public void outtake() {
        intake.setPosition(-1);
    }

    public void open() {
        claw.setPosition(1);
    }

    public void close() {
        claw.setPosition(.5);
    }
}
