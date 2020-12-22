package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

public class PrototypeHardware {
    public DcMotorEx leftDrive, rightDrive, flywheel, intake;
    public Servo pusher, armOne, armTwo, claw;

    public int PUSHER_DELAY = 600;
    public int FLYWHEEL_LAUNCH_LINE_VELOCITY = 1500;
    public int FLYWHEEL_MAX_VELOCITY = 1700;

    public PrototypeHardware(HardwareMap hardwareMap, boolean drive) {

        if (drive) {
            rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
            rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftDrive = hardwareMap.get(DcMotorEx.class, "left_drive");
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pusher = hardwareMap.get(Servo.class, "pusher");
        pusherBack();

        claw = hardwareMap.get(Servo.class, "claw");
        openClaw();

        armOne = hardwareMap.get(Servo.class, "arm_one");
        armTwo = hardwareMap.get(Servo.class, "arm_two");
        armStop();
    }

    private double PUSHER_BACK = .65;
    private double PUSHER_FORWARD = 1;

    public void pusherBack() {
        pusher.setPosition(PUSHER_BACK);
    }

    public void pusherForward() {
        pusher.setPosition(PUSHER_FORWARD);
    }

    private double CLAW_OPEN = 0;
    private double CLAW_CLOSED = 1;

    public void openClaw() {
        claw.setPosition(CLAW_OPEN);
    }

    public void closeClaw() {
        claw.setPosition(CLAW_CLOSED);
    }

    private double ARM_STOP = .5;
    private double ARM_UP = 0;
    private double ARM_DOWN = 1;

    public void armStop() {
        armOne.setPosition(ARM_STOP);
        armTwo.setPosition(ARM_STOP);
    }

    public void armUp() {
        armOne.setPosition(ARM_UP);
        armTwo.setPosition(ARM_UP);
    }

    public void armDown() {
        armOne.setPosition(ARM_DOWN);
        armTwo.setPosition(ARM_DOWN);

    }

}
