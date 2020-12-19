package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

public class PrototypeHardware {
    public DcMotorEx leftDrive, rightDrive, flywheel, intake;
    public Servo pusher;

    public double PUSHER_BACK = .65;
    public double PUSHER_FORWARD = 1;

    public int PUSHER_DELAY = 600;
    public int FLYWHEEL_LAUNCH_LINE_VELOCITY = 1450;
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
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pusher = hardwareMap.get(Servo.class, "pusher");
        pusher.setPosition(PUSHER_BACK);
    }
}
