package org.firstinspires.ftc.teamcode.zippy;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "DriveV2")
public class DriveV2 extends OpMode {
    private Arm arm;
    private MecanumDrive drive;
    private boolean clawClosed;
    private boolean clawDebounce;
    private boolean outtakeDebounce;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(180)));
        arm = new Arm(hardwareMap);

        clawDebounce = false;
        clawClosed = false;
        outtakeDebounce = false;
        arm.open();
    }

    @Override
    public void loop() {
        double heading = drive.pose.heading.toDouble();
        Vector2d controllerVector = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
        double fieldX = controllerVector.dot(new Vector2d(Math.cos(heading), Math.sin(heading)));
        double fieldY = controllerVector.dot(new Vector2d(-Math.sin(heading), Math.cos(heading)));
        Vector2d fieldVector = new Vector2d(fieldX, fieldY);

        drive.setDrivePowers(new PoseVelocity2d(fieldVector, -gamepad1.right_stick_x));

        drive.updatePoseEstimate();

        if (gamepad1.right_bumper) {
            if (!clawDebounce) {
                if (clawClosed) {
                    arm.open();
                } else {
                    arm.close();
                }
                clawClosed = !clawClosed;
            }
            clawDebounce = true;
        } else {
            clawDebounce = false;
        }

        if (gamepad1.a) {
            arm.setPose(Arm.Pose.Zero);
        } else if (gamepad1.b && !outtakeDebounce) {
            arm.outtake();
            outtakeDebounce = true;
        } else if (gamepad1.x) {
            arm.setPose(Arm.Pose.LowChamber);
        } else if (gamepad1.y) {
            arm.setPose(Arm.Pose.HighChamber);
        } else if (gamepad1.dpad_up) {
            arm.setPose(Arm.Pose.Submersible);
        } else if (gamepad1.dpad_down) {
            arm.setPose(Arm.Pose.Intake);
        } else if (gamepad1.dpad_left) {
            arm.zeroEncoders();
        } else {
            outtakeDebounce = false;
        }

        arm.adjustPosition(gamepad1.right_stick_y);

        telemetry.addData("Shoulder Position", arm.getShoulderTicks());

        telemetry.update();
    }
}
