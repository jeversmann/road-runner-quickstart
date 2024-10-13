package org.firstinspires.ftc.teamcode.zippy;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ArmTest", group = "Sensor")
public class ArmTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);

        // Wait for the start button to be pressed
        waitForStart();
        arm.zeroEncoders();

        // Loop until the OpMode ends
        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.5) {
                arm.open();
            } else {
                arm.close();
            }

            if (gamepad1.left_trigger > 0.5) {
                arm.wristDown();
            } else {
                arm.wristUp();
            }

            if (gamepad1.a) {
                arm.setPose(Arm.Pose.Zero);
            } else if (gamepad1.b) {
                arm.setPose(Arm.Pose.Submersible);
            } else if (gamepad1.x) {
                arm.setPose(Arm.Pose.Intake);
            } else if (gamepad1.y) {
                arm.setPose(Arm.Pose.WallIntake);
            } else if (gamepad1.dpad_down) {
                arm.setPose(Arm.Pose.LowBasket);
            } else if (gamepad1.dpad_up) {
                arm.setPose(Arm.Pose.HighBasket);
            } else if (gamepad1.dpad_left) {
                arm.setPose(Arm.Pose.LowChamber);
            } else if (gamepad1.dpad_right) {
                arm.setPose(Arm.Pose.HighChamber);
            }

            arm.adjustPosition(gamepad1.right_stick_y);

            telemetry.addData("Shoulder Position", arm.getShoulderTicks());

            telemetry.update();
        }
    }
}
