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
            if (gamepad1.left_trigger > .5) {
                arm.manualControl((double) gamepad1.left_stick_x, (double) gamepad1.left_stick_y, null, null);
            } else if (gamepad1.a) {
                arm.setPose(Arm.Pose.Zero);
            } else if (gamepad1.b) {
                arm.open();
            } else if (gamepad1.x) {
                arm.close();
            } else if (gamepad1.y) {
                arm.setPose(Arm.Pose.LowBasket);
            } else if (gamepad1.dpad_up) {
                arm.setPose(Arm.Pose.HighChamber);
                arm.hold();
            } else if (gamepad1.dpad_down) {
                arm.setPose(Arm.Pose.WallIntake);
            }

            telemetry.addData("Elbow Position", arm.getElbowTicks());
            telemetry.addData("Shoulder Position", arm.getShoulderTicks());

            telemetry.update();
        }
    }
}
