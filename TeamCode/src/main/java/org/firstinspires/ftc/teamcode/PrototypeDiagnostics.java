package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "diagnostics")
public class PrototypeDiagnostics extends OpMode {
    private PrototypeHardware hardware;
    @Override
    public void init() {
        hardware = new PrototypeHardware(hardwareMap, true);
    }

    @Override
    public void loop() {
        hardware.leftDrive.setPower(gamepad1.dpad_left ? 1 : 0);
        hardware.rightDrive.setPower(gamepad1.dpad_right ? 1 : 0);
        hardware.flywheel.setPower(gamepad1.y ? 1 : 0);
        hardware.intake.setPower(gamepad1.x ? 1 : 0);

        if (gamepad1.b) {
            hardware.pusherForward();
        } else {
            hardware.pusherBack();
        }

        if (gamepad1.dpad_up) {
            hardware.armUp();
        } else if (gamepad1.dpad_down) {
            hardware.armDown();
        } else {
            hardware.armStop();
        }

        if (gamepad1.a) {
            hardware.closeClaw();
        } else {
            hardware.openClaw();
        }
    }
}
