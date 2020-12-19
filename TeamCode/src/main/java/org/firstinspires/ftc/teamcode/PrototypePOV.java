package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(group = "diagnostics")
public class PrototypePOV extends OpMode {
    private PrototypeHardware hardware;

    @Override
    public void init() {
        hardware = new PrototypeHardware(hardwareMap, true);
    }

    boolean dPadWasPressed = false;
    boolean flywheelOn = false;

    @Override
    public void loop() {
        double forwardPower = Math.pow(gamepad1.left_stick_y, 2) * (gamepad1.left_stick_y > 0 ? -1 : 1);
        double turnPower = Math.pow(gamepad1.left_stick_x, 2) * (gamepad1.left_stick_x < 0 ? -1 : 1);

        telemetry.addLine(String.format("%.2f -> %.2f\t%.2f -> %.2f", gamepad1.left_stick_y, forwardPower, gamepad1.left_stick_x, turnPower));

        hardware.leftDrive.setPower(Range.clip(forwardPower + turnPower, -1, 1));
        hardware.rightDrive.setPower(Range.clip(forwardPower - turnPower, -1, 1));

        hardware.intake.setPower(gamepad1.y ? 1 : 0);

        if (dPadWasPressed) {
            dPadWasPressed = !gamepad1.dpad_up;
        } else {
            if (gamepad1.dpad_up) {
                dPadWasPressed = true;
                flywheelOn = !flywheelOn;
            }
        }
        double targetVelociy = flywheelOn ? 1500 : 0;
        telemetry.addLine(String.format("%.2f:%.2f", hardware.flywheel.getVelocity(), targetVelociy));
        hardware.flywheel.setVelocity(targetVelociy);


        hardware.pusher.setPosition(gamepad1.b ? hardware.PUSHER_FORWARD : hardware.PUSHER_BACK);
    }
}
