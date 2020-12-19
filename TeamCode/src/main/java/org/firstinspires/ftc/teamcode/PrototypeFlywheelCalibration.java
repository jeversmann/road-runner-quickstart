package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "diagnostics")
public class PrototypeFlywheelCalibration extends OpMode {

    /*
    Launch Line: 1500
    Full court shot: 1640
    Safety limit: 1700
     */

    private PrototypeHardware hardware;

    @Override
    public void init() {
        hardware = new PrototypeHardware(hardwareMap, false);
    }


    private double flywheelPower = 0;
    private boolean dPad = false;

    @Override
    public void loop() {
        if (gamepad1.b) {
            hardware.pusher.setPosition(hardware.PUSHER_FORWARD);
        } else {
            hardware.pusher.setPosition(hardware.PUSHER_BACK);
        }

        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            if (!dPad) {
                dPad = true;
                if (gamepad1.dpad_up) {
                    flywheelPower += 100;
                } else if (gamepad1.dpad_down) {
                    flywheelPower -= 100;
                } else if (gamepad1.dpad_right) {
                    flywheelPower += 10;
                } else if (gamepad1.dpad_left) {
                    flywheelPower -= 10;
                }
            }
        } else {
            dPad = false;
        }

        hardware.flywheel.setVelocity(flywheelPower);
        telemetry.addLine(String.format("Target: %.2f Actual: %.2f", flywheelPower, hardware.flywheel.getVelocity()));
    }
}
