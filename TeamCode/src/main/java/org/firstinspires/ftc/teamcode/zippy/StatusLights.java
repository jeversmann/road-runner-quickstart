package org.firstinspires.ftc.teamcode.zippy;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

import java.util.ArrayList;
import java.util.List;

public class StatusLights {
    private List<LED> greenLEDs, redLEDs;

    public StatusLights(HardwareMap hardwareMap) {
        greenLEDs = new ArrayList<>();
        greenLEDs.add(hardwareMap.get(LED.class, "g0"));
        greenLEDs.add(hardwareMap.get(LED.class, "g1"));
        greenLEDs.add(hardwareMap.get(LED.class, "g2"));
        greenLEDs.add(hardwareMap.get(LED.class, "g3"));

        redLEDs = new ArrayList<>();
        redLEDs.add(hardwareMap.get(LED.class, "r0"));
        redLEDs.add(hardwareMap.get(LED.class, "r1"));
        redLEDs.add(hardwareMap.get(LED.class, "r2"));
        redLEDs.add(hardwareMap.get(LED.class, "r3"));

        off(0);
        off(1);
        off(2);
        off(3);
    }

    public void red(int pos) {
        greenLEDs.get(pos).off();
        redLEDs.get(pos).on();
    }

    public void green(int pos) {
        greenLEDs.get(pos).on();
        redLEDs.get(pos).off();
    }

    public void orange(int pos) {
        greenLEDs.get(pos).on();
        redLEDs.get(pos).on();
    }

    public void off(int pos) {
        greenLEDs.get(pos).off();
        redLEDs.get(pos).off();
    }
}
