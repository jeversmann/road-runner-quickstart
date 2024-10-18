package org.firstinspires.ftc.teamcode.zippy;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

import java.util.ArrayList;
import java.util.List;

public class StatusLights {
    private List<LED> greenLEDs, redLEDs;

    public StatusLights(HardwareMap hardwareMap) {
        // I named everything backwards in the hardwaremap
        greenLEDs = new ArrayList<>();
        greenLEDs.add(hardwareMap.get(LED.class, "r0"));
        greenLEDs.add(hardwareMap.get(LED.class, "r1"));
        greenLEDs.add(hardwareMap.get(LED.class, "r2"));
        greenLEDs.add(hardwareMap.get(LED.class, "r3"));

        redLEDs = new ArrayList<>();
        redLEDs.add(hardwareMap.get(LED.class, "g0"));
        redLEDs.add(hardwareMap.get(LED.class, "g1"));
        redLEDs.add(hardwareMap.get(LED.class, "g2"));
        redLEDs.add(hardwareMap.get(LED.class, "g3"));

        off();
    }

    public void red(int pos) {
        greenLEDs.get(pos).off();
        redLEDs.get(pos).on();
    }

    public void red() {
        greenLEDs.forEach(LED::off);
        redLEDs.forEach(LED::on);
    }

    public void green(int pos) {
        greenLEDs.get(pos).on();
        redLEDs.get(pos).off();
    }
    public void green() {
        greenLEDs.forEach(LED::on);
        redLEDs.forEach(LED::off);
    }

    public void orange(int pos) {
        greenLEDs.get(pos).on();
        redLEDs.get(pos).on();
    }
    public void orange() {
        greenLEDs.forEach(LED::on);
        redLEDs.forEach(LED::on);
    }

    public void off(int pos) {
        greenLEDs.get(pos).off();
        redLEDs.get(pos).off();
    }

    public void off() {
        greenLEDs.forEach(LED::off);
        redLEDs.forEach(LED::off);
    }
}
