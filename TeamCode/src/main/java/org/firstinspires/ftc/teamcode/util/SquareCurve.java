package org.firstinspires.ftc.teamcode.util;

public class SquareCurve {

    public static double from(double input) {
        return Math.copySign(Math.pow(input, 2), input);
    }
}
