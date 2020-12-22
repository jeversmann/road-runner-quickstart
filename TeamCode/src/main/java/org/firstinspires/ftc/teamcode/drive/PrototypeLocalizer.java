package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class PrototypeLocalizer implements Localizer {
    private Pose2d poseEstimate, poseVelocity;

    private BNO055IMU imu;
    private DcMotorEx encoder;

    private Double lastWheelPosition;
    private double lastHeading;

    public PrototypeLocalizer(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        encoder = hardwareMap.get(DcMotorEx.class, "intake");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setPoseEstimate(new Pose2d());
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        poseEstimate = pose2d;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

    @Override
    public void update() {
        double wheelPosition = encoder.getCurrentPosition();
        double heading = getRawExternalHeading();
        if (lastWheelPosition != null) {
            double xDelta = encoderTicksToInches(wheelPosition - lastWheelPosition);
            double headingDelta = heading - lastHeading;
            Pose2d robotPoseDelta = new Pose2d(xDelta, 0, headingDelta);
            poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, robotPoseDelta);

            /*
            RobotLog.vv("Localizer", String.format("X: %.2f-%.2f=%.2f H: %.2f-%.2f=%.2f",
                    wheelPosition, lastWheelPosition, xDelta, heading, lastHeading, headingDelta));
             */
        }

        lastWheelPosition = wheelPosition;
        lastHeading = heading;
    }

    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    private double WHEEL_RADIUS = 1, GEAR_RATIO = 1, TICKS_PER_REV = 8192;

    private double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}
