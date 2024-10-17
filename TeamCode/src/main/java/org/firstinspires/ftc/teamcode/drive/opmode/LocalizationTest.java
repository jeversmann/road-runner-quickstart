package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.zippy.StatusLights;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        StatusLights lights = new StatusLights(hardwareMap);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d robotInBlueObservationCorner = new Pose2d(-72.0 + 7.25, 72.0 - 8.75, 0);

        drive.setPoseEstimate(robotInBlueObservationCorner);

        waitForStart();

        Pose2d llpose = robotInBlueObservationCorner;

        while (!isStopRequested()) {

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            double headingRads = poseEstimate.getHeading();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", headingRads);

            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate);

            limelight.updateRobotOrientation(Math.toDegrees(headingRads));
            LLResult llresult = limelight.getLatestResult();
            if (llresult == null) {
                lights.off(0);
            } else if(!llresult.isValid()) {
                lights.red(0);
            } else {
                lights.green(0);
                llpose = convert(llresult.getBotpose_MT2());
            }

            telemetry.addData("limelight x", llpose.getX());
            telemetry.addData("limelight y", llpose.getY());
            telemetry.addData("limelight heading", llpose.getHeading());

            fieldOverlay.setStroke("#0DC681");
            DashboardUtil.drawRobot(fieldOverlay, llpose);

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }

    private Pose2d convert(Pose3D pose) {
        Position pos = pose.getPosition();
        YawPitchRollAngles or = pose.getOrientation();
        // Convert from meters to inches and degrees to radians.
        return new Pose2d(pos.x * 39.37, pos.y * 39.37, Math.toRadians(or.getYaw()));
    }
}
