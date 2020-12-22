package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.drive.PrototypeLocalizer;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp
public class PrototypeAutoAim extends OpMode {
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final String VUFORIA_KEY = "AWmtrWr/////AAABmVZAFYszH01gnzhV0SLCT5k/+B6hxjVviSu+QFVs/0lA1NemfppuOIIuRmxJsozHDqCvbpb/61ZlpyJ54CVBpeqc5EY8t7m0yB3jAOyJOenvzOLNua0P5Z6ZeeCRqDUkNtdwURLQfr8Mm9hKt/n0Qs0atdAMQOPS6u8KsWGhEcKWEa+8GZc3+brWyPSW0J7Qwd/7ALLk99x6k7R9JA5i9X3Ql8pPxv1G9SqNfC9qJi1dizq3UXfcKWGvegdkQy6x3QX/K1I9scMdzD8Hq1PmYq7y6rrLyozhxBOAXJQwORcIIKzicoTyE4sTv+WblZrvVZPPkHs/fAtOv3se5EMqYOfxDHfoUBJPVTAS5PR8XsYP";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    WebcamName webcamName = null;
    private SampleTankDrive drive;
    private PrototypeHardware hardware;

    private boolean targetVisible = false;

    private VuforiaTrackables targetsUltimateGoal;
    private List<VuforiaTrackable> allTrackables;


    @Override
    public void init() {
        drive = new SampleTankDrive(hardwareMap);
        hardware = new PrototypeHardware(hardwareMap, false);

        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // Next, translate the camera lens to where it is on the robot.
        final float CAMERA_FORWARD_DISPLACEMENT = 12.0f * mmPerInch;
        final float CAMERA_VERTICAL_DISPLACEMENT = 9.0f * mmPerInch;
        final float CAMERA_LEFT_DISPLACEMENT = -12.0f * mmPerInch;

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                //.translation(CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT, CAMERA_FORWARD_DISPLACEMENT)
                .translation(0,0,0)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -90, 0));

        CameraManager cameraManager = ClassFactory.getInstance().getCameraManager();
        WebcamName webcamName = cameraManager.getAllWebcams().get(0);

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(webcamName, robotFromCamera);
        }
    }

    @Override
    public void start() {
        // This is the pose of the robot centered on the right starting line against the wall
        Pose2d startPose = new Pose2d(-60, -48, 0);
        drive.setPoseEstimate(startPose);
        targetsUltimateGoal.activate();
    }

    boolean autoAiming = false;
    boolean pusherMoving = false;
    int rings = 0;

    double CAMERA_TRUST = 0.0;

    @Override
    public void loop() {
        if (gamepad1.dpad_down) {
            autoAiming = true;
            rings = 3;
            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineTo(new Vector2d(0, -54), Math.toRadians(0))
                    .build());
        }

        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        if (autoAiming) {
            hardware.flywheel.setVelocity(hardware.FLYWHEEL_LAUNCH_LINE_VELOCITY);

            if (!drive.isBusy() && hardware.flywheel.getVelocity() < hardware.FLYWHEEL_LAUNCH_LINE_VELOCITY) {
                if (rings > 0) {
                    hardware.pusherForward();
                    sleep(hardware.PUSHER_DELAY);
                    hardware.pusherBack();
                    sleep(hardware.PUSHER_DELAY);
                    rings--;
                } else {
                    hardware.flywheel.setVelocity(0);
                    autoAiming = false;
                }
            }
        } else {
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                // If we're driving and the camera sees a target, update our position estimate
                Pose2d cameraPositionEstimate = new Pose2d(
                        translation.get(0) / mmPerInch,
                        translation.get(1) / mmPerInch,
                        Math.toRadians(rotation.thirdAngle - 90)
                );
                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("#FFFF00");
                DashboardUtil.drawRobot(fieldOverlay, cameraPositionEstimate);

                Pose2d currentEstimate = drive.getPoseEstimate();
                Pose2d poseDifference = cameraPositionEstimate.minus(currentEstimate);
                drive.setPoseEstimate(currentEstimate.plus(poseDifference.times(CAMERA_TRUST)));
            } else {
                telemetry.addData("Visible Target", "none");
            }

            Pose2d control = new Pose2d(
                    signSquare(gamepad1.left_stick_y) / 2,
                    0,
                    -signSquare(gamepad1.left_stick_x) / 2
            );
            drive.setWeightedDrivePower(control);
            telemetry.addLine(String.format("%.2f : %.2f", control.component1(), control.component3()));
        }

        if (gamepad1.y) {
            hardware.intakeOn();
        } else {
            hardware.intakeOff();
        }

        telemetry.update();
        drive.update(packet);
    }

    @Override
    public void stop() {
        targetsUltimateGoal.deactivate();
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private double signSquare(double d) {
        return Math.pow(d, 2) * (d < 0 ? -1 : 1);
    }
}
