package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="Mecanum Diagnostics")
public class Diagnostics extends OpMode
{
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "back_right_drive");
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "front_right_drive");

        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    private final double TEST_SPEED = 0.8;
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double backLeftSpeed = 0;
        double backRightSpeed = 0;
        double frontLeftSpeed = 0;
        double frontRightSpeed = 0;

        if (gamepad1.a) {
            backRightSpeed = TEST_SPEED;
        }
        if (gamepad1.b) {
            frontRightSpeed = TEST_SPEED;
        }
        if (gamepad1.y) {
            frontLeftSpeed = TEST_SPEED;
        }
        if (gamepad1.x) {
            backLeftSpeed = TEST_SPEED;
        }

        backLeftDrive.setPower(backLeftSpeed);
        frontLeftDrive.setPower(frontLeftSpeed);
        backRightDrive.setPower(backRightSpeed);
        frontRightDrive.setPower(frontRightSpeed);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Motors", "F: %.2f/%.2f B: %.2f/%.2f",
                frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
