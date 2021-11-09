package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MikemagMecanum New", group = "Linear Opmode")
public class MikemagMecanum extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;
    private BNO055IMU imu = null;
    private final boolean fieldCentric = true;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");

        // Most robots need the motor on one side to be reversed to drive forward. Reverse the motor
        // that runs backwards when connected directly to the battery.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParms = new BNO055IMU.Parameters();
        imuParms.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(imuParms);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Extra 10% to counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            if (fieldCentric) {
                double angle = -imu.getAngularOrientation().firstAngle;
                telemetry.addData("Heading", "%f", angle);

                // From https://www.ctrlaltftc.com/practical-examples/drivetrain-control
                double x_rotated = x * Math.cos(angle) - y * Math.sin(angle);
                double y_rotated = x * Math.sin(angle) + y * Math.cos(angle);
                x = x_rotated;
                y = y_rotated;
            }

            // From https://gm0.org/en/latest/docs/software/mecanum-drive.html
            // Denominator is the largest motor power (absolute value) or 1.
            // This ensures all the powers maintain the same ratio, but only when at least one is
            // out of the range [-1, 1].
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFrontDrive.setPower(frontLeftPower);
            leftRearDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightRearDrive.setPower(backRightPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
