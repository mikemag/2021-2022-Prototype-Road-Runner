package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@TeleOp(group="tmp")
public class TeleOpBasic extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        DcMotorEx duckMotor = hardwareMap.get(DcMotorEx.class, "duckMotor");
        CRServo intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        DigitalChannel sideButton = hardwareMap.get(DigitalChannel.class, "sideButton");
        DigitalChannel sideButton2 = hardwareMap.get(DigitalChannel.class, "sideButton2");
        DigitalChannel armLimitSwitch = hardwareMap.get(DigitalChannel.class, "armLimitSwitch");
        DigitalChannel armLimitSwitch2 = hardwareMap.get(DigitalChannel.class, "armLimitSwitch2");
        Rev2mDistanceSensor distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");

        waitForStart();

        if (isStopRequested()) return;

        // mmmfixme: this is a basic RR example of driving w/ field centric.
        // - I want this to be integrated with FTCLib.
        while (opModeIsActive() && !isStopRequested()) {
            FtcDashboard.getInstance().getTelemetry().addData("Sensor tests", "Arm Switch: %s", sideButton.toString());
            telemetry.addData("Sensor tests", "Side button: %s", sideButton.getState()); // mmmfixme: true always
            telemetry.addData("Sensor tests", "Side button 2: %s", sideButton2.getState()); // mmmfixme: true normally or disconnected, false when pressed
            telemetry.addData("Sensor tests", "Arm Switch: %s", armLimitSwitch.getState()); // mmmfixme: true normally or disconnected, false when triggered
            telemetry.addData("Sensor tests", "Arm Switch 2: %s", armLimitSwitch2.getState()); // mmmfixme: true normally or disconnected, false when triggered

//            intakeServo.setPower(1.0);
            duckMotor.setVelocity(1000);

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            // Update everything. Odometry. Etc.
            drive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
