package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Mikemag 7760 Single Controller", group = "Linear Opmode")
public class Mikemag7760Teleop extends FTC7760OpBase {

    @Override
    public void runOpMode() {
        setupRobot();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (gamepad1.start) {
                fieldCentricDriving = gamepad1.a;
            }

            // Driving input
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x,
                    gamepad1.right_trigger > 0.1);

            // Manual Quack Wheel input
            quackWheelManualBlue = gamepad1.a;
            quackWheelManualRed = gamepad1.y && !gamepad1.a;
            quackWheelManual();

            // Single duck Quack Wheel input
            quackWheelSingleBlue = gamepad1.x;
            quackWheelSingleRed = gamepad1.b && !gamepad1.x;
            quackWheelSingle();

            // Intake input
            intakeIn = gamepad1.left_bumper;
            intakeOut = gamepad1.right_bumper && !gamepad1.left_bumper;
            intake();

            // Arm input
//            armUp = gamepad1.dpad_up;
//            armDown = gamepad1.dpad_down && !gamepad1.dpad_up;
//            armManual();

            telemetry();
        }
    }
}