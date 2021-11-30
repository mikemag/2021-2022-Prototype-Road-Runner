package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;

@TeleOp()
public class TeleOpGeneric extends CommandOpMode {
    MecanumDrivetrain drive;
    GamepadEx gamepad;

    @Override
    public void initialize() {
        drive = new MecanumDrivetrain(hardwareMap, true);
        register(drive);

        gamepad = new GamepadEx(gamepad1);
        drive.setDefaultCommand(
                // mmmfixme: add in a boolean provider to tell us to square the stick inputs.
                new MecanumDriveCommand(drive,
                        () -> gamepad.gamepad.left_stick_y,
                        () -> gamepad.gamepad.left_stick_x,
                        () -> gamepad.gamepad.right_stick_x,
                        () -> gamepad.gamepad.left_bumper));
    }

    long loopMin = Long.MAX_VALUE;
    long loopMax = Long.MIN_VALUE;
    double loopAvg = 0;
    long loopCnt = 0;

    @Override
    public void run() {
        long loopStart = System.nanoTime();

        CommandScheduler.getInstance().run();

        long loopEnd = System.nanoTime();
        long delta = loopEnd - loopStart;
        loopMin = Math.min(loopMin, delta);
        loopMax = Math.max(loopMax, delta);
        loopAvg += (delta - loopAvg) / Math.min(++loopCnt, 1000);

        telemetry.addData("Loop timinngs", "min/avg/max: %,.2fms/%,.2fms/%,.2fms",
                loopMin / 1_000_000.0, loopAvg / 1_000_000.0, loopMax / 1_000_000.0);
        telemetry.update();
    }

}
