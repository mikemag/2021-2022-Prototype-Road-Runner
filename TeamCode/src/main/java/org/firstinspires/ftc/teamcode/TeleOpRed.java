package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ArmCommands;
import org.firstinspires.ftc.teamcode.commands.DuckCommands;
import org.firstinspires.ftc.teamcode.commands.ExtensionCommands;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DuckWheel;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;

@TeleOp(name = "Red Teleop", group = "TeleOp")
public class TeleOpRed extends CommandOpMode {
    protected AllianceColor alliance = AllianceColor.RED;

    GamepadEx gamepad;

    MecanumDrivetrain drive;
    Intake intake;
    DuckWheel duckWheel;
    Arm arm;
    Extension extension;

    @Override
    public void initialize() {
        gamepad = new GamepadEx(gamepad1);

        drive = new MecanumDrivetrain(hardwareMap, true);
        intake = new Intake(hardwareMap);
        duckWheel = new DuckWheel(hardwareMap, alliance);
        arm = new Arm(hardwareMap, telemetry);
        extension = new Extension(hardwareMap, telemetry);
        register(drive, intake, duckWheel, arm, extension);

        arm.initialize();

        // Driving

        // We want to turn off velocity control for TeleOp. Velocity control per wheel is not
        // necessary outside of motion profiled auto.
        drive.disableVelocityControl();

        // Retrieve our pose from the end of the last Autonomous mode, if any.
        drive.setPoseEstimate(AutoToTeleStorage.currentPose);

        drive.setDefaultCommand(
                new MecanumDriveCommand(drive,
                        () -> gamepad.gamepad.left_stick_y,
                        () -> gamepad.gamepad.left_stick_x,
                        () -> gamepad.gamepad.right_stick_x,
                        () -> gamepad.gamepad.right_stick_button));

        // Intake
//        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> intake.spinIntakeIn()).whenReleased(() -> intake.stop());
//        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> intake.spinIntakeOut()).whenReleased(() -> intake.stop());

        // Duck Wheel
        gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new DuckCommands.SpinForward(duckWheel)).whenReleased(new DuckCommands.Stop(duckWheel));
        gamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(new DuckCommands.SpinBackwards(duckWheel)).whenReleased(new DuckCommands.Stop(duckWheel));
        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new DuckCommands.DeliverSingleDuck(duckWheel));

        // Arm
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenHeld(new ArmCommands.ArmUp(arm));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenHeld(new ArmCommands.ArmDown(arm));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new ArmCommands.ArmNextPreset(arm, ArmCommands.ArmNextPreset.Direction.UP));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new ArmCommands.ArmNextPreset(arm, ArmCommands.ArmNextPreset.Direction.DOWN));

        // Arm Extension System
//        gamepad.getGamepadButton(GamepadKeys.Button.A).and(gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP))
//                .whileActiveOnce(new ExtensionCommands.Extend(extension));
//        gamepad.getGamepadButton(GamepadKeys.Button.A).and(gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN))
//                .whileActiveOnce(new ExtensionCommands.Retract(extension));
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(new ExtensionCommands.Extend(extension));
        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(new ExtensionCommands.Retract(extension));

        // mmmfixme:
        //  - telemetry
        //    - want dual telemetry output
        //    - let any command or subsystem add telemetry
        //    - have some standard things, like alliance color, field-centric, arm position, etc.
        //    - do in SubSystem::periodic?
        //  - vision
        //  - arm extension in/out
        //  - autos -> tele: preselect the right one, pass pose.
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
