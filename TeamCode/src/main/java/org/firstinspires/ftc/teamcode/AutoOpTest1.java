package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.DuckCommands;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectoryAsync;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DuckWheel;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;

@Autonomous(name = "Auto Test 1", group = "Tests", preselectTeleOp = "Red Teleop")
public class AutoOpTest1 extends LinearOpMode {
    protected AllianceColor alliance = AllianceColor.RED;

    MecanumDrivetrain drive;
    Intake intake;
    DuckWheel duckWheel;
    Arm arm;
    Extension extension;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrivetrain(hardwareMap, true);
        intake = new Intake(hardwareMap);
        duckWheel = new DuckWheel(hardwareMap, alliance);
        arm = new Arm(hardwareMap, telemetry);
        extension = new Extension(hardwareMap, telemetry);
        CommandScheduler.getInstance().registerSubsystem(drive, intake, duckWheel, arm, extension);

        // mmmfixme: setup camera

        arm.initialize();

        while (!isStarted()) {
            // mmmfixme: capture camera output, log it so we can confirm.
        }

        Trajectory t = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
//                .forward(5)
                .build();

        CommandBase autoCmd = new SequentialCommandGroup(
                new InstantCommand(() -> drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)))),
                new FollowTrajectoryAsync(drive, t),
                new DuckCommands.DeliverSingleDuck(duckWheel)
                );

        CommandScheduler.getInstance().schedule(autoCmd);

        while (!isStopRequested() && opModeIsActive()) {
            CommandScheduler.getInstance().run();
            // mmmfixme: pull in the loop timing stuff from my teleops, I'm curious.
        }

        AutoToTeleStorage.finalAutoPose = drive.getPoseEstimate();
        CommandScheduler.getInstance().reset();
    }
}


