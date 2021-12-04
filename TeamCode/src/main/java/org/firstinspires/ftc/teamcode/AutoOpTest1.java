package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrivetrain(hardwareMap, true);
        intake = new Intake(hardwareMap);
        duckWheel = new DuckWheel(hardwareMap, alliance);
        arm = new Arm(hardwareMap, telemetry);
        extension = new Extension(hardwareMap, telemetry);
        CommandScheduler.getInstance().registerSubsystem(drive, intake, duckWheel, arm, extension);

        // Camera
        TSEDetector tseDetector = new TSEDetector(hardwareMap);
        tseDetector.setupCamera(TSEDetector.CAMERA_1_NAME);

        // Other subsystem init can go while the camera is getting to work.
        arm.initialize();

        // Display the TSE position while we're waiting to start the opmode.
        while (!isStarted()) {
            telemetry.addData("TSE Detector", "Position %s", tseDetector.getPosition());
            telemetry.update();

            // Sleep a bit so we don't waste battery while waiting. 20 times a second is plenty.
            sleep(50);
        }

        // Remember the position of the TSE before we start moving the bot!
        TSEDetector.TSEPosition tseStartingPosition = tseDetector.getPosition();

        Trajectory t1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .build();

        Trajectory t2 = drive.trajectoryBuilder(t1.end())
                .strafeLeft(10)
                .build();

        Trajectory t3 = drive.trajectoryBuilder(t2.end())
                .splineTo(new Vector2d(10, 10), Math.toRadians(90)) // +90 is counter-clockwise
                .build();

        CommandBase autoCmd = new SequentialCommandGroup(
                new FollowTrajectoryAsync(drive, t1)
//                , new WaitCommand(5000)
                , new DuckCommands.DeliverSingleDuck(duckWheel).alongWith(new FollowTrajectoryAsync(drive, t2))
                , new FollowTrajectoryAsync(drive, t3)
        );

        CommandScheduler.getInstance().schedule(autoCmd);

        while (!isStopRequested() && opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }

        AutoToTeleStorage.finalAutoPose = drive.getPoseEstimate();
        CommandScheduler.getInstance().reset();
    }
}


