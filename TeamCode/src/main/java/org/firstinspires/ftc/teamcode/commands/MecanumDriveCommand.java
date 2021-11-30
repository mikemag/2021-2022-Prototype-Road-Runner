package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class MecanumDriveCommand extends CommandBase {

    private final MecanumDrivetrain drive;
    private final DoubleSupplier forward, strafe, rotate;
    private final BooleanSupplier slowdown;

    public MecanumDriveCommand(MecanumDrivetrain drive, DoubleSupplier forward,
                               DoubleSupplier strafe, DoubleSupplier rotate,
                               BooleanSupplier slowdown) {
        this.drive = drive;
        this.strafe = strafe;
        this.forward = forward;
        this.rotate = rotate;
        this.slowdown = slowdown;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        boolean goSlow = slowdown.getAsBoolean();
        drive.drive(
                slowItDown(forward.getAsDouble(), goSlow),
                slowItDown(strafe.getAsDouble(), goSlow),
                slowItDown(rotate.getAsDouble(), goSlow));

        Pose2d poseEstimate = drive.getPoseEstimate();
        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }

    private double slowItDown(double v, boolean goSlow) {
        return goSlow ? v * Math.abs(v) : v;
    }
}
