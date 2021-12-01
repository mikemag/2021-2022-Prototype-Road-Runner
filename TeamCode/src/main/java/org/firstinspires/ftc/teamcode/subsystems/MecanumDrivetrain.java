package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class MecanumDrivetrain extends SubsystemBase {
    private final SampleMecanumDrive drive;
    private final boolean fieldCentric;

    public MecanumDrivetrain(HardwareMap hardwareMap, boolean isFieldCentric) {
        this.drive = new SampleMecanumDrive(hardwareMap);
        fieldCentric = isFieldCentric;
    }

    public void drive(double leftY, double leftX, double rightX) {
        Vector2d input = new Vector2d(-leftY, -leftX).rotated(fieldCentric ? -getHeading() : 0);
        drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -rightX));
    }

    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public void disableVelocityControl() {
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPoseEstimate(Pose2d poseEstimate) {
        drive.setPoseEstimate(poseEstimate);
    }

}
