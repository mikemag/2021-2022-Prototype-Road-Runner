package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm extends SubsystemBase {
    final DcMotorEx armMotor;
    final DigitalChannel armLimitSwitch;

    public Arm(HardwareMap hardwareMap) {
        armLimitSwitch = hardwareMap.get(DigitalChannel.class, "armLimitSwitch");

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setVelocityPIDFCoefficients(5.0, 0.1, 2.0, 7.0);
        // armDrive.setPositionPIDFCoefficients(50.0);

//        armResetMin();
    }

    public void raiseArm() {

    }

    public void lowerArm() {

    }

    public void stop() {

    }

    public void presetUp() {

    }

    public void presetDown() {

    }

}
