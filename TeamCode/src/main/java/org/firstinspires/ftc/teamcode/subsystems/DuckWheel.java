package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AllianceColor;

public class DuckWheel extends SubsystemBase {
    private final DcMotorEx duckMotor;

    // The fast and slow speeds for automated spinning
    private final int AUTO_SLOW_SPEED = 500;
    private final int AUTO_FAST_SPEED = 10000;

    // Positions where the quack wheel automatically starts spinning quickly, and then stops
    // spinning; used to spin off a single duck.
    private final int AUTO_START_FAST_POSITION = 400; // encoder ticks
    private final int AUTO_STOP_POSITION = 2000; // encoder ticks

    public DuckWheel(HardwareMap hardwareMap, AllianceColor alliance) {
        duckMotor = hardwareMap.get(DcMotorEx.class, "duckMotor");
        duckMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        duckMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        duckMotor.setDirection(alliance == AllianceColor.RED ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
    }

    public void spinForward() {
        duckMotor.setVelocity(AUTO_SLOW_SPEED);
    }

    public void spinBackwards() {
        duckMotor.setVelocity(-AUTO_SLOW_SPEED);
    }

    public void stop() {
        duckMotor.setVelocity(0);
    }

    public void reset() {
        duckMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runSlowArc() {
        duckMotor.setTargetPosition(AUTO_START_FAST_POSITION);
        duckMotor.setVelocity(AUTO_SLOW_SPEED);
    }

    public void runFastArc() {
        duckMotor.setTargetPosition(AUTO_STOP_POSITION);
        duckMotor.setVelocity(AUTO_FAST_SPEED);
    }

    public boolean isBusy() {
        return duckMotor.isBusy();
    }

}
