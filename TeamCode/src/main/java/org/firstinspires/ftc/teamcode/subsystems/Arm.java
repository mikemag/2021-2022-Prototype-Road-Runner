package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Arm extends SubsystemBase {
    final DcMotorEx armMotor;
    final DigitalChannel armLimitSwitch;
    final Telemetry telemetry;
    List<LynxModule> lynxModules;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        armLimitSwitch = hardwareMap.get(DigitalChannel.class, "armLimitSwitch");

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPositionTolerance(10);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setVelocityPIDFCoefficients(5.0, 0.1, 2.0, 7.0);
        // armDrive.setPositionPIDFCoefficients(50.0);

        // TODO: feels like this ought to be somewhere more general. A overall robot class or subsystem?
        lynxModules = hardwareMap.getAll(LynxModule.class);
    }

    public void setPosition(int position) {
        armMotor.setTargetPosition(position);
        armMotor.setPower(0.8);
    }

    public int getCurrentPosition() {
        return armMotor.getCurrentPosition();
    }

    public void resetPositionCache() {
        for (LynxModule module : lynxModules) {
            module.clearBulkCache();
        }
    }

    public void holdCurrentPosition() {
        // We have to clear the bulk cache since concurrent commands may have populated the cache
        // with a stale motor position.
        resetPositionCache();
        armMotor.setTargetPosition(armMotor.getCurrentPosition());
        armMotor.setPower(0.8);
    }

    public void armResetMin() {
        // A little timeout, just in case the limit switch is broken.
        ElapsedTime runtime = new ElapsedTime();
        while (armLimitSwitch.getState() && runtime.seconds() < 8) {
            armMotor.setTargetPosition(-5000); // ~5000 encoder ticks in a full revolution.
            armMotor.setPower(0.25);
        }
        armLimitSwitchReset();
    }

    public void armLimitSwitchReset() {
        armMotor.setPower(0.0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean isArmAtLowerLimit() {
        return !armLimitSwitch.getState();
    }
}
