package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Thing {
    DcMotor m1 = null;
    DcMotor m2 = null;

    public Thing() {
        // Set stuff up, of course!
    }

    // This does everything to move the thing, and doesn't return until it's all done.
    public void moveThing(int distance) throws InterruptedException {
        m1.setPower(0.5);
        Thread.sleep(1000 * distance);
        m1.setPower(0);
        m2.setPower(0.5);
        Thread.sleep(1000 * distance);
        m2.setPower(0);
    }

    public void stopThing() {
        // Nothing to do here, all the action happens in moveThing() and it's stopped after.
    }
}


