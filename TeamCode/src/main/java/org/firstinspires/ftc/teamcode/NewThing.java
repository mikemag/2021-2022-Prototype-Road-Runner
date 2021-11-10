package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class NewThing {
    DcMotor m1 = null;
    DcMotor m2 = null;

    private enum MoveThingState {
        Initial,
        WaitingForFirstMotionToFinish,
        DoSecondMotion,
        WaitingForSecondMotionToFinish,
        DoThirdMotion,
    }

    MoveThingState moveThingState = MoveThingState.Initial;
    int distanceToMove = 0;
    long delayStart = 0;

    public NewThing() {
        // Set stuff up, of course!
    }

    // This starts moving the thing and returns right away.
    public void moveThing(int distance) {
        // TODO: maybe throw an exception if state != State.Initial;
        m1.setPower(0.5);
        distanceToMove = distance;
        delayStart = System.currentTimeMillis();
        moveThingState = MoveThingState.WaitingForFirstMotionToFinish;
    }

    // This makes the next moves when it's time. Keep calling it in the main OpMode loop, all the time.
    public void update() {
        switch (moveThingState) {
            case WaitingForFirstMotionToFinish:
                if (System.currentTimeMillis() - delayStart >= 1000L * distanceToMove) {
                    moveThingState = MoveThingState.DoSecondMotion;
                }
                break;
            case DoSecondMotion:
                m1.setPower(0);
                m2.setPower(0.5);
                delayStart = System.currentTimeMillis();
                moveThingState = MoveThingState.WaitingForSecondMotionToFinish;
                break;
            case WaitingForSecondMotionToFinish:
                if (System.currentTimeMillis() - delayStart >= 1000L * distanceToMove) {
                    moveThingState = MoveThingState.DoThirdMotion;
                }
                break;
            case DoThirdMotion:
                m2.setPower(0);
                moveThingState = MoveThingState.Initial; // Ready to go again.
                break;
        }
    }

    // Call this if you want to know if the thing is still moving.
    public boolean isBusy() {
        return moveThingState != MoveThingState.Initial;
    }

    // Call this if you started moving the thing but don't want it to finish anymore.
    public void stopThing() {
        // Stop all the motors, wherever they are...
        m1.setPower(0);
        m2.setPower(0);

        // ... and we're done. (You could make an "interrupted" state if you wanted.)
        moveThingState = MoveThingState.Initial;
    }
}


