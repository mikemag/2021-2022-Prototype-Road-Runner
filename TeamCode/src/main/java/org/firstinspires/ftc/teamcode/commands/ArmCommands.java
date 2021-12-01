package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

import java.util.Arrays;

public class ArmCommands {

    private static class ArmUpDown extends CommandBase {
        final Arm arm;
        final int destination;

        public ArmUpDown(Arm arm, int destination) {
            this.arm = arm;
            this.destination = destination;
            addRequirements(arm);
        }

        @Override
        public void initialize() {
            arm.setPosition(destination);
        }

        @Override
        public void end(boolean isInterrupted) {
            arm.holdCurrentPosition();
        }
    }

    public static class ArmUp extends ArmUpDown {
        public ArmUp(Arm arm) {
            super(arm, 4000);
        }
    }

    public static class ArmDown extends ArmUpDown {
        public ArmDown(Arm arm) {
            super(arm, 0);
        }
    }

    public static class ArmNextPreset extends CommandBase {
        private static final int[] presetPositions = {0, 200, 2200, 2800, 3500};

        // Factor in a small tolerance to the current position so we don't go from 1999 to 2000.
        private static final int TOLERANCE = 50;

        public enum Direction {UP, DOWN}

        final Arm arm;
        final Direction direction;

        public ArmNextPreset(Arm arm, Direction direction) {
            this.arm = arm;
            this.direction = direction;
        }

        @Override
        public void initialize() {
            // We have to clear the bulk cache since concurrent commands may have populated the cache
            // with a stale motor position.
            arm.resetPositionCache();
            int pos = arm.getCurrentPosition();

            if (direction == Direction.UP) {
                // nb: read the doc for binarySearch to understand the negative coding for the
                // insertion point.
                int point = Arrays.binarySearch(presetPositions, pos + TOLERANCE);
                point = Math.abs(point + 1);
                if (point < presetPositions.length) {
                    arm.setPosition(presetPositions[point]);
                }
            } else {
                int point = Arrays.binarySearch(presetPositions, pos - TOLERANCE);
                if (point >= 0) {
                    point -= 1;
                } else {
                    point = Math.abs(point + 1) - 1;
                }
                if (point >= 0) {
                    arm.setPosition(presetPositions[point]);
                }
            }
        }

        @Override
        public final boolean isFinished() {
            return true;
        }
    }


}
