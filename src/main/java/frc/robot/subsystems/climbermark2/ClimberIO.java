package frc.robot.subsystems.climbermark2;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        // actuator
        double actuatorTargetPositionRotations;
        double actuatorTargetPositionInches;

        double actuatorPositionRotations;
        double actuatorPositionInches;

        double actuatorDutyCycle;
        double actuatorVelocityRPS;
        double actuatorCurrentAmps;

        // grab
        double grabTargetPositionRotations;
        double grabTargetPositionDegrees;

        // grab leader
        double grabLeaderPositionRotations;
        double grabLeaderPositionInches;

        double grabLeaderDutyCycle;
        double grabLeaderVelocityRPS;
        double grabLeaderCurrentAmps;

        // grab follower
        double grabFollowerPositionRotations;
        double grabFollowerPositionInches;

        double grabFollowerVelocityRPS;
        double grabFollowerCurrentAmps;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void idle() {}

    public default void setActuatorPosition(double position) {}
    public default void setGrabPosition(double position) {}
}
