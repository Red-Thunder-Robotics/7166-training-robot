package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        double targetPositionRotations;
        double targetPositionDegrees;

        double positionRotations;
        double positionDegrees;

        double dutyCycle;
        double velocityRPS;
        double currentAmps;
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default void idle() {}

    public default void setPosition(double position) {}
}
