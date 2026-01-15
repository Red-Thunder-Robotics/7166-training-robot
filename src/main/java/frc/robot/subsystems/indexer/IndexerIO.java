package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        double generalFeedMotorDutyCycle;
        double generalFeedMotorVelocityRPS;
        double generalFeedMotorCurrentAmps;

        double shooterFeedMotorDutyCycle;
        double shooterFeedMotorVelocityRPS;
        double shooterFeedMotorCurrentAmps;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void idle() {}

    public default void generalFeedDutyCycle(double output) {}
    public default void generalFeedStop() {}

    public default void shooterFeedDutyCycle(double output) {}
    public default void shooterFeedStop() {}
}
