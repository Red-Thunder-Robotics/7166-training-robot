package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        double targetVelocityRPS;
    
        double velocityRPS;
        double currentAmps;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void idle() {}

    public default void runVelocity(AngularVelocity velocity) {}
}
