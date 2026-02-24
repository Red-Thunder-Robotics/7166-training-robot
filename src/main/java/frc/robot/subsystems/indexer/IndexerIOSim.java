package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;

public final class IndexerIOSim implements IndexerIO {
    private double m_targetVelocity = 0d;
    private double m_velocity = 0d;

    private final PIDController m_PID = new PIDController(0.5d, 0d, 0d);

    public IndexerIOSim() {

    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.targetVelocityRPS = m_targetVelocity;

        m_velocity += m_PID.calculate(m_velocity);
        inputs.velocityRPS = m_velocity;
    }

    @Override
    public void idle() {
        m_targetVelocity = 0d;
    }

    @Override
    public void runVelocity(AngularVelocity velocity) {
        m_targetVelocity = velocity.in(RotationsPerSecond);
        m_PID.setSetpoint(m_targetVelocity);
    }
}
