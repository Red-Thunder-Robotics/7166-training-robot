package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;

public final class IndexerIOSim implements IndexerIO {
    // TODO 1: two doubles for the indexer, the target and the current speed, both starting at
    // 0d. The top roller pair below is the same two lines.

    private double m_topRollerTargetVelocity = 0d;
    private double m_topRollerVelocity = 0d;

    private double m_lowerKickerTargetVelocity = 0d;
    private double m_lowerKickerVelocity = 0d;

    // TODO 2: one PIDController for the indexer, same gains as the two below.
    private final PIDController m_topRollerPID = new PIDController(0.5d, 0d, 0d);
    private final PIDController m_lowerKickerPID = new PIDController(0.5d, 0d, 0d);

    public IndexerIOSim() {}

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        // TODO 3: three lines, copying the top roller block below.
        // Report the target, then move the current speed towards it by adding the PID output onto
        // it, then report the result. 

        inputs.topRollerTargetVelocityRPS = m_topRollerTargetVelocity;
        m_topRollerVelocity += m_topRollerPID.calculate(m_topRollerVelocity);
        inputs.topRollerVelocityRPS = m_topRollerVelocity;

        inputs.lowerKickerTargetVelocityRPS = m_lowerKickerTargetVelocity;
        m_lowerKickerVelocity += m_lowerKickerPID.calculate(m_lowerKickerVelocity);
        inputs.lowerKickerVelocityRPS = m_lowerKickerVelocity;
    }

    @Override
    public void indexerVelocity(AngularVelocity velocity) {
        // TODO 4: two lines. Store the velocity in rotations per second, and give the same number
        // to the PID controller as its setpoint. topRollerVelocity below is the same two lines.
    }

    @Override
    public void indexerStop() {
        // TODO 5: one line. In simulation, stopping is commanding zero, because there is no
        // motor controller here to fall back to a neutral mode. topRollerStop below is the same.
    }

    @Override
    public void topRollerVelocity(AngularVelocity velocity) {
        m_topRollerTargetVelocity = velocity.in(RotationsPerSecond);
        m_topRollerPID.setSetpoint(m_topRollerTargetVelocity);
    }

    @Override
    public void topRollerStop() {
        topRollerVelocity(RotationsPerSecond.of(0d));
    }

    @Override
    public void lowerKickerVelocity(AngularVelocity velocity) {
        m_lowerKickerTargetVelocity = velocity.in(RotationsPerSecond);
        m_lowerKickerPID.setSetpoint(m_lowerKickerTargetVelocity);
    }

    @Override
    public void lowerKickerStop() {
        lowerKickerVelocity(RotationsPerSecond.of(0d));
    }
}
