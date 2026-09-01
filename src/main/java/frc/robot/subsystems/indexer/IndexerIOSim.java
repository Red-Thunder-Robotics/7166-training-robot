package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;

public final class IndexerIOSim implements IndexerIO {
    // TODO 1: the plant. The motor curve for one Kraken X60 with FOC
    // Use LinearSystemId.createDCMotorSystem(gearbox, moi, gearing). 

    private double m_indexerTargetVelocity = 0d;

    private double m_topRollerTargetVelocity = 0d;
    private double m_topRollerVelocity = 0d;

    private double m_lowerKickerTargetVelocity = 0d;
    private double m_lowerKickerVelocity = 0d;

    // TODO 2: the stand-in controller. 
    // A PIDController for the correction and a SimpleMotorFeedforward for the guess.

    private final PIDController m_topRollerPID = new PIDController(0.5d, 0d, 0d);
    private final PIDController m_lowerKickerPID = new PIDController(0.5d, 0d, 0d);

    public IndexerIOSim() {}

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerTargetVelocityRPS = m_indexerTargetVelocity;

        // TODO 3: run one loop of the model. 
        // Read the speed off the plant
        // Ask the  feedforward for its guess and the PID for its correction, add the two.
        // Clamp the total to -12 and +12 with MathUtil.clamp.
        // Set it to zero while DriverStation.isDisabled(),
        // Hand it to the plant with setInputVoltage, then step the plant with update(0.02).

        // TODO 4: two readings off the plant. 
        // The speed goes in inputs.indexerVelocityRPS and the current in inputs.indexerCurrentAmps. 
        // Watch the units: getAngularVelocityRPM returns rotations per minute and the field is rotations per second.

        inputs.topRollerTargetVelocityRPS = m_topRollerTargetVelocity;
        m_topRollerVelocity += m_topRollerPID.calculate(m_topRollerVelocity);
        inputs.topRollerVelocityRPS = m_topRollerVelocity;

        inputs.lowerKickerTargetVelocityRPS = m_lowerKickerTargetVelocity;
        m_lowerKickerVelocity += m_lowerKickerPID.calculate(m_lowerKickerVelocity);
        inputs.lowerKickerVelocityRPS = m_lowerKickerVelocity;

        // TODO 5: Publish the error, target minus measurement, with
        // Logger.recordOutput under the key "Indexer/VelocityErrorRPS".

    }

    @Override
    public void indexerVelocity(AngularVelocity velocity) {
        // Used to call m_indexerPID.setSetpoint on the next line as well. The controller you
        // write in TODO 2 takes its setpoint as the second argument of calculate, so storing the
        // target is all this method has to do. 
        m_indexerTargetVelocity = velocity.in(RotationsPerSecond);
    }

    @Override
    public void indexerStop() {
        indexerVelocity(RotationsPerSecond.of(0d));
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
