package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.indexer.IndexerConstants.indexerMotorReduction;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

public final class IndexerIOSim implements IndexerIO {
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final double MOI = 0.001;

    private final DCMotorSim m_indexerSim =
            new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, MOI, indexerMotorReduction), GEARBOX);

    private double m_indexerTargetVelocity = 0d;

    private double m_topRollerTargetVelocity = 0d;
    private double m_topRollerVelocity = 0d;

    private double m_lowerKickerTargetVelocity = 0d;
    private double m_lowerKickerVelocity = 0d;

    private final PIDController m_indexerPID = new PIDController(0.05d, 0d, 0d);
    private final SimpleMotorFeedforward m_indexerFF = new SimpleMotorFeedforward(0d, 0.248d);

    private final PIDController m_topRollerPID = new PIDController(0.5d, 0d, 0d);
    private final PIDController m_lowerKickerPID = new PIDController(0.5d, 0d, 0d);

    public IndexerIOSim() {}

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerTargetVelocityRPS = m_indexerTargetVelocity;

        double measured = m_indexerSim.getAngularVelocityRPM() / 60.0;
        double ff = m_indexerFF.calculate(m_indexerTargetVelocity);
        double fb = m_indexerPID.calculate(measured, m_indexerTargetVelocity);

        double volts = MathUtil.clamp(ff + fb, -12.0, 12.0);
        if (DriverStation.isDisabled()) {
            volts = 0.0;
        }
        m_indexerSim.setInputVoltage(volts);
        m_indexerSim.update(0.02);

        inputs.indexerVelocityRPS = m_indexerSim.getAngularVelocityRPM() / 60.0;
        inputs.indexerCurrentAmps = m_indexerSim.getCurrentDrawAmps();

        inputs.topRollerTargetVelocityRPS = m_topRollerTargetVelocity;
        m_topRollerVelocity += m_topRollerPID.calculate(m_topRollerVelocity);
        inputs.topRollerVelocityRPS = m_topRollerVelocity;

        inputs.lowerKickerTargetVelocityRPS = m_lowerKickerTargetVelocity;
        m_lowerKickerVelocity += m_lowerKickerPID.calculate(m_lowerKickerVelocity);
        inputs.lowerKickerVelocityRPS = m_lowerKickerVelocity;

        Logger.recordOutput("Indexer/VelocityErrorRPS", m_indexerTargetVelocity - inputs.indexerVelocityRPS);
    }

    @Override
    public void indexerVelocity(AngularVelocity velocity) {
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
