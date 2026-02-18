package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public final class IndexerIOReal implements IndexerIO {
    private final TalonFX m_generalFeedMotor = new TalonFX(generalFeedMotorId, Constants.CANBUS);
    private final TalonFX m_shooterFeedMotor = new TalonFX(shooterFeedMotorId, Constants.CANBUS);

    private final StatusSignal<AngularVelocity> m_generalFeedVelocitySignal = m_generalFeedMotor.getVelocity();
    private final StatusSignal<Current> m_generalFeedCurrentSignal = m_generalFeedMotor.getSupplyCurrent();

    private final StatusSignal<AngularVelocity> m_shooterFeedVelocitySignal = m_shooterFeedMotor.getVelocity();
    private final StatusSignal<Current> m_shooterFeedCurrentSignal = m_shooterFeedMotor.getSupplyCurrent();

    private final DutyCycleOut m_generalFeedDutyCycleRequest = new DutyCycleOut(0d);
    private final DutyCycleOut m_shooterFeedDutyCycleRequest = new DutyCycleOut(0d);

    public IndexerIOReal() {
        var generalFeedConfig = new TalonFXConfiguration();
        generalFeedConfig.MotorOutput.NeutralMode = generalFeedNeutralMode;
        generalFeedConfig.MotorOutput.Inverted = generalFeedInverted;

        // generalFeedConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // generalFeedConfig.CurrentLimits.SupplyCurrentLimit = generalFeedCurrentLimit;

        var shooterFeedConfig = new TalonFXConfiguration();
        shooterFeedConfig.MotorOutput.NeutralMode = shooterFeedNeutralMode;
        shooterFeedConfig.MotorOutput.Inverted = shooterFeedInverted;

        // shooterFeedConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // shooterFeedConfig.CurrentLimits.SupplyCurrentLimit = shooterFeedCurrentLimit;

        PhoenixUtil.tryUntilOk(5, () -> m_generalFeedMotor.getConfigurator().apply(generalFeedConfig));
        PhoenixUtil.tryUntilOk(5, () -> m_shooterFeedMotor.getConfigurator().apply(shooterFeedConfig));

        BaseStatusSignal.setUpdateFrequencyForAll(50d, m_generalFeedVelocitySignal, m_generalFeedCurrentSignal, m_shooterFeedVelocitySignal, m_shooterFeedCurrentSignal);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        BaseStatusSignal.refreshAll(m_generalFeedVelocitySignal, m_generalFeedCurrentSignal, m_shooterFeedVelocitySignal, m_shooterFeedCurrentSignal);
        
        inputs.generalFeedMotorDutyCycle = PhoenixUtil.getRequestDutyCycle(m_generalFeedMotor.getAppliedControl());
        inputs.generalFeedMotorVelocityRPS = m_generalFeedVelocitySignal.getValueAsDouble();
        inputs.generalFeedMotorCurrentAmps = m_generalFeedCurrentSignal.getValueAsDouble();

        inputs.shooterFeedMotorDutyCycle = PhoenixUtil.getRequestDutyCycle(m_generalFeedMotor.getAppliedControl());
        inputs.shooterFeedMotorVelocityRPS = m_shooterFeedVelocitySignal.getValueAsDouble();
        inputs.shooterFeedMotorCurrentAmps = m_shooterFeedCurrentSignal.getValueAsDouble();
    }

    @Override
    public void idle() {
        m_generalFeedMotor.disable();
        m_shooterFeedMotor.disable();
    }

    @Override
    public void generalFeedDutyCycle(double output) {
        m_generalFeedMotor.setControl(m_generalFeedDutyCycleRequest.withOutput(output));
    }
    @Override
    public void generalFeedStop() {
        m_generalFeedMotor.disable();
    }
    @Override
    public void shooterFeedDutyCycle(double output) {
        m_shooterFeedMotor.setControl(m_shooterFeedDutyCycleRequest.withOutput(output));
    }
    @Override
    public void shooterFeedStop() {
        m_shooterFeedMotor.disable();
    }
}
