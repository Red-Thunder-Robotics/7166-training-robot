package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public final class IndexerIOReal implements IndexerIO {
    private final TalonFX m_motor = new TalonFX(motorId, Constants.CANBUS);

    private final StatusSignal<AngularVelocity> m_velocitySignal = m_motor.getVelocity();
    private final StatusSignal<Current> m_currentSignal = m_motor.getSupplyCurrent();

    private final MotionMagicVelocityVoltage m_velocityRequest = new MotionMagicVelocityVoltage(0d)
        .withEnableFOC(true);

    public IndexerIOReal() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = neutralMode;
        config.MotorOutput.Inverted = inverted;
        
        // config.CurrentLimits.SupplyCurrentLimitEnable = true;
        // config.CurrentLimits.SupplyCurrentLimit = currentLimit;

        config.Slot0.kP = pidP;
        config.Slot0.kV = pidV;
        config.MotionMagic.MotionMagicAcceleration = targetAcceleration;

        PhoenixUtil.tryUntilOk(5, () -> m_motor.getConfigurator().apply(config));

        BaseStatusSignal.setUpdateFrequencyForAll(50d, m_velocitySignal, m_currentSignal);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        BaseStatusSignal.refreshAll(m_velocitySignal, m_currentSignal);

        inputs.targetVelocityRPS = PhoenixUtil.getRequestVelocity(m_motor.getAppliedControl());
        
        inputs.velocityRPS = m_velocitySignal.getValueAsDouble();
        inputs.currentAmps = m_currentSignal.getValueAsDouble();
    }

    @Override
    public void idle() {
        m_motor.disable();
    }

    @Override
    public void runVelocity(AngularVelocity velocity) {
        m_motor.setControl(m_velocityRequest.withVelocity(velocity));
    }
}
