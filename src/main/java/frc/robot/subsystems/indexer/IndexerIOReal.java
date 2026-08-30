package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.PhoenixUtil.MotorAction;

public final class IndexerIOReal implements IndexerIO {
    // TODO 1: the motor.
    // One private final TalonFX. Its constructor takes the CAN id and the name of the bus the device is on. 
    // indexerMotorId comes from the static import at the top of the file

    private final TalonFX m_topRollerMotor = new TalonFX(topRollerMotorId, Constants.CANBUS);
    private final TalonFX m_lowerKickerMotor = new TalonFX(lowerKickerMotorId, Constants.CANBUS);

    // TODO 2: the two status signals, velocity and supply current.
    // Copy the shape of the four below. getVelocity() does not read the motor; it gives you a
    // handle on the newest copy the library already holds, so you ask for it once here and keep it.

    private final StatusSignal<AngularVelocity> m_topRollerVelocitySignal = m_topRollerMotor.getVelocity();
    private final StatusSignal<Current> m_topRollerCurrentSignal = m_topRollerMotor.getSupplyCurrent();

    private final StatusSignal<AngularVelocity> m_lowerKickerVelocitySignal = m_lowerKickerMotor.getVelocity();
    private final StatusSignal<Current> m_lowerKickerCurrentSignal = m_lowerKickerMotor.getSupplyCurrent();

    // TODO 3: the control request.
    // One MotionMagicVelocityVoltage, built at 0d, with .withEnableFOC(true). 

    private final MotionMagicVelocityVoltage m_topRollerVelocityRequest =
            new MotionMagicVelocityVoltage(0d).withEnableFOC(true);
    private final MotionMagicVelocityVoltage m_lowerKickerVelocityRequest =
            new MotionMagicVelocityVoltage(0d).withEnableFOC(true);

    public IndexerIOReal() {
        // TODO 4: the configuration.
        // Make a TalonFXConfiguration for the indexer roller and set eight things on it. The top
        // roller block directly below is the same eight in the same order.
        //   MotorOutput.NeutralMode                 indexerNeutralMode
        //   MotorOutput.Inverted                    indexerInverted
        //   CurrentLimits.SupplyCurrentLimitEnable  true
        //   CurrentLimits.SupplyCurrentLimit        indexerCurrentLimit
        //   Feedback.SensorToMechanismRatio         indexerMotorReduction
        //   Slot0.kP                                indexerPidP
        //   Slot0.kV                                indexerPidV
        //   MotionMagic.MotionMagicAcceleration     indexerTargetAcceleration
        // Every value is already in IndexerConstants. You are not choosing numbers.

        var topRollerConfig = new TalonFXConfiguration();
        topRollerConfig.MotorOutput.NeutralMode = topRollerNeutralMode;
        topRollerConfig.MotorOutput.Inverted = topRollerInverted;
        //
        topRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        topRollerConfig.CurrentLimits.SupplyCurrentLimit = topRollerCurrentLimit;
        //
        topRollerConfig.Feedback.SensorToMechanismRatio = topRollerReduction;
        //
        topRollerConfig.Slot0.kP = topRollerPidP;
        topRollerConfig.Slot0.kV = topRollerPidV;
        topRollerConfig.MotionMagic.MotionMagicAcceleration = topRollerTargetAcceleration;

        var lowerKickerConfig = new TalonFXConfiguration();
        lowerKickerConfig.MotorOutput.NeutralMode = lowerKickerNeutralMode;
        lowerKickerConfig.MotorOutput.Inverted = lowerKickerInverted;
        //
        lowerKickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        lowerKickerConfig.CurrentLimits.SupplyCurrentLimit = lowerKickerCurrentLimit;
        //
        lowerKickerConfig.Slot0.kP = indexerPidP;
        lowerKickerConfig.Slot0.kV = indexerPidV;
        lowerKickerConfig.MotionMagic.MotionMagicAcceleration = indexerTargetAcceleration;

        // PhoenixUtil.tryUntilOk(5, () -> m_indexerMotor.getConfigurator().apply(indexerConfig));
        // PhoenixUtil.tryUntilOk(5, () -> m_topRollerMotor.getConfigurator().apply(topRollerConfig));
        // PhoenixUtil.tryUntilOk(5, () -> m_lowerKickerMotor.getConfigurator().apply(lowerKickerConfig));

        // TODO 4, second half: send the configuration you just built.
        // MotorAction.configureMotor("Indexer Roller", <motor>, <config>).run();
        // run() waits a quarter second for the controller to acknowledge, and retries five times.

        MotorAction.configureMotor("Indexer Top Roller", m_topRollerMotor, topRollerConfig)
                .run();
        MotorAction.configureMotor("Lower Kicker", m_lowerKickerMotor, lowerKickerConfig)
                .run();

        // TODO 5: add your two signals to this call, at the front of the list.
        // This tells the controller how often to broadcast those readings: 50 times a second, once
        // per robot loop. D

        BaseStatusSignal.setUpdateFrequencyForAll(
                50d,
                m_topRollerVelocitySignal,
                m_topRollerCurrentSignal,
                m_lowerKickerVelocitySignal,
                m_lowerKickerCurrentSignal);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        // TODO 6, first half: add your two signals to this call, at the front of the list.
        // One call rather than one per signal, because each call costs its own wait.

        BaseStatusSignal.refreshAll(
                m_topRollerVelocitySignal,
                m_topRollerCurrentSignal,
                m_lowerKickerVelocitySignal,
                m_lowerKickerCurrentSignal);

        // TODO 6, second half: fill in the three indexer fields. 
        //   indexerTargetVelocityRPS  PhoenixUtil.getRequestVelocity(<motor>.getAppliedControl())
        //   indexerVelocityRPS        the velocity signal's getValueAsDouble()
        //   indexerCurrentAmps        the current signal's getValueAsDouble()

        inputs.topRollerTargetVelocityRPS = PhoenixUtil.getRequestVelocity(m_topRollerMotor.getAppliedControl());
        inputs.topRollerVelocityRPS = m_topRollerVelocitySignal.getValueAsDouble();
        inputs.topRollerCurrentAmps = m_topRollerCurrentSignal.getValueAsDouble();

        inputs.lowerKickerTargetVelocityRPS = PhoenixUtil.getRequestVelocity(m_lowerKickerMotor.getAppliedControl());
        inputs.lowerKickerVelocityRPS = m_lowerKickerVelocitySignal.getValueAsDouble();
        inputs.lowerKickerCurrentAmps = m_lowerKickerCurrentSignal.getValueAsDouble();
    }

    @Override
    public void indexerVelocity(AngularVelocity velocity) {
        // TODO 7: one line. Send the request from TODO 3 with this velocity on it. topRollerVelocity
        // below is the same line. That sends a target, not a voltage; the controller picks the volts.
    }

    @Override
    public void indexerStop() {
        // TODO 8: one line. Stop commanding the motor at all, rather than commanding zero.
        // topRollerStop below is the same line. 
    }

    @Override
    public void topRollerVelocity(AngularVelocity velocity) {
        m_topRollerMotor.setControl(m_topRollerVelocityRequest.withVelocity(velocity));
    }

    @Override
    public void topRollerStop() {
        m_topRollerMotor.disable();
    }

    @Override
    public void lowerKickerVelocity(AngularVelocity velocity) {
        m_lowerKickerMotor.setControl(m_lowerKickerVelocityRequest.withVelocity(velocity));
    }

    @Override
    public void lowerKickerStop() {
        m_lowerKickerMotor.disable();
    }
}
