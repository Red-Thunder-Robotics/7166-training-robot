package frc.robot.subsystems.climbermark2;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.climbermark2.ClimberConstants.*;
import static frc.robot.util.ConversionUtil.mechanismPositionToAngle;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public final class ClimberIOReal implements ClimberIO {
    private final TalonFX m_actuatorMotor = new TalonFX(actuatorMotorId, Constants.CANBUS);
    private final TalonFX m_grabLeaderMotor = new TalonFX(grabLeaderId, Constants.CANBUS);
    private final TalonFX m_grabFollowerMotor = new TalonFX(grabFollowerId, Constants.CANBUS);

    private double m_actuatorTargetPosition = actuatorPositionHome;
    private double m_grabTargetPosition = grabPositionHome;

    private final StatusSignal<Angle> m_actuatorPositionSignal = m_actuatorMotor.getPosition();
    private final StatusSignal<AngularVelocity> m_actuatorVelocitySignal = m_actuatorMotor.getVelocity();
    private final StatusSignal<Current> m_actuatorCurrentSignal = m_actuatorMotor.getSupplyCurrent();
    
    private final StatusSignal<Angle> m_grabLeaderPositionSignal = m_grabLeaderMotor.getPosition();
    private final StatusSignal<AngularVelocity> m_grabLeaderVelocitySignal = m_grabLeaderMotor.getVelocity();
    private final StatusSignal<Current> m_grabLeaderCurrentSignal = m_grabLeaderMotor.getSupplyCurrent();
    
    private final StatusSignal<Angle> m_grabFollowerPositionSignal = m_grabFollowerMotor.getPosition();
    private final StatusSignal<AngularVelocity> m_grabFollowerVelocitySignal = m_grabFollowerMotor.getVelocity();
    private final StatusSignal<Current> m_grabFollowerCurrentSignal = m_grabFollowerMotor.getSupplyCurrent();

    private final MotionMagicVoltage m_actuatorPositionRequest = new MotionMagicVoltage(m_actuatorTargetPosition);
    private final MotionMagicVoltage m_grabPositionRequest = new MotionMagicVoltage(m_grabTargetPosition);

    public ClimberIOReal() {
        var actuatorConfig = new TalonFXConfiguration();
        actuatorConfig.MotorOutput.NeutralMode = actuatorNeutralMode;
        actuatorConfig.MotorOutput.Inverted = actuatorInverted;

        actuatorConfig.Feedback.SensorToMechanismRatio = actuatorMotorReduction;

        actuatorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        actuatorConfig.Slot0.kP = actuatorPidP;

        actuatorConfig.MotionMagic.MotionMagicAcceleration = actuatorTargetAcceleration;
        actuatorConfig.MotionMagic.MotionMagicCruiseVelocity = actuatorMaxVelocity;

        var grabConfig = new TalonFXConfiguration();
        grabConfig.MotorOutput.NeutralMode = grabLeaderNeutralMode;
        grabConfig.MotorOutput.Inverted = grabLeaderInverted;

        grabConfig.Feedback.SensorToMechanismRatio = grabLeaderMotorReduction;

        grabConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        grabConfig.Slot0.kP = grabPidP;

        PhoenixUtil.tryUntilOk(5, () -> m_actuatorMotor.getConfigurator().apply(actuatorConfig));
        PhoenixUtil.tryUntilOk(5, () -> m_grabLeaderMotor.getConfigurator().apply(grabConfig));
        PhoenixUtil.tryUntilOk(5, () -> m_grabFollowerMotor.getConfigurator().apply(grabConfig));

        BaseStatusSignal.setUpdateFrequencyForAll(50d, m_actuatorPositionSignal, m_actuatorVelocitySignal, m_actuatorCurrentSignal, m_grabLeaderPositionSignal, m_grabLeaderVelocitySignal, m_grabLeaderCurrentSignal, m_grabFollowerPositionSignal, m_grabFollowerVelocitySignal, m_grabFollowerCurrentSignal);

        m_grabFollowerMotor.setControl(new Follower(grabLeaderId, grabFollowerMatchLeaderInverted ? MotorAlignmentValue.Aligned : MotorAlignmentValue.Opposed));

        m_actuatorMotor.setPosition(m_actuatorTargetPosition);
        m_grabLeaderMotor.setPosition(m_grabTargetPosition);
        m_grabFollowerMotor.setPosition(m_grabTargetPosition);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        // actuator
        final double actuatorTargetPosition = m_actuatorTargetPosition;
        inputs.actuatorTargetPositionRotations = actuatorTargetPosition;
        inputs.actuatorTargetPositionDegrees = mechanismPositionToAngle(actuatorTargetPosition).in(Degrees);

        final double actuatorPositionRotations = m_actuatorPositionSignal.getValueAsDouble();
        inputs.actuatorPositionRotations = actuatorPositionRotations;
        inputs.actuatorPositionDegrees = mechanismPositionToAngle(actuatorPositionRotations).in(Degrees);

        inputs.actuatorVelocityRPS = m_actuatorVelocitySignal.getValueAsDouble();
        inputs.actuatorCurrentAmps = m_actuatorCurrentSignal.getValueAsDouble();

        // grab
        final double grabTargetPosition = m_grabTargetPosition;
        inputs.grabTargetPositionRotations = grabTargetPosition;
        inputs.grabTargetPositionDegrees = mechanismPositionToAngle(grabTargetPosition).in(Degrees);

        // grab leader
        final double grabLeaderPositionRotations = m_grabLeaderPositionSignal.getValueAsDouble();
        inputs.grabLeaderPositionRotations = grabLeaderPositionRotations;
        inputs.grabLeaderPositionDegrees = mechanismPositionToAngle(grabLeaderPositionRotations).in(Degrees);

        inputs.grabLeaderVelocityRPS = m_grabLeaderVelocitySignal.getValueAsDouble();
        inputs.grabLeaderCurrentAmps = m_grabLeaderCurrentSignal.getValueAsDouble();

        // grab follower
        final double grabFollowerPositionRotations = m_grabFollowerPositionSignal.getValueAsDouble();
        inputs.grabFollowerPositionRotations = grabFollowerPositionRotations;
        inputs.grabFollowerPositionDegrees = mechanismPositionToAngle(grabFollowerPositionRotations).in(Degrees);

        inputs.grabFollowerVelocityRPS = m_grabFollowerVelocitySignal.getValueAsDouble();
        inputs.grabFollowerCurrentAmps = m_grabFollowerCurrentSignal.getValueAsDouble();
    }

    @Override
    public void idle() {
        m_actuatorMotor.disable();
        m_grabLeaderMotor.disable();
    }

    @Override
    public void setActuatorPosition(double position) {
        m_actuatorTargetPosition = position;
        m_actuatorMotor.setControl(m_actuatorPositionRequest.withPosition(position));
    }
    @Override
    public void setGrabPosition(double position) {
        m_grabTargetPosition = position;
        m_grabLeaderMotor.setControl(m_grabPositionRequest.withPosition(position));
    }
}
