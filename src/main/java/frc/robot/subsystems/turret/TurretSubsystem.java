package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.turret.TurretConstants.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.state_machine.StateMachine;
import frc.robot.state_machine.TurretState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ConversionUtil;

public final class TurretSubsystem extends SubsystemBase {
    public static TurretSubsystem instance = null;

    private final TurretIO m_io;
    private final TurretIOInputsAutoLogged m_inputs = new TurretIOInputsAutoLogged();
    
    public TurretSubsystem(TurretIO io) {
        instance = this;

        m_io = io;
    }

    @AutoLogOutput(key="TurretDebugPose")
    private Pose3d m_debugPose = new Pose3d();

    public Pose3d getDebugPose() {
        return m_debugPose;
    }

    public Rotation3d getFieldRotation() {
        return m_debugPose.getRotation();
    }

    @AutoLogOutput(key="TurretShouldIndex")
    public boolean shouldIndex() {
        return Math.abs(m_inputs.targetPositionDegrees - m_inputs.positionDegrees) <= shouldIndexThresholdDegrees;
    }

    @AutoLogOutput(key="TurretDebugTargetFieldPose")
    private final Pose3d m_debugTargetFieldPose = new Pose3d(
        Units.inchesToMeters(182.11d),
        Units.inchesToMeters(158.84d),
        Units.feetToMeters(6d),
        new Rotation3d());

    private Pose3d m_targetPose = null;

    public boolean hasTarget() {
        return m_targetPose != null;
    }
    public Pose3d getTargetPose() {
        return m_targetPose;
    }

    @AutoLogOutput(key="TurretManualEnabled")
    private boolean m_manualEnabled = false;
    private Angle m_manualTarget;
    private DoubleSupplier m_manualSupplier = () -> 0d;

    // t2
    // private static final double lookAheadTime = 0.05d;

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);

        Logger.processInputs("Turret", m_inputs);

        Pose3d targetPose = null;
        if (m_manualEnabled) {
            m_manualTarget = m_manualTarget.plus(Degrees.of(m_manualSupplier.getAsDouble()));
            setAngle(m_manualTarget);
        } else {
            switch (StateMachine.getTurretState()) {
                case HubTracking:
                    targetPose = m_debugTargetFieldPose;
                    target3dPose(m_debugTargetFieldPose);
                    break;
                default:
                    break;
            }
        }

        m_targetPose = targetPose;

        m_debugPose = new Pose3d(
            0,
            0,
            Units.inchesToMeters(30d),
            new Rotation3d(
                0,
                0,
                Units.degreesToRadians(m_inputs.positionDegrees)));
    }

    public Command toggleManualModeCommand() {
        return runOnce(() -> {
            m_manualEnabled = !m_manualEnabled;
            if (m_manualEnabled)
                m_manualTarget = getAngle();
        });
    }

    public void setManualSupplier(DoubleSupplier supplier) {
        m_manualSupplier = supplier;
    }

    public Angle getAngle() {
        return Degrees.of(m_inputs.positionDegrees);
    }
    public void setAngle(Angle measurement) {
        m_io.setPosition(ConversionUtil.angleToMechanismPosition(measurement));
    }

    public void stateUpdate(TurretState turretState) {
        m_manualEnabled = false;
        switch (turretState) {
            case Idle:
                m_io.idle();
                break;
            case HubTracking:
                break;
            case AllianceFeed:
                break;
        }
    }

    private void target3dPose(Pose3d targetPose) {
        Pose2d robotPose = Drive.instance.getPose();
        final ChassisSpeeds robotSpeeds = Drive.instance.getChassisSpeeds();

        // t1
        // robotPose = robotPose.exp(new Twist2d(
        //     robotSpeeds.vxMetersPerSecond * lookAheadTime,
        //     robotSpeeds.vyMetersPerSecond * lookAheadTime,
        //     robotSpeeds.omegaRadiansPerSecond * lookAheadTime
        // ));

        // t0
        // Translation2d robotTranslation = robotPose.getTranslation();
        // Translation2d targetTranslation = targetPose.getTranslation().toTranslation2d();

        // Rotation2d targetRotation = targetTranslation.minus(robotTranslation).getAngle();
        // Rotation2d currentRotation = robotPose.getRotation();

        // Rotation2d rotationError = targetRotation.minus(currentRotation);

        // final double targetangle = rotationError.getRadians();

        // setAngle(Radians.of(targetangle));

        final Translation2d robotTranslation = robotPose.getTranslation();
        final Translation2d targetTranslation = targetPose.getTranslation().toTranslation2d();

        Translation2d yawTranslation = targetTranslation.minus(robotTranslation);
        // Translation2d yawTranslation = robotTranslation.minus(targetTranslation);
        yawTranslation = yawTranslation.minus(new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond));

        final Rotation2d currentRotation = robotPose.getRotation();
        final Rotation2d rotationError = yawTranslation.getAngle().minus(currentRotation);

        final double targetangle = rotationError.getRadians();

        setAngle(Radians.of(targetangle));
    }
}
