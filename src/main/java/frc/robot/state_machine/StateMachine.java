package frc.robot.state_machine;

import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Robot;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.ground_intake.GroundIntakeSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.HubActiveState;

public class StateMachine {
    public static Alliance ALLIANCE = null;

    public static Distance allianceFlipX(Distance x) {
        return ALLIANCE == Alliance.Red ? FieldConstants.FIELD_LENGTH.minus(x) : x;
    }

    public static Distance allianceFlipY(Distance y) {
        return ALLIANCE == Alliance.Red ? FieldConstants.FIELD_WIDTH.minus(y) : y;
    }

    public static Translation2d allianceFlip(Translation2d translation) {
        return new Translation2d(allianceFlipX(translation.getMeasureX()), allianceFlipY(translation.getMeasureY()));
    }
    
    public static Rotation2d allianceFlip(Rotation2d rotation) {
        return ALLIANCE == Alliance.Red ? rotation.rotateBy(Rotation2d.kPi) : rotation;
    }

    public static Pose2d allianceFlip(Pose2d pose) {
        return ALLIANCE == Alliance.Red
            ? new Pose2d(allianceFlip(pose.getTranslation()), allianceFlip(pose.getRotation()))
            : pose;
    }

    public static Translation3d allianceFlip(Translation3d translation) {
        return new Translation3d(
            allianceFlipX(translation.getMeasureX()), allianceFlipY(translation.getMeasureY()), translation.getMeasureZ());
    }

    public static Rotation3d allianceFlip(Rotation3d rotation) {
        return ALLIANCE == Alliance.Red ? rotation.rotateBy(new Rotation3d(0d, 0d, Math.PI)) : rotation;
    }

    public static Pose3d allianceFlip(Pose3d pose) {
        return new Pose3d(allianceFlip(pose.getTranslation()), allianceFlip(pose.getRotation()));
    }

    private static Optional<Pose3d> shooterTargetPose = Optional.empty();
    public static Optional<Pose3d> getShooterTargetPose() {
        return shooterTargetPose;
    }

    private static IntakeState intakeState = IntakeState.HomeOff;

    public static IntakeState getIntakeState() {
        return intakeState;
    }
    public static void setIntakeState(IntakeState newIntakeState) {
        intakeState = newIntakeState;
        GroundIntakeSubsystem.instance.stateUpdate(intakeState);
        // IndexerSubsystem.instance.intakeStateUpdate(intakeState);
    }

    private static ShooterTargetState shooterTargetState = ShooterTargetState.Idle;

    public static ShooterTargetState getShooterTargetState() {
        return shooterTargetState;
    }
    public static void setShooterTargetState(ShooterTargetState newTurretState) {
        shooterTargetState = newTurretState;
        if (Constants.USE_TURRET)
            TurretSubsystem.instance.stateUpdate(shooterTargetState);
    }

    private static boolean withinJoystickRotationErrorThreshold = false;
    public static boolean getWithinJoystickRotationErrorThreshold() {
        return withinJoystickRotationErrorThreshold;
    }
    public static void setWithinJoystickRotationErrorThreshold(boolean within) {
        withinJoystickRotationErrorThreshold = within;
    }

    public static boolean turretShouldIndex() {
        return Constants.USE_TURRET ? TurretSubsystem.instance.shouldIndex() : withinJoystickRotationErrorThreshold;
    }
    public static boolean shouldIndex() {
        return turretShouldIndex() && ShooterSubsystem.instance.shouldIndex();
    }

    private static ShooterState shooterState = ShooterState.Idle;

    public static ShooterState getShooterState() {
        return shooterState;
    }
    public static void setShooterState(ShooterState newShooterState) {
        shooterState = newShooterState;
        ShooterSubsystem.instance.stateUpdate(newShooterState);
    }

    private static ClimberState climberState = ClimberState.Idle;

    public static ClimberState getClimberState() {
        return climberState;
    }
    public static void setClimberState(ClimberState newClimberState) {
        climberState = newClimberState;
        ClimberSubsystem.instance.stateUpdate(climberState);
    }

    public static final class RobotCommands {
        // intake
        public static Command setIntakeState(IntakeState intakeState) {
            return GroundIntakeSubsystem.instance.runOnce(
                () -> StateMachine.setIntakeState(intakeState)
            );
        }

        public static Command deployIntake() {
            return setIntakeState(IntakeState.DeployedOn);
        }
        public static Command retractIntake() {
            return setIntakeState(IntakeState.HomeOff);
        }

        // turret
        public static Command setShooterTargetState(ShooterTargetState turretState) {
            Command command = Commands.runOnce(() -> StateMachine.setShooterTargetState(turretState));
            if (Constants.USE_TURRET)
                command.addRequirements(TurretSubsystem.instance);
            return command;
        }

        public static Command setShooterTargetIdle() {
            return setShooterTargetState(ShooterTargetState.Idle);
        }
        public static Command setShooterTargetHubTracking() {
            return setShooterTargetState(ShooterTargetState.HubTracking);
        }
        public static Command setShooterTargetAllianceFeed() {
            return setShooterTargetState(ShooterTargetState.AllianceFeed);
        }

        // shooter
        public static Command setShooterState(ShooterState shooterState) {
            return ShooterSubsystem.instance.runOnce(
                () -> StateMachine.setShooterState(shooterState)
            );
        }

        public static Command setShooterIdle() {
            return setShooterState(ShooterState.Idle);
        }
        public static Command setShooterShooting() {
            return setShooterState(ShooterState.Shooting);
        }

        // shooter + turret
        public static Command engageShooterHub() {
            return setShooterTargetHubTracking()
                .andThen(Commands.waitUntil(StateMachine::turretShouldIndex))
                .andThen(setShooterShooting());
        }
        public static Command disengageShooter() {
            return setShooterTargetIdle()
                .alongWith(setShooterIdle());
        }

        // climber
        public static Command setClimberState(ClimberState climberState) {
            return ClimberSubsystem.instance.runOnce(
                () -> StateMachine.setClimberState(climberState)
            );
        }

        public static Command climberStateIncrease() {
            return Commands.defer(
                () -> setClimberState(getClimberState().getNext()),
                Set.of(ClimberSubsystem.instance)
            );
        }
        public static Command climberStateDecrease() {
            return Commands.defer(
                () -> setClimberState(getClimberState().getPrevious()),
                Set.of(ClimberSubsystem.instance)
            );
        }

        // FIXME: auto climb command
        public static Command autoClimb() {
            return Commands.none();
        }
    }

    private static Rotation2d swerveRotationOffset = new Rotation2d();

    public static Rotation2d getSwerveRotationOffset() {
        return swerveRotationOffset;
    }

    public static Rotation2d initialSwerveRotation = null;

    public static Angle getRobotRotationalGoal(Pose3d targetPose) {
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

        // return Radians.of(targetangle);

        final Translation2d robotTranslation = robotPose.getTranslation();
        final Translation2d targetTranslation = targetPose.getTranslation().toTranslation2d();

        Translation2d yawTranslation = targetTranslation.minus(robotTranslation);
        // Translation2d yawTranslation = robotTranslation.minus(targetTranslation);
        yawTranslation = yawTranslation.minus(new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond));

        final Rotation2d currentRotation = robotPose.getRotation();
        final Rotation2d rotationError = yawTranslation.getAngle().minus(currentRotation);

        final double targetangle = rotationError.getRadians();

        return Radians.of(targetangle);
    }

    private static final Pose3d hubPoseBlue = new Pose3d(
        Units.inchesToMeters(182.11d),
        Units.inchesToMeters(158.84d),
        Units.feetToMeters(6d),
        new Rotation3d());
    private static Pose3d hubPose = new Pose3d();

    private static boolean needsToUpdateRobot = true;
    @SuppressWarnings("unused") // for useTurret
    public static synchronized void periodic(Robot robot) {
        Logger.recordOutput("StateMachine/IntakeState", intakeState);
        Logger.recordOutput("StateMachine/ShooterTargetState", shooterTargetState);
        Logger.recordOutput("StateMachine/ShooterState", shooterState);
        Logger.recordOutput("StateMachine/ClimberState", climberState);

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() != ALLIANCE) {
            needsToUpdateRobot = true;

            ALLIANCE = alliance.get();
            Logger.recordOutput("StateMachine/Alliance", ALLIANCE);

            // AprilTagConstants.update(ALLIANCE);

            if (ALLIANCE == Alliance.Blue)
                initialSwerveRotation = Rotation2d.kZero;
            else
                initialSwerveRotation = Rotation2d.kPi;

            swerveRotationOffset = initialSwerveRotation.plus(Rotation2d.kPi);

            // VisionSubsystem.update();
            // for (var value : VisionSubsystem.RelativeReefLocation.values())
            //     value.update();
            // for (var value : VisionSubsystem.CoralStationID.values())
            //     value.update();

            hubPose = allianceFlip(hubPoseBlue);
        }

        if (needsToUpdateRobot && robot != null && ALLIANCE != null) {
            needsToUpdateRobot = false;

            // if (m_visionSubsystem != null && ALLIANCE != null)
            //     VisionSubsystem.RelativeReefLocation.postUpdate(m_visionSubsystem);
        }

        if (Constants.USE_TURRET && TurretSubsystem.instance.getManualEnabled())
            shooterTargetPose = Optional.empty();
        else {
            switch (StateMachine.getShooterTargetState()) {
                case HubTracking:
                    shooterTargetPose = Optional.of(hubPose);
                    break;
                default:
                    shooterTargetPose = Optional.empty();
                    break;
            }
        }

        Logger.recordOutput("StateMachine/ShooterTargetPose", shooterTargetPose.orElse(new Pose3d()));

        HubActiveState.periodic();

        Logger.recordOutput("StateMachine/HubActive", HubActiveState.isOurHubActive());
        Logger.recordOutput("StateMachine/TimeUntilSwap", HubActiveState.timeUntilSwap());
    }

}
