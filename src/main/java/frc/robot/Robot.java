// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;
import java.util.Set;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.StateMachine.RobotCommands;
import frc.robot.StateMachine.TurretState;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SimulationCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.ground_intake.GroundIntakeIO;
import frc.robot.subsystems.ground_intake.GroundIntakeIOReal;
import frc.robot.subsystems.ground_intake.GroundIntakeIOSim;
import frc.robot.subsystems.ground_intake.GroundIntakeSubsystem;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private CommandScheduler m_commandScheduler = null;
    private Command m_autoCommand = null;

    // subsystems
    private Drive m_driveSubsystem = null;
    private TurretSubsystem m_turretSubsystem = null;
    private GroundIntakeSubsystem m_intakeSubsystem = null;
    private IndexerSubsystem m_indexerSubsystem = null;
    private ShooterSubsystem m_shooterSubsystem = null;
    private ClimberSubsystem m_climberSubsystem = null;

    private final LoggedDashboardChooser<Command> m_autoChooser;

    public Robot() {
        SignalLogger.setPath("/U/logs");
        SignalLogger.start();

        m_commandScheduler = CommandScheduler.getInstance();

        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
              "GitDirty",
              switch (BuildConstants.DIRTY) {
                case 0 -> "All changes committed";
                case 1 -> "Uncommitted changes";
                default -> "Unknown";
              });

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());

                m_driveSubsystem = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));
                m_turretSubsystem = new TurretSubsystem(new TurretIOReal());
                m_intakeSubsystem = new GroundIntakeSubsystem(new GroundIntakeIOReal());
                m_indexerSubsystem = new IndexerSubsystem(new IndexerIOReal());
                m_shooterSubsystem = new ShooterSubsystem(new ShooterIOReal());
                m_climberSubsystem = new ClimberSubsystem(new ClimberIOReal());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());

                m_driveSubsystem = new Drive(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));
                m_turretSubsystem = new TurretSubsystem(new TurretIOSim());
                m_intakeSubsystem = new GroundIntakeSubsystem(new GroundIntakeIOSim());
                m_indexerSubsystem = new IndexerSubsystem(new IndexerIOSim());
                m_shooterSubsystem = new ShooterSubsystem(new ShooterIOSim());
                m_climberSubsystem = new ClimberSubsystem(new ClimberIOSim());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));

                m_driveSubsystem = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {});
                m_turretSubsystem = new TurretSubsystem(new TurretIO() {});
                m_intakeSubsystem = new GroundIntakeSubsystem(new GroundIntakeIO() {});
                m_indexerSubsystem = new IndexerSubsystem(new IndexerIO() {});
                m_shooterSubsystem = new ShooterSubsystem(new ShooterIO() {});
                m_climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
                break;
        }

        // Start AdvantageKit logger
        Logger.start();

        DriverStation.silenceJoystickConnectionWarning(true);
        m_commandScheduler.setPeriod(0.04d);

        StateMachine.updateState(null);

        m_autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        StateMachine.updateState(this);

        resetGyro(Rotation2d.kZero);
        configureButtons();
        setupNamedCommands();
        m_commandScheduler.onCommandInterrupt((Command command, Optional<Command> interruptor) -> {
            System.out.println(command.getName() + " was interrupted by " + (interruptor.isPresent() ? interruptor.get().getName() : "null"));
        });
    }

    private void configureButtons() {
        m_driveSubsystem.setDefaultCommand(
            DriveCommands.joystickDrive(
                m_driveSubsystem,
                () -> -Controls.controller.getLeftY(),
                () -> -Controls.controller.getLeftX(),
                () -> -Controls.controller.getRightX()));
        
        Controls.resetGyroButton.onTrue(Commands.runOnce(this::resetGyro));

        Controls.deployIntakeButton.onTrue(RobotCommands.deployIntake());
        Controls.retractIntakeButton.onTrue(RobotCommands.retractIntake());

        Controls.shootButton.onTrue(
            RobotCommands.engageShooterHub()
        );
        Controls.shootButton.onFalse(
            RobotCommands.disengageShooter()
        );
        if (Robot.isSimulation()) {
            // this is nonfunctional; ignore
            Controls.shootButton.whileTrue(Commands.repeatingSequence(
                new SimulationCommands.SimFuelCommand(
                    () -> new Pose3d(Drive.instance.getPose())
                        .plus(new Transform3d(0, 0, Units.feetToMeters(1.5d), Rotation3d.kZero)),
                    () -> {
                        // final Rotation3d fieldRotation = m_turretSubsystem.getFieldRotation();
                        // final Translation2d robotPose = m_driveSubsystem.getPose().minus(m_turretSubsystem.getDebugPose().toPose2d()).getTranslation();//.rotateBy(new Rotation2d(fieldRotation.getZ()));
                        // return new Twist3d(robotPose.getX(), robotPose.getY(), 6d, 0d, 0d, 0d);
                        return new Twist3d(0, 0, 6d, 0d, 0d, 0d);
                    }
                ),
                Commands.waitSeconds(0.002d)
            ));
        }

        Controls.climbForward.onTrue(Commands.either(
            RobotCommands.climbStateIncrease(),
            Commands.none(),
            Controls.climbSafetyButton
        ));
        Controls.climbBackward.onTrue(Commands.either(
            RobotCommands.climbStateDecrease(),
            Commands.none(),
            Controls.climbSafetyButton
        ));

        // debug
        var dbgController = new CommandXboxController(1);
        dbgController.rightStick().onTrue(m_turretSubsystem.toggleManualModeCommand());

        m_turretSubsystem.setManualSupplier(() -> {
            if (Math.abs(dbgController.getRawAxis(XboxController.Axis.kRightX.value)) >= 0.15)
                return dbgController.getRightX() * 2.5;
            return 0d;
        });
    }

    private void setupNamedCommands() {
        NamedCommands.registerCommand("EngageShooterHub", RobotCommands.engageShooterHub());
        NamedCommands.registerCommand("StopShooting", RobotCommands.disengageShooter());

        NamedCommands.registerCommand("ClimbStepNext", RobotCommands.climbStateIncrease());
        NamedCommands.registerCommand("ClimbStepPrevious", RobotCommands.climbStateDecrease());
    }

    public void resetGyro(Rotation2d offset) {
        if (StateMachine.initialSwerveRotation != null) {
            final Pose2d pose = new Pose2d(m_driveSubsystem.getPose().getTranslation(), StateMachine.initialSwerveRotation.plus(offset));
            m_driveSubsystem.setPose(pose);
        }
    }
    public void resetGyro() {
        resetGyro(Rotation2d.kPi);
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);

        m_commandScheduler.run();

        Threads.setCurrentThreadPriority(true, 10);

        StateMachine.updateState(this);
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {}

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        // ensure flywheel is spinning if we want it to be
        StateMachine.setShooterState(StateMachine.getShooterState());

        m_autoCommand = m_autoChooser.get();

        if (m_autoCommand != null)
            m_commandScheduler.schedule(m_autoCommand);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        if (m_autoCommand != null)
            m_commandScheduler.cancel(m_autoCommand);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        m_commandScheduler.cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
