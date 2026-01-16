package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;

public final class VisionSubsystem extends SubsystemBase {
    static {
        for (int port = 5800; port <= 5809; port++)
            PortForwarder.add(port, limelightFrontName + ".local", port);

        for (int port = 5800; port <= 5809; port++)
            PortForwarder.add(port + 10, limelightBackName + ".local", port);
    }

    public VisionSubsystem() {
        
    }

    @Override
    public void periodic() {
        boolean megaTagSuccess = updateVisionMegaTag2(limelightFrontName);
        {
            var success = updateVisionMegaTag2(limelightBackName);
            megaTagSuccess |= success;
        }
    }

    private final Matrix<N3, N1> stdDevs = VecBuilder.fill(.7,.7,9999999);
    private boolean updateVisionMegaTag2(String limelightName) {
        final Drive drive = Drive.instance;

        if(Math.abs(drive.getYawVelocityRadPerSec()) > (4 * Math.PI))
            return false;

        final double yaw_degrees = drive.getRotation().getDegrees();

        LimelightHelpers.SetRobotOrientation(limelightName, yaw_degrees, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (mt2 == null)
            return false;

        if(mt2.tagCount == 0)
            return false;

        drive.addVisionMeasurement(
            mt2.pose, mt2.timestampSeconds,
            stdDevs
        );

        return true;
    }

}
