package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.PoseEstimate;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class Constants {

    public static final class turretTestConstants{
        public static final int id = 16;

        public static final boolean attached = true;

        public static final double kP0 = 0.1; 
        public static final double kP1 = 10; 

        public static final double kS = 0; 
        public static final double kV = 0; 
        public static final double voltageOut = 0;
        public static final double tol = 0.4;
        public static final double rightLimit = 10;
        public static final double leftLimit = -10;

    }

    public static final class turretMMConstants{
        public static final double acceleration = 10;
        public static final double speed = 10;
        public static final double jerk = 0;

    }


    public static final class VisionConstants {
        public static final String limelightFrontName = "limelight-front";
        public static final Vector<N3> visionStdDevs = VecBuilder.fill(.5,.5,9999999);
        public static PoseEstimate bestLimelightPose;
        public static boolean doTargetTracking = false;
    }

    public static boolean isWithinTol(double targetPose, double currentPose, double tolerance) {
        return (Math.abs(targetPose - currentPose) <= tolerance);
    }
    
}
