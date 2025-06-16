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

    public static final class turretConstants{
        public static final int id = 16;

        public static final boolean attached = true;

        public static final double kP0 = 0.1; 
        public static final double kP1 = 0;  

        public static final double rightLimit = 10;
        public static final double leftLimit = -10;

    }

    public static final class turretMMConstants{
        public static final double acceleration = 10;
        public static final double speed = 10;
        public static final double jerk = 0;

    }


    public static final class hoodConstants{
        public static final int id = 0;

        public static final boolean attached = false;

        public static final double kP0 = 0; 
        public static final double kP1 = 0;  
        public static final double kP2 = 0;  


        public static final double topLimit = 10;
        public static final double bottomLimit = -10;

    }

    public static final class intakePivotConstants{
        public static final int id = 0;

        public static final boolean attached = false;

        public static final double kP0 = 0; 
        public static final double kP1 = 0;  
        public static final double kP2 = 0;  


        public static final double restPose = 0;
        public static final double intakingPose = 0;

    }


    public static final class intakeRollersConstants{
        public static final int id = 0;

        public static final boolean attached = false;

        public static final double kP0 = 0; 
        public static final double kP1 = 0;  
        public static final double kP2 = 0;  


        public static final double veloctiyIntaking = 0;
        public static final double velocityDescoring = 1;


    }

    public static final class flywheelsConstants{
        public static final int id = 0;

        public static final boolean attached = false;

        public static final double kP0 = 0; 
        public static final double kP1 = 0;  
        public static final double kP2 = 0;  


        public static final double velocity1 = 0;

    }

    public static final class indexerConstants{
        public static final int id = 0;

        public static final boolean attached = false;

        public static final double kP0 = 0; 
        public static final double kP1 = 0;  
        public static final double kP2 = 0;  


        public static final double velocity = 0;

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
