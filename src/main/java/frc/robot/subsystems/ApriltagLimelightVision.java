package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.SubsystemUtils.SubsystemLib;
import com.ctre.phoenix6.Utils;


public class ApriltagLimelightVision {
    private HashMap<Double, PoseEstimate> poseEstimates = new HashMap<>();
    private ArrayList<Double> avgAreas = new ArrayList<>();
    private PoseEstimate bestEstimate = null;

    private RobotContainer m_robotContainer;
    
    




    public ApriltagLimelightVision(RobotContainer robotContainer) {
        this.m_robotContainer = robotContainer;
    }



    public void updateVisionEstimates(Rotation2d orientation) {
        poseEstimates.clear();
        avgAreas.clear();

        LimelightHelpers.SetRobotOrientation(Constants.VisionConstants.limelightFrontName, orientation.getDegrees(), 0, 0, 0, 0, 0);

        PoseEstimate front;
     
            front = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.VisionConstants.limelightFrontName);
       

        if (front != null && front.tagCount > 0) {
            poseEstimates.put(front.avgTagArea, front);
        }
     

        avgAreas.addAll(poseEstimates.keySet());
        avgAreas.sort(null);


    


       

        
        if (!avgAreas.isEmpty()) {
            bestEstimate = poseEstimates.get(avgAreas.get(avgAreas.size() - 1));
        }




        

        if (bestEstimate != null && bestEstimate.tagCount > 0) {
            Constants.VisionConstants.bestLimelightPose = bestEstimate;
        } else {

            bestEstimate = null;
        }
    }

    public void applyVisionToEstimator() {
        if (bestEstimate != null && Math.abs(m_robotContainer.drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) <= 720) {

            m_robotContainer.drivetrain.setVisionMeasurementStdDevs(Constants.VisionConstants.visionStdDevs); //change this to rely on vision more in the future
            m_robotContainer.drivetrain.addVisionMeasurement(bestEstimate.pose, Utils.fpgaToCurrentTime(bestEstimate.timestampSeconds));
        }

        
    }

    public void resetPoseFromVision(Rotation2d rot) {
        if (bestEstimate != null && bestEstimate.tagCount > 0) {
            m_robotContainer.drivetrain.resetPose(new Pose2d(bestEstimate.pose.getTranslation(), rot));
        }
    }

    public Rotation2d estimateRotationFromVision() {
        updateVisionEstimates(m_robotContainer.drivetrain.getState().Pose.getRotation());
        return (bestEstimate != null) ? bestEstimate.pose.getRotation() : new Rotation2d();
    }

    // public void updateAutoTargetFromVision() {
    //     if (bestEstimate == null) return;

    //     RawFiducial closest = null;
    //     for (RawFiducial tag : bestEstimate.rawFiducials) {
    //         if (closest == null || tag.distToRobot < closest.distToRobot) {
    //             closest = tag;
    //         }
    //     }

    //     if (closest != null) {
    //         Constants.DriveToPosRuntime.autoTargets = Constants.DriveToPoseConstants.tagDestinationMap.getOrDefault(Integer.toString(closest.id), new ArrayList<>());
    //         SmartDashboard.putNumber("frontClosestTag", closest.id);
    //     }
    // }

    

   
    

    public void periodic() {
        // Optional: update dashboard or logs
        if (bestEstimate != null) {
            SmartDashboard.putString("Vision Pose", bestEstimate.pose.toString());
        }
    }
}