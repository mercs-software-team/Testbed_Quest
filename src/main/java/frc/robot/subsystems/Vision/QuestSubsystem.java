package frc.robot.subsystems.Vision;
// package frc.robot.subsystems;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.QuestNav;

// public class QuestSubsystem extends SubsystemBase {
//     private final QuestNav questNav = new QuestNav();

//     // Define the transformation from the Quest headset to the robot center.
//     // Replace these with your actual offsets (x, y, rotation)
//     private static final Transform2d QUEST_TO_ROBOT = new Transform2d(
//        new edu.wpi.first.math.geometry.Translation2d(0.0, 0.0),
//        new Rotation2d(0.0)
//     );

//     public QuestSubsystem() {}

//     @Override
//     public void periodic() {
//         questNav.cleanupResponses();
//         questNav.processHeartbeat();
//     }

//     /**
//      * Retrieves the robot’s estimated pose by transforming the Quest’s pose.
//      * If QuestNav isn’t connected or isn’t tracking, a default pose is returned.
//      *
//      * @return The robot’s estimated Pose2d.
//      */
//     public Pose2d getRobotPose() {
//         if (questNav.getConnected() && questNav.getTrackingStatus()) {
//             Pose2d questPose = questNav.getPose();
//             // Transform the Quest pose to get the robot-relative pose.
//             return questPose.transformBy(QUEST_TO_ROBOT.inverse());
//         } else {
//             return new Pose2d(); // Default pose if tracking is lost
//         }
//     }

//     /**
//      * Resets the QuestNav pose based on a known field-relative robot pose.
//      * @param robotPose The field-relative pose for resetting.
//      */
//     public void resetPose(Pose2d robotPose) {
//         Pose2d questPose = robotPose.transformBy(QUEST_TO_ROBOT);
//         questNav.setPose(questPose);
//     }
    
//     /**
//      * Returns the Quest’s latest timestamp from NetworkTables.
//      *
//      * @return Timestamp in seconds.
//      */
//     public double getQuestTimestamp() {
//         return questNav.getTimestamp();
//     }
// }
