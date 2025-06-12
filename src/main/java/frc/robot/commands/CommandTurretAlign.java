package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretTestSubsystem;
import frc.robot.LimelightHelpers;


public class CommandTurretAlign extends Command {

   
    private TurretTestSubsystem turret;

    private final ProfiledPIDController thetaController =
    new ProfiledPIDController(8, 0.01, 0.01, new TrapezoidProfile.Constraints(Math.PI, Math.PI));
  
    public CommandTurretAlign (TurretTestSubsystem m_turret) {
        this.turret = m_turret;
        

        thetaController.setTolerance(Units.degreesToRadians(1));
        
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SmartDashboard.putData("aligningTheta_Pid", thetaController);

        SmartDashboard.putNumber("Theta_Tgt", -1);

    }

    @Override
    public void initialize() {
        var currPose = turret.turretGetPosition();
        thetaController.reset(currPose * (Math.PI * 2));
    }

    @Override
    public void execute() {
        double dval;
        dval =  LimelightHelpers.getTX("limelight-front");
        
        // targetingAngularVelocity *= MaxAngularRate;

        double currentYaw = turret.turretGetPosition() * (Math.PI *2);
        double targetYaw = currentYaw + Units.degreesToRadians(dval);
        double thetaVelocity = thetaController.calculate(currentYaw, targetYaw);


        SmartDashboard.putNumber("targetTheta", targetYaw);
        SmartDashboard.putNumber("thetaTargetvelocity", thetaVelocity);
        turret.setTurretPower(thetaVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        turret.turretToPos(0);

    }

    @Override
    public boolean isFinished() {
        return (LimelightHelpers.getTargetCount(Constants.VisionConstants.limelightFrontName) == 0);
    }
}
