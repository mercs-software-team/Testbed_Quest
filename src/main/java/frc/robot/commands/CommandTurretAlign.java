package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.TurretTestSubsystem;

public class CommandTurretAlign extends Command {

    private TurretTestSubsystem turret;


    private final PIDController thetaController = new PIDController(0.3, 0.01, 0.01);

    public CommandTurretAlign(TurretTestSubsystem m_turret) {
        this.turret = m_turret;

        thetaController.setTolerance(Units.degreesToRadians(1));

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SmartDashboard.putData("aligningTheta_Pid", thetaController);

        SmartDashboard.putNumber("Theta_Tgt", -1);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double dval;
        dval = LimelightHelpers.getTX("limelight-front");

        // targetingAngularVelocity *= MaxAngularRate;

        double currentYaw = turret.turretGetPosition() * (Math.PI * 2);
        double targetYaw = currentYaw + Units.degreesToRadians(dval);
        double thetaVelocity = thetaController.calculate(currentYaw, targetYaw);

        SmartDashboard.putNumber("targetYaw", targetYaw);
        SmartDashboard.putNumber("currentYaw", currentYaw);
        SmartDashboard.putNumber("thetaVelocity", thetaVelocity);
        
          
        if (LimelightHelpers.getTargetCount(Constants.VisionConstants.limelightFrontName) > 0 ) {
            turret.setTurretPower(thetaVelocity);
        } else {
            turret.turretStop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.turretStop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
