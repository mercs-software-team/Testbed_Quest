package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.turretTestConstants;
import frc.robot.subsystems.TurretTestSubsystem;

public class turretToPosSlot0 extends Command {
    private TurretTestSubsystem m_turret;
    private double tarPos;


    public turretToPosSlot0(TurretTestSubsystem m_turret, double tarPos) {
        this.tarPos = tarPos;
        this.m_turret= m_turret;
    }

    @Override 
    public void initialize() {
        // This is where you put stuff that happens right at the start of the command

                m_turret.turretToPosSlot0(tarPos);

        
    }
    @Override 
    public void execute() {
        // This is where you put stuff that happens while the command is happening, it will loop here over and over
        
    }

    @Override 
    public void end(boolean interrupted) {
        // This is where you put stuff that happens when the command ends
    }

    @Override 
    public boolean isFinished() {
        return true;
    }


}
