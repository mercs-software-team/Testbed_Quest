package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import frc.robot.subsystems.SubsystemUtils.SubsystemLib;
import frc.robot.subsystems.SubsystemUtils.TalonFXFactory;
import frc.robot.Constants.turretTestConstants;
import frc.robot.Constants.turretMMConstants;

public class TurretTestSubsystem extends SubsystemLib {
    public class TestSubsystemConfig extends Config {
     

        public final double velocityKp = turretTestConstants.kP;
        public final double velocityKs = turretTestConstants.kS;
        public final double velocityKv = 0;

        public TestSubsystemConfig() {
            super("ELevatorMotor1", turretTestConstants.id, "rio");  //It is on rio, but make sure that you change the id
            configPIDGains(velocityKp, 0, 0);
            configForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1);
            configNeutralBrakeMode(true);
           
            isClockwise(false); //true if you want it to spin clockwise
            configMotionMagic(turretMMConstants.speed, turretMMConstants.acceleration, turretMMConstants.jerk);
     
        }
    }





    public TestSubsystemConfig config;





    // }
    public TurretTestSubsystem(boolean attached){
        super(attached);
        if(attached){
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);
        }

    }

    public void motorToPosMM(double pos) {
        setMMPosition(pos);
    }

    public void turretToPos(double pos){
        SetPositionVoltage(pos);
    }

    public double turretGetPosition() {
        return GetPosition();
    }

    public void setTurretPower(double power) {
        power = Math.max(-1.0, Math.min(1.0, power));
        motor.setControl(new DutyCycleOut(power));
   }
    
   

    @Override
    protected Config setConfig() {
        config = new TestSubsystemConfig();
        return config;
    }


    @Override 
    public void periodic(){

        SmartDashboard.putNumber("turret Pos", motor.getPosition().getValueAsDouble());

    }

    

}