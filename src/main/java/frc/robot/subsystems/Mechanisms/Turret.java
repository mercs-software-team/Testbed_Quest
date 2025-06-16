package frc.robot.subsystems.Mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import frc.robot.subsystems.SubsystemUtils.SubsystemLib;
import frc.robot.subsystems.SubsystemUtils.TalonFXFactory;
import frc.robot.Constants.turretConstants;
import frc.robot.Constants.turretMMConstants;

public class Turret extends SubsystemLib {
    public class TestSubsystemConfig extends Config {
     

        public final double velocityKp0 = turretConstants.kP0;
        public final double velocityKp1 = turretConstants.kP1;

        public final double velocityKv = 0;

        public TestSubsystemConfig() {
            super("turret", turretConstants.id, "rio");  //It is on rio, but make sure that you change the id
            configSlot0PIDGains(velocityKp0, 0, 0);
            configSlot1PIDGains(velocityKp1, 0, 0);
            configGearRatio(1);
            configNeutralBrakeMode(true);
            isClockwise(false); //true if you want it to spin clockwise
            configMotionMagic(turretMMConstants.speed, turretMMConstants.acceleration, turretMMConstants.jerk);
            configVirtualLimitSwitch(20, -20, true, true);
     
        }
    }


    public TestSubsystemConfig config;


    public Turret(boolean attached){
        super(attached);
        if(attached){
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);
        }

    }


    public void turretStop(){
        setBrakeMode(attached);
    }

    public double turretGetPosition() {
        return GetPosition();
    }

    public void setTurretPower(double power) {
        power = Math.max(-1.0, Math.min(1.0, power));
        SmartDashboard.putNumber("turrPow", power);
        setDutyCycleOut(power, 0);
   }

    @Override
    protected Config setConfig() {
        config = new TestSubsystemConfig();
        return config;
    }


    @Override 
    public void periodic(){

        SmartDashboard.putNumber("turret Pos", GetPosition());

    }

    

}