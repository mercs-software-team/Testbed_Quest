package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import frc.robot.subsystems.SubsystemUtils.SubsystemLib;
import frc.robot.subsystems.SubsystemUtils.TalonFXFactory;
import frc.robot.Constants.turretTestConstants;
import frc.robot.Constants;
import frc.robot.Constants.turretMMConstants;
import frc.robot.Constants.*;

public class TurretTestSubsystem extends SubsystemLib {
    public class TestSubsystemConfig extends Config {
     

        public final double velocityKp0 = turretTestConstants.kP0;
        public final double velocityKp1 = turretTestConstants.kP1;

        public final double velocityKs = turretTestConstants.kS;
        public final double velocityKv = 0;

        public TestSubsystemConfig() {
            super("ELevator", turretTestConstants.id, "rio");  //It is on rio, but make sure that you change the id
            configSlot0PIDGains(velocityKp0, 0, 0);
            configSlot1PIDGains(velocityKp1, 0, 0);
            configslot0ForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1);
            configNeutralBrakeMode(true);
           
            isClockwise(false); //true if you want it to spin clockwise
            configMotionMagic(turretMMConstants.speed, turretMMConstants.acceleration, turretMMConstants.jerk);
            configVirtualLimitSwitch(20, -20, true, true);
     
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

    public void turretToPosSlot0(double pos){
        motor.setControl(new PositionVoltage(pos).withSlot(0));
    }

    public void turretToPosSlot1(double pos){
        motor.setControl(new PositionVoltage(pos).withSlot(1));
        }



    public void turretToPosMM(double pos){
        motor.setControl(new MotionMagicDutyCycle(0));
    }

    public void turretStop(){
        // motor.setControl(new MotionMagicDutyCycle(0).withSlot(1));
        motor.setControl(new NeutralOut());
    }

    public double turretGetPosition() {
        return GetPosition();
    }

    public void setTurretPower(double power) {
        power = Math.max(-1.0, Math.min(1.0, power));
        SmartDashboard.putNumber("turrPow", power);
        setDutyCycleOut(power, 0);
   }


   public boolean isWithinTurretTol(){
    if (Constants.isWithinTol(Constants.turretTestConstants.rightLimit, Math.abs(motor.getPosition().getValueAsDouble()), 1)){
        return false;
    }
    
    return true;
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