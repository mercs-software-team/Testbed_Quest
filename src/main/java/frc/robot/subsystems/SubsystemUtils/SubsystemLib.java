package frc.robot.subsystems.SubsystemUtils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public abstract class SubsystemLib extends SubsystemBase{
    protected boolean attached = false;
    protected TalonFX motor;
    public Config config;  

    public SubsystemLib(boolean attached) {
        this.attached = attached;
        this.config = setConfig(); 
    }

    protected abstract Config setConfig(); 

    protected void setConfig(Config config) { 
        this.config = config;
    };

    public void stop() {
        if (attached) {
            motor.stopMotor();
        }
    }

    public void tareMotor() {
        if (attached) {
            setMotorPosition(0);
        }
    }

    /**     
     * @param position 
     */
    public void setMotorPosition(double position) {
        if (attached) {
            motor.setPosition(position);
        }
    }


    /**
     * velocity control
     *
     * @param velocity
     */
    public void setVelocity(double velocity) {
        if (attached) {
            VelocityVoltage output = config.velocityControl.withVelocity(velocity);
            motor.setControl(output);
        }
    }


    /**
     * voltage control
     *
     * @param voltage
     */
    public void setVoltage(double voltage) {
        if (attached) {
            VoltageOut output = config.voltageControl.withOutput(voltage);
            // System.out.println(voltage);
            motor.setControl(output);
        }
    }

    /**
     * 
     * @param position
     * @param slot
     */

    public void SetPositionVoltage(double position, int slot){
    
        if (attached) {
            PositionVoltage output = config.positionVoltage.withSlot(slot).withPosition(position);
            motor.setControl(output);
        }
    }

    public double GetPosition() {
        if (attached) {
            return motor.getPosition().getValueAsDouble();
        } else {
            return 0.0;
        }
    }


    /**
     * Closed-loop Position Motion Magic with torque control (requires Pro)
     *
     * @param position rotations
     */
    public void setMMPositionFOC(double position) {
        if (attached) {
            MotionMagicTorqueCurrentFOC mm = config.mmPositionFOC.withPosition(position);
            motor.setControl(mm);
        }
    }

    /**
     * Closed-loop Position Motion Magic
     *
     * @param position rotations
     */
    public void setMMPosition(double position) {
        if (attached) {
            MotionMagicVoltage mm = config.mmPositionVoltage.withPosition(position);
            motor.setControl(mm);
        }
    }

    /**
     * Closed-loop Position Motion Magic
     *
     * @param position rotations
     */
    public void setMMPosition(DoubleSupplier position) {
        if (attached) {
            MotionMagicVoltage mm = config.mmPositionVoltage.withPosition(position.getAsDouble());
            motor.setControl(mm);
        }
    }

    /**
     * Closed-loop Position Motion Magic using a slot other than 0
     *
     * @param position rotations
     * @param slot gains slot
     */
    public void setMMPosition(double position, int slot) {
        if (attached) {
            MotionMagicVoltage mm =
                    config.mmPositionVoltageSlot.withSlot(slot).withPosition(position);
            motor.setControl(mm);
        }
    }

    /**
     * Open-loop Percent output control with voltage compensation
     *
     * @param percent fractional units between -1 and +1
     */
    public void setPercentOutput(double percent) {
        if (attached) {
            VoltageOut output =
                    config.voltageControl.withOutput(12 * percent);
            motor.setControl(output);
        }
    }

    public void setBrakeMode(boolean isInBrake) {
        if (attached) {
            config.configNeutralBrakeMode(isInBrake);
            config.applyTalonConfig(motor);
        }
    }

    /**
     * @param percent
     */
    public void setDutyCycleOut(double percent, int slot){
        if(attached){
            DutyCycleOut output = config.dutyCycleOut.withOutput(percent);
            motor.setControl(output);
        }
    }
    


    public static class Config {
        public String name;
        public CanDeviceId id;
        public TalonFXConfiguration talonConfig;
        // public double voltageCompSaturation; // 12V by default

        public MotionMagicVelocityTorqueCurrentFOC mmVelocityFOC = new MotionMagicVelocityTorqueCurrentFOC(0);
        public MotionMagicTorqueCurrentFOC mmPositionFOC = new MotionMagicTorqueCurrentFOC(0);
        public MotionMagicVelocityVoltage mmVelocityVoltage = new MotionMagicVelocityVoltage(0);
        public MotionMagicVoltage mmPositionVoltage = new MotionMagicVoltage(0);
        public MotionMagicVoltage mmPositionVoltageSlot = new MotionMagicVoltage(0).withSlot(1);
        public VoltageOut voltageControl = new VoltageOut(0);
        public VelocityVoltage velocityControl = new VelocityVoltage(0);
        public VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        public PositionVoltage positionVoltage = new PositionVoltage(0);
        public DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
        
        

        public Config(String name, int id, String canbus) {
            this.name = name;
            this.id = new CanDeviceId(id, canbus);
            talonConfig = new TalonFXConfiguration();

            /* Put default config settings for all mechanisms here */
            talonConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
            talonConfig.HardwareLimitSwitch.ReverseLimitEnable = false;
        }

        public void applyTalonConfig(TalonFX talon) {
            StatusCode result = talon.getConfigurator().apply(talonConfig);
            if (!result.isOK()) {
                DriverStation.reportWarning(
                        "Could not apply config changes to motor!", false); //Maybe this will help fix the chain problem?
            }
        }

        public void isClockwise(boolean clockwise) {  //ALWAYS test this! We do not want chains snapping. Run the motros before we put the mechanism on
            if(!clockwise){
                talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            }
            else {
                talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            }
        }

        
        public void configStatorCurrentLimit(double statorLimit, boolean enabled) {
            talonConfig.CurrentLimits.StatorCurrentLimit = statorLimit;
            talonConfig.CurrentLimits.StatorCurrentLimitEnable = enabled;
        }

        public void configFeedbackSensorSource(FeedbackSensorSourceValue source) {
            configFeedbackSensorSource(source, 0);
        }

        public void configFeedbackSensorID(int id){
            talonConfig.Feedback.FeedbackRemoteSensorID = id;
        }

        public void configFeedbackSensorSource(FeedbackSensorSourceValue source, double offset) {
            talonConfig.Feedback.FeedbackSensorSource = source;
            talonConfig.Feedback.FeedbackRotorOffset = offset;
        }



       
        public void configMotionMagicVelocity(double acceleration, double feedforward) {
            mmVelocityFOC = mmVelocityFOC.withAcceleration(acceleration).withFeedForward(feedforward);
            mmVelocityVoltage = mmVelocityVoltage.withAcceleration(acceleration).withFeedForward(feedforward);
        }

        public void configMotionMagicPosition(double feedforward) {
            mmPositionFOC = mmPositionFOC.withFeedForward(feedforward);
            mmPositionVoltage = mmPositionVoltage.withFeedForward(feedforward);
        }

        public void configMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
            talonConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
            talonConfig.MotionMagic.MotionMagicAcceleration = acceleration;
            talonConfig.MotionMagic.MotionMagicJerk = jerk;
        }

       
        public void configGearRatio(double gearRatio) {  //find the number of gearboxes here for an accurate number of shaft rotations
            talonConfig.Feedback.SensorToMechanismRatio = gearRatio;
        }

        public double getGearRatio() {
            return talonConfig.Feedback.SensorToMechanismRatio;
        }

        public void configNeutralBrakeMode(boolean isInBrake) { 
            if (isInBrake) {
                talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            } else {
                talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            }
        }
        
        public void configVirtualLimitSwitch(double fwdLim, double revLim, boolean enableFwd, boolean enableRev) {
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = fwdLim;
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = revLim;
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = enableFwd;
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = enableRev;
        }
        /**
         * Defaults to slot 0  //Some other teams have a complicated slot configuration but I just have this simple default until I learn more 
         *
         * @param kP 
         * @param kI
         * @param kD
         */
       
        public void configSlot0PIDGains(double kP, double kI, double kD) { //Super simple functions to set the PID gains. 
            configSlot0PID(kP, kI, kD);
        }

        /**
         * 
         * @param kP
         * @param kI
         * @param kD
         */

        public void configSlot1PIDGains(double kP, double kI, double kD) { //Super simple functions to set the PID gains. 
            configSlot1PID(kP, kI, kD);
        }

        /**
         * 
         * @param kP
         * @param kI
         * @param kD
         */
        public void configSlot2PIDGains(double kP, double kI, double kD) { //Super simple functions to set the PID gains. 
            configSlot2PID(kP, kI, kD);
        }


        
        private void configSlot0PID(double kP, double kI, double kD) {
            talonConfig.Slot0.kP = kP;
            talonConfig.Slot0.kI = kI;
            talonConfig.Slot0.kD = kD;
        }
        
        private void configSlot1PID(double kP, double kI, double kD) {
            talonConfig.Slot1.kP = kP;
            talonConfig.Slot1.kI = kI;
            talonConfig.Slot1.kD = kD;
        }
        
        private void configSlot2PID(double kP, double kI, double kD) {
            talonConfig.Slot2.kP = kP;
            talonConfig.Slot2.kI = kI;
            talonConfig.Slot2.kD = kD;
        }
        
        
        public void configslot0ForwardGains(double  kV, double kA, double kS, double kG) {
            talonConfig.Slot0.kV = kV;
            talonConfig.Slot0.kA = kA;
            talonConfig.Slot0.kS = kS;
            talonConfig.Slot0.kG = kG;
        }
        
        public void configslot1ForwardGains(double  kV, double kA, double kS, double kG) {
            talonConfig.Slot1.kV = kV;
            talonConfig.Slot1.kA = kA;
            talonConfig.Slot1.kS = kS;
            talonConfig.Slot1.kG = kG;
        }
    
        public void configslot2ForwardGains(double  kV, double kA, double kS, double kG) {
            talonConfig.Slot2.kV = kV;
            talonConfig.Slot2.kA = kA;
            talonConfig.Slot2.kS = kS;
            talonConfig.Slot2.kG = kG;
        }


        public void configSlot0ForwardGains(double kV, double kA, double kS, double kG){
            configslot0ForwardGains(kV, kA, kS, kG);
        }
        public void configSlot1ForwardGains(double kV, double kA, double kS, double kG){
            configslot1ForwardGains(kV, kA, kS, kG);
        }
        public void configSlot2ForwardGains(double kV, double kA, double kS, double kG){
            configslot2ForwardGains(kV, kA, kS, kG);
        }
        
    }
}


  






    

