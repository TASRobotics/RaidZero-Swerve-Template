package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    // Initialize rotor & throttle motors 
    private CANSparkMax mRotor;
    private CANSparkMax mThrottle;

    // Initialize throttle encoder
    private RelativeEncoder mThrottleEncoder;

    // Initialize rotor encoder
    private WPI_CANCoder mRotorEncoder; 

    // Initialize rotor PID controller
    private PIDController mRotorPID; 

    /**
     * Construct new SwerveModule
     * 
     * @param throttleID CAN ID of throttle motor
     * @param rotorID CAN ID of rotor motor
     * @param rotorEncoderID CAN ID of rotor encoder
     * @param rotorOffsetAngleDeg rotor encoder offset
     */
    public SwerveModule(int throttleID, int rotorID, int rotorEncoderID, double rotorOffsetAngleDeg) 
    {
        // Instantiate throttle motor & respective encoder
        mThrottle = new CANSparkMax(throttleID, MotorType.kBrushless);
        mThrottleEncoder = mThrottle.getEncoder();

        // Instantiate rotor motor 
        mRotor = new CANSparkMax(rotorID, MotorType.kBrushless);
        
        // Instantiate rotor absolute encoder
        mRotorEncoder = new WPI_CANCoder(rotorEncoderID);

        // Reset all configuration 
        // (technically optional, but good practice in case there are old 
        // configurations that can mess with the code)
        mThrottle.restoreFactoryDefaults();
        mRotor.restoreFactoryDefaults();
        mRotorEncoder.configFactoryDefault();

        // Configures rotor motor according to previously defined constants
        mRotor.setInverted(SwerveConstants.kRotorMotorInversion); 
        mRotor.enableVoltageCompensation(Constants.kVoltageCompensation);
        mRotor.setIdleMode(IdleMode.kBrake);

        // Configures rotor encoder according to previously defined constants
        mRotorEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        mRotorEncoder.configMagnetOffset(rotorOffsetAngleDeg);
        mRotorEncoder.configSensorDirection(SwerveConstants.kRotorEncoderDirection); 
        mRotorEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        // Configures rotor PID controller according to previously defined constants
        mRotorPID = new PIDController(SwerveConstants.kRotor_kP, SwerveConstants.kRotor_kI, SwerveConstants.kRotor_kD);
        
        // Continuous input considers the min & max to be the same point and 
        // automatically calculates the shortest route to the setpoint
        mRotorPID.enableContinuousInput(-180, 180);
        
        // Configures throttle motor according to previously defined constants
        mThrottle.enableVoltageCompensation(Constants.kVoltageCompensation);
        mThrottle.setIdleMode(IdleMode.kBrake);

        // Sets conversion factor to throttle encoder so that it reads 
        // velocity in meters per second instead of RPM
        mThrottleEncoder.setVelocityConversionFactor(SwerveConstants.kThrottleVelocityConversionFactor);
    }

    /**
     * Return current state of module
     * 
     * @return module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            mThrottleEncoder.getVelocity(),
            Rotation2d.fromDegrees(mRotorEncoder.getAbsolutePosition())
        );
    }

    /**
     * Set module state
     * 
     * @param state module state 
     */
    public void setState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getState().angle);
        double rotorOutput = mRotorPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());

        mRotor.set(rotorOutput);
        mThrottle.set(optimizedState.speedMetersPerSecond);
    }
}
