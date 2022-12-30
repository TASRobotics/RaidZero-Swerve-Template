package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    // Initialize rotor & throttle motors 
    private WPI_TalonFX mRotor;
    private WPI_TalonFX mThrottle;

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
        // Instantiate throttle motor
        mThrottle = new WPI_TalonFX(throttleID);

        // Instantiate rotor motor 
        mRotor = new WPI_TalonFX(rotorID);

        // Instantiate rotor absolute encoder
        mRotorEncoder = new WPI_CANCoder(rotorEncoderID);

        // Reset all configuration 
        // (technically optional, but good practice in case there are old 
        // configurations that can mess with the code)
        mThrottle.configFactoryDefault();
        mRotor.configFactoryDefault();
        mRotorEncoder.configFactoryDefault();

        // Configures rotor motor according to previously defined constants
        mRotor.setInverted(SwerveConstants.kRotorMotorInversion); 
        mRotor.configVoltageCompSaturation(Constants.kVoltageCompensation);
        mRotor.enableVoltageCompensation(true);
        mRotor.setNeutralMode(NeutralMode.Brake);

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
        mThrottle.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mThrottle.configVoltageCompSaturation(Constants.kVoltageCompensation);
        mThrottle.enableVoltageCompensation(true);
        mThrottle.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Return current state of module
     * 
     * @return module state
     */
    public SwerveModuleState getState() {
        double throttleVelocity = mThrottle.getSelectedSensorVelocity() * SwerveConstants.kThrottleVelocityConversionFactor; 
        
        return new SwerveModuleState(
            throttleVelocity, 
            Rotation2d.fromDegrees(mRotorEncoder.getAbsolutePosition())
        );
    }

    /**
     * Set module state
     * 
     * @param state module state 
     */
    public void setState(SwerveModuleState state) {
        // Optimize the desired state so that the rotor doesn't have to spin more than
        // 90 degrees to get to the desired angle
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getState().angle);
        
        // Calculate rotor output using rotor PID controller by comparing the current
        // angle with the desired angle
        double rotorOutput = mRotorPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());

        mRotor.set(rotorOutput);
        mThrottle.set(optimizedState.speedMetersPerSecond);
    }
}
