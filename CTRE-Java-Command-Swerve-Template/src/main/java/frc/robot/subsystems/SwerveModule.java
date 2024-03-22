package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    // Initialize rotor & throttle motors 
    // 初始化 rotor & throttle 馬達
    private TalonFX mRotor;
    private TalonFX mThrottle;

    // Initialize rotor encoder
    // 初始化 rotor encoder
    private CANcoder mRotorEncoder; 

    // Initialize rotor PID controller
    // 初始化 rotor PID controller
    private PIDController mRotorPID; 

    /**
     * 構建新的 SwerveModule
     *
     * @param throttleID CAN ID of throttle 馬達
     * @param rotorID CAN ID of rotor 馬達
     * @param rotorEncoderID CAN ID of rotor encoder
     * @param rotorOffsetAngleDeg rotor encoder 偏移量
     */
    public SwerveModule(int throttleID, int rotorID, int rotorEncoderID, double rotorOffsetAngleDeg)
    {
        // 實例化 throttle 馬達
        mThrottle = new TalonFX(throttleID);

        // 實例化 rotor 馬達
        mRotor = new TalonFX(rotorID);

        // 實例化 rotor absolute encoder
        mRotorEncoder = new CANcoder(rotorEncoderID);

        // 根據之前的常數配置 rotor 馬達
        MotorOutputConfigs rotorMotorOutputConfigs = new MotorOutputConfigs();
        rotorMotorOutputConfigs.Inverted = SwerveConstants.kRotorMotorInversion;
        rotorMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        mRotor.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(rotorMotorOutputConfigs), Constants.kLongTimeoutMs);

        // 根據之前的常數配置轉向 rotor encoder
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        magnetSensorConfigs.SensorDirection = SwerveConstants.kRotorEncoderDirection;
        magnetSensorConfigs.MagnetOffset = rotorOffsetAngleDeg;
        mRotorEncoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(magnetSensorConfigs), Constants.kLongTimeoutMs);

        // 根據之前的常數配置 rotor 馬達的PID控制器
        mRotorPID = new PIDController(
            SwerveConstants.kRotor_kP,
            SwerveConstants.kRotor_kI,
            SwerveConstants.kRotor_kD
        );

        // ContinuousInput 認為 min 和 max 是同一點並且自動計算到設定點的最短路線
        mRotorPID.enableContinuousInput(-180, 180);

        // 根據之前的常數配置 throttle 馬達
        MotorOutputConfigs throttleMotorOutputConfigs = new MotorOutputConfigs();
        throttleMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        mThrottle.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(throttleMotorOutputConfigs), Constants.kLongTimeoutMs);
    }

    /**
     * Return current state of module
     * 
     * @return module state
     */
    public SwerveModuleState getState() {
        double throttleVelocity = mThrottle.getVelocity().getValueAsDouble() * SwerveConstants.kThrottleVelocityConversionFactor; 
        
        return new SwerveModuleState(
            throttleVelocity, 
            Rotation2d.fromRotations(mRotorEncoder.getAbsolutePosition().getValueAsDouble())
        );
    }

    /**
     * Return current position of module
     * 
     * @return module position
     */
    public SwerveModulePosition getPosition() {
        double throttlePosition = mThrottle.getPosition().getValueAsDouble() * SwerveConstants.kThrottlePositionConversionFactor;

        return new SwerveModulePosition(
            throttlePosition, 
            Rotation2d.fromRotations(mRotorEncoder.getAbsolutePosition().getValueAsDouble())
        );
    }

    /**
     * Set module state
     * 
     * @param state module state 
     */
    public void setState(SwerveModuleState state) {
        // 優化狀態，使轉向馬達不必旋轉超過 90 度來獲得目標的角度
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getState().angle);
        
        // 通過比較目前角度與目標角度來用 PID 控制器計算轉向馬達所需的輸出
        double rotorOutput = mRotorPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());

        mRotor.set(rotorOutput);
        mThrottle.set(optimizedState.speedMetersPerSecond);
    }
}
