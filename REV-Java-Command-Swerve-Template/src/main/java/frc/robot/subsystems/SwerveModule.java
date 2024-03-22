package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
     // 初始化 rotor & throttle 馬達
    private CANSparkMax mRotor;
    private CANSparkMax mThrottle;

    // 初始化 throttle encoder
    private RelativeEncoder mThrottleEncoder;

    // 初始化 rotor encoder
    private CANcoder mRotorEncoder;

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
        // 實例化 throttle 馬達 & encoder
        mThrottle = new CANSparkMax(throttleID, MotorType.kBrushless);
        mThrottleEncoder = mThrottle.getEncoder();

        // 實例化 rotor 馬達
        mRotor = new CANSparkMax(rotorID, MotorType.kBrushless);

        // 實例化 rotor absolute encoder
        mRotorEncoder = new CANcoder(rotorEncoderID);

        // 重置所有配置（保險起見以免有舊的配置）
        mThrottle.restoreFactoryDefaults();
        mRotor.restoreFactoryDefaults();

        // 根據之前的常數配置 rotor 馬達
        mRotor.setInverted(SwerveConstants.kRotorMotorInversion);
        mRotor.enableVoltageCompensation(Constants.kVoltageCompensation);
        mRotor.setIdleMode(IdleMode.kBrake);

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
        mThrottle.enableVoltageCompensation(Constants.kVoltageCompensation);
        mThrottle.setIdleMode(IdleMode.kBrake);

        // 給與 throttle encoder 轉換係數以便它以米每秒而不是 RPM 為單位讀取速度
        mThrottleEncoder.setVelocityConversionFactor(
            SwerveConstants.kThrottleVelocityConversionFactor
        );
        mThrottleEncoder.setPositionConversionFactor(
            SwerveConstants.kThrottlePositionConversionFactor
        );
    }

    /**
     * Return current state of module
     * 
     * @return module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            mThrottleEncoder.getVelocity(),
            Rotation2d.fromRotations(mRotorEncoder.getAbsolutePosition().getValueAsDouble())
        );
    }
    
    /**
     * Return current position of module
     * 
     * @return module position
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            mThrottleEncoder.getPosition(), 
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
