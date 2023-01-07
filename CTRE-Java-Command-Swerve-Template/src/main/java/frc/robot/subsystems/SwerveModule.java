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
    // 初始化 rotor & throttle 馬達
    private WPI_TalonFX mRotor;
    private WPI_TalonFX mThrottle;

    // Initialize rotor encoder
    // 初始化 rotor encoder
    private WPI_CANCoder mRotorEncoder; 

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
        mThrottle = new WPI_TalonFX(throttleID);

        // 實例化 rotor 馬達
        mRotor = new WPI_TalonFX(rotorID);

        // 實例化 rotor absolute encoder
        mRotorEncoder = new WPI_CANCoder(rotorEncoderID);

        // 重置所有配置（保險起見以免有舊的配置）
        mThrottle.configFactoryDefault();
        mRotor.configFactoryDefault();
        mRotorEncoder.configFactoryDefault();

        // 根據之前的常數配置 rotor 馬達
        mRotor.setInverted(SwerveConstants.kRotorMotorInversion);
        mRotor.configVoltageCompSaturation(Constants.kVoltageCompensation);
        mRotor.enableVoltageCompensation(true);
        mRotor.setNeutralMode(NeutralMode.Brake);

        // 根據之前的常數配置轉向 rotor encoder
        mRotorEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        mRotorEncoder.configMagnetOffset(rotorOffsetAngleDeg);
        mRotorEncoder.configSensorDirection(SwerveConstants.kRotorEncoderDirection);
        mRotorEncoder.configSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition
        );

        // 根據之前的常數配置 rotor 馬達的PID控制器
        mRotorPID = new PIDController(
            SwerveConstants.kRotor_kP,
            SwerveConstants.kRotor_kI,
            SwerveConstants.kRotor_kD
        );

        // ContinuousInput 認為 min 和 max 是同一點並且自動計算到設定點的最短路線
        mRotorPID.enableContinuousInput(-180, 180);

        // 根據之前的常數配置 throttle 馬達
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
        // 優化狀態，使轉向馬達不必旋轉超過 90 度來獲得目標的角度
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getState().angle);
        
        // 通過比較目前角度與目標角度來用 PID 控制器計算轉向馬達所需的輸出
        double rotorOutput = mRotorPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());

        mRotor.set(rotorOutput);
        mThrottle.set(optimizedState.speedMetersPerSecond);
    }
}
