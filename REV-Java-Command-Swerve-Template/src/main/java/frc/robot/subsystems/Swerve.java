package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Swerve extends SubsystemBase {
    // Initialize IMU
    // 初始化 IMU
    private final Pigeon2 mImu = new Pigeon2(SwerveConstants.kImuID);

    private final SwerveModule mLeftFrontModule, mRightFrontModule, mLeftRearModule, mRightRearModule;
    private SwerveDriveOdometry mOdometry;

    public Swerve() {
        // Instantiate swerve modules - each representing unique module on the robot
        // 實例化（instantiate）swerve module - 個代表機器上四個其一
        mLeftFrontModule = new SwerveModule(
            SwerveConstants.kLeftFrontThrottleID, 
            SwerveConstants.kLeftFrontRotorID, 
            SwerveConstants.kLeftFrontCANCoderID, 
            SwerveConstants.kLeftFrontRotorOffset
        );

        mRightFrontModule = new SwerveModule(
            SwerveConstants.kRightFrontThrottleID, 
            SwerveConstants.kRightFrontRotorID, 
            SwerveConstants.kRightFrontCANCoderID, 
            SwerveConstants.kRightFrontRotorOffset
        );

        mLeftRearModule = new SwerveModule(
            SwerveConstants.kLeftRearThrottleID, 
            SwerveConstants.kLeftRearRotorID, 
            SwerveConstants.kLeftRearCANCoderID, 
            SwerveConstants.kLeftRearRotorOffset
        );

        mRightRearModule = new SwerveModule(
            SwerveConstants.kRightRearThrottleID, 
            SwerveConstants.kRightRearRotorID, 
            SwerveConstants.kRightRearCANCoderID, 
            SwerveConstants.kRightRearRotorOffset
        );

        // Instantiate odometry - used for tracking position
        // 實例化（instantiate）測程法（odometry） - 用於追踪位置
        mOdometry = new SwerveDriveOdometry(SwerveConstants.kSwerveKinematics, mImu.getRotation2d(), getModulePositions());

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                4.5, // Max module speed, in m/s
                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    @Override
    public void periodic() {
        // Updates odometry with current module state
        // 使用當前Swerve module狀態更新測程法（odometry）。
        mOdometry.update(
            mImu.getRotation2d(), 
            getModulePositions()
        );
    }

    /**
     * Drives the swerve - Input range: [-1, 1]
     * 
     * @param xSpeed percent power in the X direction (X 方向的功率百分比)
     * @param ySpeed percent power in the Y direction (Y 方向的功率百分比)
     * @param zSpeed percent power for rotation (旋轉的功率百分比)
     * @param fieldOriented configure robot movement style (設置機器運動方式) (field or robot oriented)
     */
    public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented) {
        SwerveModuleState[] states = null;
        if(fieldOriented) {
            states = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
                // IMU used for field oriented control
                // IMU 用於 Field Oriented Control
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, mImu.getRotation2d())
            );
        } else {
            states = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(xSpeed, ySpeed, zSpeed)
            );
        }
        setModuleStates(states);
    }

    /**
     * Get current swerve module states
     * 輸出 4 個 Swerve Module 的當前狀態 modules
     * 
     * @return swerve module states
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{
            mLeftFrontModule.getState(), 
            mRightFrontModule.getState(), 
            mLeftRearModule.getState(), 
            mRightRearModule.getState()
        };
    }

    /**
     * Get current swerve module positions
     * 
     * @return swerve module positions 
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            mLeftFrontModule.getPosition(), 
            mRightFrontModule.getPosition(), 
            mLeftRearModule.getPosition(), 
            mRightRearModule.getPosition()
        };
    }

    /**
     * Sets swerve module states
     * 設置 4 個 Swerve module 的狀態。
     * 
     * @param desiredStates array of desired states, order: [leftFront, leftRear, rightFront, rightRear]
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 1);
        mLeftFrontModule.setState(desiredStates[0]);
        mRightFrontModule.setState(desiredStates[1]);
        mLeftRearModule.setState(desiredStates[2]);
        mRightRearModule.setState(desiredStates[3]);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return SwerveConstants.kSwerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        setModuleStates(SwerveConstants.kSwerveKinematics.toSwerveModuleStates(speeds));
    }

    /**
     * Get predicted pose
     * 獲取機器人的當前位置
     * 
     * @return pose
     */
    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    /**
     * Set robot pose
     * 將測程法（odometry）位置設置為給與的 x、y、位置和角度
     * 
     * @param pose robot pose
     */
    public void setPose(Pose2d pose) {
        mOdometry.resetPosition(mImu.getRotation2d(), getModulePositions(), pose);
    }
}
