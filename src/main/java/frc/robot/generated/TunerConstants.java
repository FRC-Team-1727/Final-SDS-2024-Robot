package frc.robot.generated;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class TunerConstants {
        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with
        // the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs()
                        .withKP(100).withKI(0).withKD(0.2)
                        .withKS(0).withKV(1.5).withKA(0);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs()
                        .withKP(3).withKI(0).withKD(0)
                        .withKS(0).withKV(0).withKA(0);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final double kSlipCurrentA = 150.0;

        // Initial configs for the drive and steer motors and the CANcoder; these cannot
        // be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API
        // documentation.
        private static TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        private static TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
                        .withCurrentLimits(
                                        new CurrentLimitsConfigs()
                                                        // Swerve azimuth does not require much torque output, so we can
                                                        // set a relatively low
                                                        // stator current limit to help avoid brownouts without
                                                        // impacting performance.
                                                        .withStatorCurrentLimit(60)
                                                        .withStatorCurrentLimitEnable(true));

        private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        private static final Pigeon2Configuration pigeonConfigs = null;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        public static final double kSpeedAt12VoltsMps = 5.95;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 0;

        private static final double kDriveGearRatio = 5.36;
        private static final double kSteerGearRatio = 18.75;
        private static final double kWheelRadiusInches = 2;

        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        private static final String kCANbusName = "Canivore";
        private static final int kPigeonId = 0;

        // These are only used for simulation
        private static final double kSteerInertia = 0.00001;
        private static final double kDriveInertia = 0.001;
        // Simulated voltage necessary to overcome friction
        private static final double kSteerFrictionVoltage = 0.25;
        private static final double kDriveFrictionVoltage = 0.25;

        private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                        .withCANbusName(kCANbusName)
                        .withPigeon2Id(kPigeonId)
                        .withPigeon2Configs(pigeonConfigs);

        private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                        .withDriveMotorGearRatio(kDriveGearRatio)
                        .withSteerMotorGearRatio(kSteerGearRatio)
                        .withWheelRadius(kWheelRadiusInches)
                        .withSlipCurrent(kSlipCurrentA)
                        .withSteerMotorGains(steerGains)
                        .withDriveMotorGains(driveGains)
                        .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                        .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                        .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                        .withSteerInertia(kSteerInertia)
                        .withDriveInertia(kDriveInertia)
                        .withSteerFrictionVoltage(kSteerFrictionVoltage)
                        .withDriveFrictionVoltage(kDriveFrictionVoltage)
                        .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                        .withCouplingGearRatio(kCoupleRatio)
                        .withDriveMotorInitialConfigs(driveInitialConfigs)
                        .withSteerMotorInitialConfigs(steerInitialConfigs)
                        .withCANcoderInitialConfigs(cancoderInitialConfigs);

        // Front Left
        private static final int kFrontLeftDriveMotorId = 2;
        private static final int kFrontLeftSteerMotorId = 3;
        private static final int kFrontLeftEncoderId = 1;
        private static final double kFrontLeftEncoderOffset = -0.237060546875;
        private static final boolean kFrontLeftSteerInvert = true;

        public static final double kFrontLeftXPosInches = 8.375;
        public static final double kFrontLeftYPosInches = 9.375;

        // Front Right
        private static final int kFrontRightDriveMotorId = 1;
        private static final int kFrontRightSteerMotorId = 0;
        private static final int kFrontRightEncoderId = 0;
        private static final double kFrontRightEncoderOffset = -0.16455078125;
        private static final boolean kFrontRightSteerInvert = true;

        public static final double kFrontRightXPosInches = 8.375;
        public static final double kFrontRightYPosInches = -9.375;

        // Back Left
        private static final int kBackLeftDriveMotorId = 5;
        private static final int kBackLeftSteerMotorId = 4;
        private static final int kBackLeftEncoderId = 2;
        private static final double kBackLeftEncoderOffset = 0.212158203125;
        private static final boolean kBackLeftSteerInvert = true;

        public static final double kBackLeftXPosInches = -8.375;
        public static final double kBackLeftYPosInches = 9.375;

        // Back Right
        private static final int kBackRightDriveMotorId = 6;
        private static final int kBackRightSteerMotorId = 7;
        private static final int kBackRightEncoderId = 3;
        private static final double kBackRightEncoderOffset = 0.3154296875;
        private static final boolean kBackRightSteerInvert = true;

        public static final double kBackRightXPosInches = -8.375;
        public static final double kBackRightYPosInches = -9.375;

        // Motor IDs
        public static final int kLeftIntakeID = 1;
        public static final int kRightIntakeID = 6;
        public static final int kPivotID = 2;
        public static final int kIndexerID = 3;
        public static final int kLeftShooterID = 4;
        public static final int kRightShooterID = 5;
        public static final int kSensorID = 0;
        public static final int kSensorID2 = 1;

        public static final double kShooterP = .0008;
        public static final double kShooterI = 0;
        public static final double kShooterD = 0;

        public static final double kIndexerP = 0;
        public static final double kIndexerI = 0;
        public static final double kIndexerD = 0;
        
        public static final double kAnglerP = 0.15;
        public static final double kAnglerI = 0;
        public static final double kAnglerD = 0;

        //Intake
        public static final double kIndexAngle = 6.988;
        public static final double kStuckAngle = 14.5;
        public static final double kSubwooferAngle = 7.00;
        public static final double kSourceAngle = 7.605;
        public static final double kDefaultAngle = kIndexAngle;
        public static final double kIntakeSpd = 1;
        public static final double kIndexerSpd = 1;
        public static final double kInverseIndexerSpd = -.75;
        public static final double kOuttakeSpd = 0.9;
        public static final double kBackoutSpeed = 0.4;

        //drive stuff
        public static final HolonomicPathFollowerConfig kConfig = 
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5.0, 0.0, 0.0), 
                        new PIDConstants(5.0, 0.0, 0.0), 
                        3.5, 
                        .32, 
                        new ReplanningConfig());

        private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                        kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
                        Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches),
                        kInvertLeftSide)
                        .withSteerMotorInverted(kFrontLeftSteerInvert);
        private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
                        kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
                        kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches),
                        Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide)
                        .withSteerMotorInverted(kFrontRightSteerInvert);
        private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                        kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                        Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches),
                        kInvertLeftSide)
                        .withSteerMotorInverted(kBackLeftSteerInvert);
        private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                        kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
                        Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches),
                        kInvertRightSide)
                        .withSteerMotorInverted(kBackRightSteerInvert);

        // FrontRight.configNeutralMode(NeutralModeValue.Coast);

        public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants,
                        FrontLeft,
                        FrontRight, BackLeft, BackRight);
}
