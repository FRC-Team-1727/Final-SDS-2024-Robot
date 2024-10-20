package frc.robot;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldLayout {
	public static double kFieldLength = Units.inchesToMeters(651.223);
	public static double kFieldWidth = Units.inchesToMeters(323.277);
	public static double kWingX = Units.inchesToMeters(229.201);
	public static double kPodiumX = Units.inchesToMeters(126.75);
	public static double kStartingLineX = Units.inchesToMeters(74.111);
	public static Pose2d kSpeakerCenter = new Pose2d(0.2, kFieldWidth - Units.inchesToMeters(104.0), new Rotation2d());
    public static Pose2d kSpeakerRed = new Pose2d(kFieldLength - 0.2, kFieldWidth - Units.inchesToMeters(104.0), Rotation2d.fromDegrees(180));

	public static final double kApriltagWidth = Units.inchesToMeters(6.50);
	public static final AprilTagFieldLayout kTagMap;

	static {
		try {
			kTagMap = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}

	// center notes labeled 1-5, with 1 being closest to the fms table
	public static Translation2d kCenterNote5 = new Translation2d(kFieldLength / 2.0, 0.752856);
	public static Translation2d kCenterNote4 = new Translation2d(kFieldLength / 2.0, 2.429256);
	public static Translation2d kCenterNote3 = new Translation2d(kFieldLength / 2.0, 4.105656);
	public static Translation2d kCenterNote2 = new Translation2d(kFieldLength / 2.0, 5.782056);
	public static Translation2d kCenterNote1 = new Translation2d(kFieldLength / 2.0, 7.458456);

	public static Translation2d[] kCenterNotes =
			new Translation2d[] {kCenterNote1, kCenterNote2, kCenterNote3, kCenterNote4, kCenterNote5};

	public static Translation2d kAmpCenter =
			new Translation2d(Units.inchesToMeters(72.455), Units.inchesToMeters(322.996));

	/** Center of the speaker opening (blue alliance) */
	public static double distanceFromAllianceWall(double x_coordinateMeters, boolean is_red_alliance) {
		if (is_red_alliance) {
			return kFieldLength - x_coordinateMeters;
		}
		return x_coordinateMeters;
	}


    public static double angleDiffToSpeaker(Pose2d current_pos) {
        Pose2d thepose = DriverStation.getAlliance().get() == Alliance.Red ? kSpeakerRed : kSpeakerCenter;

        return thepose.getTranslation().minus(current_pos.getTranslation()).getAngle().getDegrees();
    }
}
