// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final double ksVolts = 0.929;
	public static final double kvVoltSecondsPerMeter = 6.33;
	public static final double kaVoltSecondsSquaredPerMeter = 0.0389;
	public static final SimpleMotorFeedforward kDriveFeedforward =
		new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);

	public static final double kPDriveVel = 0.085;
	public static final double kIDriveVel = 0;
	public static final double kDDriveVel = 0;

	public static final double kTrackwidthMeters = 0.142072613;
	public static final DifferentialDriveKinematics kDriveKinematics =
		new DifferentialDriveKinematics(kTrackwidthMeters);

	public static final double kMaxSpeedMetersPerSecond = 0.8;
	public static final double kMaxAccelerationMetersPerSecondSquared = 0.8;

	public static final List<Translation2d> kInteriorWaypoints = List.of(
		new Translation2d(0.35, 0.15),
		new Translation2d(0.32, 0.5),
		new Translation2d(-0.15, 0.25),
		new Translation2d(-0.25, 0.6)
	);
	public static final Pose2d kEndPose = new Pose2d(0.4, 0.55, new Rotation2d());

	// Reasonable baseline values for a RAMSETE follower in units of meters and
	// seconds
	public static final double kRamseteB = 2;
	public static final double kRamseteZeta = 0.7;
}
