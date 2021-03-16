// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;

public class RomiDrivetrain extends SubsystemBase {
	private static final double kCountsPerRevolution = 1440.0;
	private static final double kWheelDiameterMeter = 0.07; // 70 mm

	// The Romi has the left and right motors set to
	// PWM channels 0 and 1 respectively
	private final Spark m_leftMotor = new Spark(0);
	private final Spark m_rightMotor = new Spark(1);

	// The Romi has onboard encoders that are hardcoded
	// to use DIO pins 4/5 and 6/7 for the left and right
	private final Encoder m_leftEncoder = new Encoder(4, 5);
	private final Encoder m_rightEncoder = new Encoder(6, 7);

	private final RomiGyro m_gyro = new RomiGyro();

	// Set up the differential drive controller
	private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

	private final DifferentialDriveOdometry m_odometry;

	private final Field2d m_field = new Field2d();

	/** Creates a new RomiDrivetrain. */
	public RomiDrivetrain() {
		// Use inches as unit for encoder distances
		m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
		m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
		resetEncoders();

		m_odometry = new DifferentialDriveOdometry(getRotation2d(), Constants.kInitialPose);
		SmartDashboard.putData("field", m_field);
	}

	public void curvatureDrive(double xaxisSpeed, double zaxisRotate, boolean isQuickTurn) {
		m_diffDrive.curvatureDrive(xaxisSpeed, zaxisRotate, isQuickTurn);
	}

	public void resetEncoders() {
		m_leftEncoder.reset();
		m_rightEncoder.reset();
	}

	public void stopDrivetrain() {
		tankDriveVolts(0, 0);
	}

	public double getLeftDistanceMeter() {
		return m_leftEncoder.getDistance();
	}

	public double getRightDistanceMeter() {
		return m_rightEncoder.getDistance();
	}

	public double getAngle() {
		return m_gyro.getAngleZ();
	}

	public Rotation2d getRotation2d() {
		return Rotation2d.fromDegrees(-getAngle());
	}

	public void resetOdometry(Pose2d pose) {
		resetEncoders();
		m_odometry.resetPosition(pose, getRotation2d());
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
	}

	public void tankDriveVolts(double leftVolts, double rightVolts) {
		m_leftMotor.setVoltage(leftVolts);
		m_rightMotor.setVoltage(-rightVolts);
		m_diffDrive.feed();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		m_odometry.update(getRotation2d(), getLeftDistanceMeter(), getRightDistanceMeter());

		m_field.setRobotPose(getPose());
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}
}
