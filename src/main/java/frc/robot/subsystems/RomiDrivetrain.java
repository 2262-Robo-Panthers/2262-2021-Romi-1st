// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class RomiDrivetrain extends SubsystemBase {
	// The Romi has the left and right motors set to
	// PWM channels 0 and 1 respectively
	private final Spark m_leftMotor = new Spark(0);
	private final Spark m_rightMotor = new Spark(1);
	private final PWM m_doorMotor = new PWM(2);

	// Set up the differential drive controller
	private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

	public void curvatureDrive(double xaxisSpeed, double zaxisRotate, boolean isQuickTurn) {
		m_diffDrive.curvatureDrive(xaxisSpeed, zaxisRotate, isQuickTurn);
	}

	public void stopDrivetrain() {
		tankDriveVolts(0, 0);
	}

	public void tankDriveVolts(double leftVolts, double rightVolts) {
		m_leftMotor.setVoltage(leftVolts);
		m_rightMotor.setVoltage(-rightVolts);
		m_diffDrive.feed();
	}
	public void SpinDoor(double rate) {
		m_doorMotor.setSpeed(rate);
	}
}
