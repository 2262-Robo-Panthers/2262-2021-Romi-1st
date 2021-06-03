package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;

public class RomiPositionTracking extends SubsystemBase{
    private static final double kCountsPerRevolution = 1440.0;
	private static final double kWheelDiameterMeter = 0.07; // 70 mm

    private final Encoder m_leftEncoder = new Encoder(4, 5);
	private final Encoder m_rightEncoder = new Encoder(6, 7);

    private final RomiGyro m_gyro = new RomiGyro();

    private final DifferentialDriveOdometry m_odometry;

    private final Field2d m_field = new Field2d();

    public RomiPositionTracking(){
        m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
		m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
        resetEncoders();
        
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-getAngle()), Constants.kInitialPose);
        
        SmartDashboard.putData("field", m_field);
    }

    public void resetEncoders() {
		m_leftEncoder.reset();
		m_rightEncoder.reset();
    }

    public void resetAngle(){
        m_gyro.reset();
    }

    public void resetOdometry(Pose2d pose) {
		resetEncoders();
		m_odometry.resetPosition(pose, Rotation2d.fromDegrees(-getAngle()));
    }
    
    @Override
	public void periodic() {
		// This method will be called once per scheduler run
		m_odometry.update(Rotation2d.fromDegrees(-getAngle()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
        
        m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    ///////////////
    //get methods//
    ///////////////
    public double getAngle() {
		return m_gyro.getAngleZ();
    }

    public Pose2d getPose() {
		return m_odometry.getPoseMeters();
    }
    
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
	}
}