package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry

public class Odometry {
    // Locations for the swerve drive modules relative to the
// robot center.
    Translation2d m_frontLeftLocation =
            new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation =
            new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation =
            new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation =
            new Translation2d(-0.381, -0.381);

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics
            (
                    m_frontLeftLocation, m_frontRightLocation,
                    m_backLeftLocation, m_backRightLocation
            );

    // Creating my odometry object from the kinematics object. Here,
// our starting pose is 5 meters along the long end of the
// field and in the
// center of the field along the short end, facing forward.
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry
            (
                    m_kinematics, getGyroHeading(),
                    new Pose2d(5.0, 13.5, new Rotation2d()
                    );


}
