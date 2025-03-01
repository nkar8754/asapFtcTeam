package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import android.text.PrecomputedText;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class Odometry {
    // Locations for the swerve drive modules relative to the
// robot center.

    public static SparkFunOTOS myOtos;
    Translation2d m_frontLeftLocation =
            new Translation2d(152.406, 117);
    Translation2d m_frontRightLocation =
            new Translation2d(152.406, -117);
    Translation2d m_backLeftLocation =
            new Translation2d(-152.406, 117);
    Translation2d m_backRightLocation =
            new Translation2d(-152.406, -117);
     Translation2d m_centerLocation =
            new Translation2d(0.0, 0.0);

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
                    m_kinematics, new Rotation2d(myOtos.getPosition().h), new Pose2d(5.0, 13.5, new Rotation2d())
            );
}
