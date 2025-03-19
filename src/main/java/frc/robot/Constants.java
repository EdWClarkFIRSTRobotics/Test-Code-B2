package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {

    public static RobotConfig robotConfig;
   
    public static final class Swerve {
        public static final Translation2d flModuleOffset = new Translation2d(0.546 / 2.0, 0.546 / 2.0);
        public static final Translation2d frModuleOffset = new Translation2d(0.546 / 2.0, -0.546 / 2.0);
        public static final Translation2d blModuleOffset = new Translation2d(-0.546 / 2.0, 0.546 / 2.0);
        public static final Translation2d brModuleOffset = new Translation2d(-0.546 / 2.0, -0.546 / 2.0);
    
        public static final double maxModuleSpeed = 4.5; // M/S
    
        public static final PIDConstants translationConstants = new PIDConstants(5.0, 0.0, 0.0);
        public static final PIDConstants rotationConstants = new PIDConstants(5.0, 0.0, 0.0);
      }



    // public class DriveSubsystem extends SubsystemBase {
    //     // Assuming ypur class name is DriveSubsytem
    //     private static final RobotConfig RobotConfig = null;

    //     public DriveSubsystem() {

    //         // All other subsystem initialization

    //         // Load the RobotConfig from the GUI settings. You should probably
    //         // store this in your Constants file
    //         RobotConfig config;
    //         try {
    //             config = RobotConfig.fromGUISettings();
    //         } catch (Exception e) {
    //             // Handle exception as needed
    //             e.printStackTrace();
    //         }

    //         // Configure AutoBuilder last
    //         AutoBuilder.configure(
    //                 this::getPose, // Robot pose supplier
    //                 this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
    //                 this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //                 (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given
    //                                                                       // ROBOT RELATIVE ChassisSpeeds. Also
    //                                                                       // optionally outputs individual module
    //                                                                       // feedforwards
    //                 new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
    //                                                 // for holonomic drive trains
    //                         new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //                         new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    //                 ),
    //                 RobotConfig, // The robot configuration
    //                 () -> {
    //                     // Boolean supplier that controls when the path will be mirrored for the red
    //                     // alliance
    //                     // This will flip the path being followed to the red side of the field.
    //                     // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //                     var alliance = DriverStation.getAlliance();
    //                     if (alliance.isPresent()) {
    //                         return alliance.get() == DriverStation.Alliance.Red;
    //                     }
    //                     return false;
    //                 },
    //                 this // Reference to this subsystem to set requirements
    //         );
    //     }
    // }
}
