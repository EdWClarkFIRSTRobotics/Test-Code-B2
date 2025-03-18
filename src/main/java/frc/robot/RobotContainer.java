// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;

import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer extends SubsystemBase{

  

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                    // max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


  private SendableChooser<Command> autoChooser;
  
    public RobotContainer() {
  
  
      
      // Subsystem initialization
      Swerve swerve = new Swerve();
      ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
  
      PathPlannerAuto autoCommand = new PathPlannerAuto("Franklin Test");
      // PathPlannerAuto can also be created with a custom command
      // autoCommand = new PathPlannerAuto(new CustomAutoCommand());
  
      // Bind to different auto triggers
      autoCommand.isRunning().onTrue(Commands.print("Example Auto started"));
      autoCommand.timeElapsed(5).onTrue(Commands.print("5 seconds passed"));
      autoCommand.timeRange(6, 8).whileTrue(Commands.print("between 6 and 8 seconds"));
      autoCommand.event("Example Event Marker").onTrue(Commands.print("passed example event marker"));
      autoCommand.pointTowardsZone("Speaker").onTrue(Commands.print("aiming at speaker"));
      autoCommand.activePath("Example Path").onTrue(Commands.print("started following Example Path"));
      autoCommand.nearFieldPosition(new Translation2d(2, 2), 0.5).whileTrue(Commands.print("within 0.5m of (2, 2)"));
      autoCommand.inFieldArea(new Translation2d(2, 2), new Translation2d(4, 4))
          .whileTrue(Commands.print("in area of (2, 2) - (4, 4)"));
      // Register Named Commands
      NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
      NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
      // NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());
  
      // Do all other initialization
      // ...
      configureBindings();
  
      autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
      SmartDashboard.putData("Auto Mode", autoChooser);
  
      // For convenience a programmer could change this when going to competition.
      boolean isCompetition = true;
  
      // Build an auto chooser. This will use Commands.none() as the default option.
      // autoChooser = AutoBuilder.buildAutoChooser();
      autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith("comp"))
        : stream
    );
    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);

    final Field2d field;
field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });
    
}

    

  private Object driveRobotRelative(ChassisSpeeds speeds) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'driveRobotRelative'");
  }

  private void resetPose(Pose2d pose2d1) {
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    return null;
  }

  private Pose2d getPose() {
    return null;
  }




  private void configureBindings() {

        // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
        SmartDashboard.putData("Franklin Test", new PathPlannerAuto("Franklin Test "));

        
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with //
                                                                                           // (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                        // negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain.applyRequest(
        () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // joystick.x().whileTrue(() -> InstantCommand(new
    // ClawSubsystem.angleClaw(0.1)));
    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {


  // // Drivetrain will execute this command periodically
  // return drivetrain.applyRequest(() -> drive.withVelocityX(-0.25 * MaxSpeed) //
  // // Drive forward with negative Y (forward)
  // .withVelocityY(0) // Drive left with negative X (left)
  // .withRotationalRate(0) // Drive counterclockwise with negative X (left)
  // );

  
    return autoChooser.getSelected();

  }


    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    // return new PathPlannerAuto("Franklin Test");
   // try {
      // Load the path you want to follow using its name in the GUI
      public Command followPathCommand(String pathName) {
        try{
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    
            return new FollowPathCommand(
                    path,
                    () -> getPose(), // Robot pose supplier
                    () -> getRobotRelativeSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (t, u) -> drive(t, u), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                    ),
                    Constants.robotConfig, // The robot configuration
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

      // Create a path following command using AutoBuilder. This will also trigger
      // event markers.
      // PathPlannerPath path;
      //return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("LOSER, it doesn't work.: " + e.getMessage(), e.getStackTrace());
      return Commands.none();

    }
  
    //return autoChooser.getSelected();
  }
// Load the path we want to pathfind to and follow
//PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

// Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        9.42, 12.57); //Units.degreesToRadians(540), Units.degreesToRadians(720));
        private PathPlannerPath path;
        
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                path,
        constraints);

      private Object drive(ChassisSpeeds t, DriveFeedforwards u) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'drive'");
      }
      public class MyPathfinder implements Pathfinder {
  /**
   * Get if a new path has been calculated since the last time a path was retrieved
   *
   * @return True if a new path is available
   */
  @Override
  public boolean isNewPathAvailable() {
      return false;
      //return false;
    // TODO: Implement your pathfinder
  }
  public void robotInit() {
  // ... all other robot initialization

  // DO THIS AFTER CONFIGURATION OF YOUR DESIRED PATHFINDER
  PathfindingCommand.warmupCommand().schedule();

  // Override the X feedback
PPHolonomicDriveController.overrideXFeedback(() -> {
  // Calculate feedback from your custom PID controller
  return 0.0;
});
// Clear the X feedback override
PPHolonomicDriveController.clearXFeedbackOverride();

// Override the Y feedback
PPHolonomicDriveController.overrideYFeedback(() -> {
  // Calculate feedback from your custom PID controller
  return 0.0;
});
// Clear the Y feedback override
PPHolonomicDriveController.clearYFeedbackOverride();

// Override the rotation feedback
PPHolonomicDriveController.overrideRotationFeedback(() -> {
  // Calculate feedback from your custom PID controller
  return 0.0;
});
// Clear the rotation feedback override
PPHolonomicDriveController.clearRotationFeedbackOverride();

// Clear all feedback overrides
PPHolonomicDriveController.clearFeedbackOverrides();
}
// Load a full Choreo trajectory as a PathPlannerPath
//PathPlannerPath exampleChoreoTraj = PathPlannerPath.fromChoreoTrajectory("Example Choreo Traj");
// Load a split Choreo trajectory as a PathPlannerPath, using the split point with index 1
//PathPlannerPath exampleChoreoTrajSplit = PathPlannerPath.fromChoreoTrajectory("Example Choreo Traj", 1);

  /**
   * Get the most recently calculated path
   *
   * @param constraints The path constraints to use when creating the path
   * @param goalEndState The goal end state to use when creating the path
   * @return The PathPlannerPath created from the points calculated by the pathfinder
   */
  @Override
  public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
      return path;
    // TODO: Implement your pathfinder
  }

  /**
   * Set the start position to pathfind from
   *
   * @param startPosition Start position on the field. If this is within an obstacle it will be
   *     moved to the nearest non-obstacle node.
   */
  @Override
  public void setStartPosition(Translation2d startPosition) {
    // TODO: Implement your pathfinder
  }

  /**
   * Set the goal position to pathfind to
   *
   * @param goalPosition Goal position on the field. f this is within an obstacle it will be moved
   *     to the nearest non-obstacle node.
   */
  @Override
  public void setGoalPosition(Translation2d goalPosition) {
    // TODO: Implement your pathfinder
  }

  /**
   * Set the dynamic obstacles that should be avoided while pathfinding.
   *
   * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d represents
   *     opposite corners of a bounding box.
   * @param currentRobotPos The current position of the robot. This is needed to change the start
   *     position of the path to properly avoid obstacles
   */
  @Override
  public void setDynamicObstacles(
          List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
    // TODO: Implement your pathfinder
  }
}
}

