package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public SwerveDrivePoseEstimator m_poseEstimator; // this pose estimator can do essentially everything swerve odometry can, with added vision capabilities
    public double poseAngle;
    public double poseForwardDistance;
    public double poseSideDistance;
    public PIDController pidControllerForTrackingOutput;
    public Double nuetralAimingTarget;

    // Cached hub coordinates — updated once per loop when alliance is known
    private double cachedHubX = Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_X;
    private double cachedHubY = Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_Y;

    //ll stuff
    private double pipeline = 0; 
    private double tv;
    public Pose2d poseLL; //want to use this pose after this command, after moving with odometry
    public Pose2d targetPose;

    //SmartDashboard
    private Field2d field = new Field2d();

    //targeting
    public SwerveControllerCommand currentSwerveControllerCommand;
    public Trajectory currentTrajectory;

    public double[] driveTargetingValues;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "usb");
        //when calibrated on 3/31/25, gyro mount pose configs quarternion values were:
        //gyro.getConfigurator().apply(-0.041875,  0.012086,  0.005250, Z	-0.997314))
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants), //front left
            new SwerveModule(1, Constants.Swerve.Mod1.constants), //front right
            new SwerveModule(2, Constants.Swerve.Mod2.constants), //back left
            new SwerveModule(3, Constants.Swerve.Mod3.constants) //back right
        };

        /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings, for 3D targeting. 
        The numbers used below are robot specific, and should be tuned. */
        m_poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                mSwerveMods[0].getPosition(), //front left
                mSwerveMods[1].getPosition(), //front right
                mSwerveMods[2].getPosition(), //back left
                mSwerveMods[3].getPosition()  //back right
            },
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Math.toRadians(5)), //std deviations in X, Y (meters), and angle of the pose estimate
            VecBuilder.fill(0.5, 0.5, Math.toRadians(30))  //std deviations  in X, Y (meters) and angle of the vision (LL) measurement
        );

        SmartDashboard.putData("Field", field);

        // PATH PLANNER

        SmartDashboard.putString("Starting swerve and pathplanner", "yes");

        RobotConfig config = null; // this is a PathPlannerLib object that will store the robot config values like mass, wheel numbers, etc. that are set in the App GUI
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        // PathPlanner requires an "AutoBuilder" object to be configured in order to run any paths. This object needs access to the robot's drive/pose methods and other
        // important information. PathPlanner recommends that this is configured in the drive subsystem's constructor (because it can take a bit to load), so it's done here.
        if (config == null) {
            System.out.println("PathPlanner RobotConfig settings null, may have errored");
        } else {
            AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getChassisSpeeds,
                (speeds, feedforwards) -> driveWithChassisSpeeds(speeds),
                new PPHolonomicDriveController(
                    Constants.PathPlanner.TRANSLATION_PID_CONSTANTS, //translation
                    Constants.PathPlanner.ROTATION_PID_CONSTANTS // rotation -- both can be tuned I think
                ),
                config,
                () -> { 
                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == Alliance.Red; //0 = red, 1 = blue
                    }
                    return false;
                },
                this
            );
        }

        pidControllerForTrackingOutput = new PIDController(
        Constants.Targeting.AUTO_ROTATE_KP,
        0.0,
        Constants.Targeting.AUTO_ROTATE_KD);

        pidControllerForTrackingOutput.enableContinuousInput(-Math.PI, Math.PI);
        // tolerance is to prevent gittering (this will need to be tuned)
        pidControllerForTrackingOutput.setTolerance(Math.toRadians(0.1));
        pidControllerForTrackingOutput.setSetpoint(0.0);


        // Update cached hub coordinates once per loop (avoids repeated DriverStation.getAlliance() calls)
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            cachedHubX = Constants.Targeting.RED_ALLIANCE_HUB_CENTER_X;
            cachedHubY = Constants.Targeting.RED_ALLIANCE_HUB_CENTER_Y;
        } else {
            cachedHubX = Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_X;
            cachedHubY = Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_Y;
        }
    }

//Methods start here:

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        var alliance = DriverStation.getAlliance();

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    alliance.isPresent() && alliance.get() == Alliance.Red ? -translation.getX() : translation.getX(), 
                                    alliance.isPresent() && alliance.get() == Alliance.Red ? -translation.getY() : translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false); //closed loop auto
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    // drive method used/required for path planner. Basically just a rewritten drive() (refer to above) method 
    public void driveWithChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true); //TODO: may be worth it to check out closedLoop, especially for auto. light research indicates it may be more precise (but less responsive to input)?
        }
    }

        //Follows path that assumes starting pose is robot's current pose (by RESETTING the robots odometry to be the start pose of the path)

    public Command followPathCommand(String pathName) {
        try {
            SmartDashboard.putNumber("RobotPoseX before path start", this.getPose().getX());
            SmartDashboard.putNumber("RobotPoseY before path start", this.getPose().getY());
            SmartDashboard.putNumber("RobotPoseRotation before path start", this.getPose().getRotation().getDegrees());

            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            
            Pose2d bluePathStartingPose = path.getStartingHolonomicPose().get(); // starting pose of path, defaults to blue side coordinates
            
            if (DriverStation.getAlliance().get() == Alliance.Red) { // if red alliance, flip the path starting pose
                this.resetPose(FlippingUtil.flipFieldPose(bluePathStartingPose));
            } else  {
                this.resetPose(path.getStartingHolonomicPose().get());
            }

            SmartDashboard.putNumber("RobotPoseX AFTER", this.getPose().getX());
            SmartDashboard.putNumber("RobotPoseY AFTER", this.getPose().getY());
            SmartDashboard.putNumber("RobotPoseRotation AFTER", this.getPose().getRotation().getDegrees());

            Waypoint pathStartWaypoint = path.getWaypoints().get(0);
            SmartDashboard.putNumber("Path start  X", pathStartWaypoint.anchor().getX());
            SmartDashboard.putNumber("Path start  Y", pathStartWaypoint.anchor().getY());
            SmartDashboard.putNumber("Path start  Rotation", path.getStartingHolonomicPose().get().getRotation().getDegrees());

            Waypoint pathEndWaypoint = path.getWaypoints().get(path.getWaypoints().size() - 1);
            SmartDashboard.putNumber("Path end  X", pathEndWaypoint.anchor().getX());
            SmartDashboard.putNumber("Path end  Y", pathEndWaypoint.anchor().getY());
            SmartDashboard.putNumber("Path end  Rotation", path.getGoalEndState().rotation().getDegrees());

            FieldObject2d start = field.getObject("PathStart");
            start.setPose(pathStartWaypoint.anchor().getX(), pathStartWaypoint.anchor().getY(), path.getIdealStartingState().rotation());
            
            FieldObject2d end = field.getObject("PathEnd");
            end.setPose(pathEndWaypoint.anchor().getX(), pathEndWaypoint.anchor().getY(), path.getGoalEndState().rotation());

            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    // Barebones and default PathPlanner way of following a path. This means that if the robot is not at the start pose of the path,
    // it will attempt to move there (this is done by PathPlanner). Therefore, ONLY USE if you know robot has a pose and it is at/extremely close to start pose
    public Command followPathCommandNoReset(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    //Follows a path where end pose does not change, but it starts at the robot's current pose. Does not retain event markers (if applicable)
    public Command followPathCommandRobotStartingPose(String pathName) {
        try {
            PathPlannerPath originalPath = PathPlannerPath.fromPathFile(pathName);
            PathConstraints constraints = originalPath.getGlobalConstraints(); // just use original path's constraints by default. if this doesn't work then can reconstruct

            List<Waypoint> newWaypoints = originalPath.getWaypoints();
            newWaypoints.set(0, PathPlannerPath.waypointsFromPoses(this.getPose()).get(0)); // turns robot's current pose into waypoint, sets it as first waypoint

            PathPlannerPath path = new PathPlannerPath(
                newWaypoints, 
                constraints, 
                null, 
                originalPath.getGoalEndState()
            );
            
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public Command getClimbTargetingPath() {
        try {
            Pose2d robotPoseBlue;
            
            // Alliance Check
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) { 
                robotPoseBlue = FlippingUtil.flipFieldPose(this.getPose());
            } else  {
                robotPoseBlue = this.getPose();    
            }
            
            PathConstraints constraints = PathPlannerPath.fromPathFile("climb-targeting").getGlobalConstraints();
            Pose2d pathEndPose;

            if (robotPoseBlue.getY() < Constants.Targeting.BLUE_TOWER_CENTER_Y_METERS) {
                pathEndPose = new Pose2d(1.146 + Constants.Targeting.ROBOT_WIDTH_METERS / 2.0, 3.302, Rotation2d.fromDegrees(180)); 
            } else {
                pathEndPose = new Pose2d(1.146 + Constants.Targeting.ROBOT_WIDTH_METERS / 2.0, 4.162, Rotation2d.fromDegrees(180));
            }

            Rotation2d splineHeading = pathEndPose.getTranslation().minus(robotPoseBlue.getTranslation()).getAngle();
            Pose2d startPoseForSpline = new Pose2d(robotPoseBlue.getTranslation(), splineHeading);

            // USE START POSE BC IT USES BLUE ONLY AUTOBUILDER I THINK FLIPS IT FOR US
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                startPoseForSpline, 
                pathEndPose 
            );

            PathPlannerPath path = new PathPlannerPath(
                waypoints, 
                constraints, 
                null, 
                new GoalEndState(0.0, Rotation2d.fromDegrees(0))
            );
            
            return AutoBuilder.followPath(path);
            
        } catch (Exception e) {
            DriverStation.reportError("Error in getClimbTargetingPath: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
    
    // ALSO WE CAN USE AutoBuilder.pathfindToPose(targetPose, constraints); I FOUND OTHERS USING IT

    // public Command getClimbTargetingPath() {
    //     try {
    //         Pose2d robotPoseBlue;
            
    //         if (DriverStation.getAlliance().get() == Alliance.Red) { // if red alliance, flip the path starting pose
    //             robotPoseBlue = (FlippingUtil.flipFieldPose(this.getPose()));
    //         } else  {
    //             robotPoseBlue = this.getPose();    
    //         }

            
    //         // this placeholder path is only used to get global constraints; could also put in manually
    //         PathConstraints constraints = PathPlannerPath.fromPathFile("climb-targeting").getGlobalConstraints();
    //         Pose2d pathEndPose;

    //         if (robotPoseBlue.getY() < Constants.Targeting.BLUE_TOWER_CENTER_Y_METERS) {
    //             pathEndPose = new Pose2d(1.146 + Constants.Targeting.ROBOT_WIDTH_METERS / 2.0, 3.302, Rotation2d.fromDegrees(180)); // 0 degrees is travel direction at poitn, not heading
    //         } else {
    //             pathEndPose = new Pose2d(1.146 + Constants.Targeting.ROBOT_WIDTH_METERS / 2.0, 4.153, Rotation2d.fromDegrees(180));
    //         }

    //         List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
    //             this.getPose(), // start at robot's current pose
    //             pathEndPose // end at fixed pose in front of tower)
    //         );

    //         // this path is all blue relative. however, it should be flipped automatically according to PathPlanner
    //         PathPlannerPath path = new PathPlannerPath(
    //             waypoints, 
    //             constraints, // these constraints will likely need to be tuned
    //             null, 
    //             new GoalEndState(0.0, Rotation2d.fromDegrees(0))
    //         );
            
    //         return AutoBuilder.followPath(path);
    //     } catch (Exception e) {
    //         DriverStation.reportError(e.getMessage(), e.getStackTrace());
    //         return Commands.none();
    //     }
    // }

    //Follows path relative to robot's current pose (shifts all poses and states to accomodate). Different than just changing the starting pose to Robot and keeping end same (as above)
    public Command followPathCommandRobotRelative(String pathName) {
        try {
            PathPlannerPath originalPath = PathPlannerPath.fromPathFile(pathName);

            PathConstraints constraints = originalPath.getGlobalConstraints(); // just use original path's constraints by default. if this doesn't work then can reconstruct
            IdealStartingState originalStartingState = originalPath.getIdealStartingState();
            GoalEndState originalGoalEndState = originalPath.getGoalEndState();
            
            Pose2d pathInitialPose = originalPath.getStartingHolonomicPose().get(); 
            List<Pose2d> newPoses = new ArrayList<>();
            newPoses.add(this.getPose()); // start with current robot pose

            int i = 0;
            for (Pose2d pose : originalPath.getPathPoses()) {
                if (i > 0) { // skip first pose since it has already been added
                    newPoses.add(this.getPose().transformBy(pose.minus(pathInitialPose))); 
                }

                i++;
            }

            PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(newPoses),
                constraints, 
                new IdealStartingState(originalStartingState.velocity(), originalStartingState.rotation().plus(getHeading().minus(pathInitialPose.getRotation()))), 
                new GoalEndState(originalGoalEndState.velocity(), originalGoalEndState.rotation().plus(getHeading().minus(pathInitialPose.getRotation())))
            );

            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

        public Command getFinishClimbCommand() {
        try {
            Pose2d robotPoseBlue;
            
            // Alliance Check
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) { 
                robotPoseBlue = FlippingUtil.flipFieldPose(this.getPose());
            } else  {
                robotPoseBlue = this.getPose();    
            }
            
            PathConstraints constraints = PathPlannerPath.fromPathFile("finish-climb").getGlobalConstraints();
            Pose2d pathEndPose;

            if (robotPoseBlue.getY() < Constants.Targeting.BLUE_TOWER_CENTER_Y_METERS) {
                pathEndPose = new Pose2d(.955 + Constants.Targeting.ROBOT_WIDTH_METERS / 2.0, 3.302, Rotation2d.fromDegrees(180)); 
            } else {
                pathEndPose = new Pose2d(.955 + Constants.Targeting.ROBOT_WIDTH_METERS / 2.0, 4.162, Rotation2d.fromDegrees(180));
            }

            Rotation2d splineHeading = pathEndPose.getTranslation().minus(robotPoseBlue.getTranslation()).getAngle();
            Pose2d startPoseForSpline = new Pose2d(robotPoseBlue.getTranslation(), splineHeading);

            // USE START POSE BC IT USES BLUE ONLY AUTOBUILDER I THINK FLIPS IT FOR US
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                startPoseForSpline, 
                pathEndPose 
            );

            PathPlannerPath path = new PathPlannerPath(
                waypoints, 
                constraints, 
                null, 
                new GoalEndState(0.0, Rotation2d.fromDegrees(0))
            );
            
            return AutoBuilder.followPath(path);
            
        } catch (Exception e) {
            DriverStation.reportError("Error in getClimbTargetingPath: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
    

    public ChassisSpeeds getChassisSpeeds() {
        SwerveModuleState[] states = getModuleStates();
        ChassisSpeeds fieldRelFromStates = Constants.Swerve.swerveKinematics.toChassisSpeeds(states);
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelFromStates, getHeading());
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        //return Rotation2d.fromDegrees(gyro.getYaw().getValue());
       return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void getLLPose() {
            // turn on the LED,  3 = force on
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
        // s_Swerve.zeroHeading(); //added this to fix the targeting going the wrong way

        //tv =1 means Limelight sees a target
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()  && (tv == 1)) { //have alliance color and see target
            if (ally.get() == Alliance.Red){
            poseLL = LimelightHelpers.getBotPose2d_wpiRed("limelight");
            //  s_Swerve.resetPose(poseLL); //do this later in ResetPose command
            }
            if (ally.get() == Alliance.Blue){
            poseLL = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
            // s_Swerve.resetPose(poseLL); //do this later in ResetPose command
            }   
        }
        //else do nothing
    }

    public void getTargetPose(Pose2d targetPose) {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.get() == Alliance.Blue){
            this.targetPose = new Pose2d(targetPose.getX(), targetPose.getY(), targetPose.getRotation().plus(new Rotation2d(Math.PI))); //do this later in ResetPose command
        }
        if (ally.get() == Alliance.Red){ // not sure why but we needed to swap these
            this.targetPose = targetPose; //do this later in ResetPose command
        }   

        // this.resetPose(targetPose);

    }

    public void resetFieldPoseWithTarget() {
        if (targetPose != null) {
            resetPose(targetPose);
        }
    }

    public void resetLLPose() {
        if (poseLL != null) {
            resetPose(poseLL);
        }
    }

    public void MegaTag2UpdateOdometry() {

        m_poseEstimator.update(getGyroYaw(), getModulePositions());

        boolean useMegaTag2 = true; //set to false to use MegaTag1

        // evaluating which Megatag one or two to use based on above boolean value and 
        // only incorporate Limelight's estimates when more than one tag is visible (tagcount >= 1)
        if (useMegaTag2 == false)
        {
            for (String limelightName : Constants.Targeting.CAMERA_NAMES) {
                boolean useThisEstimate = true;
                LimelightHelpers.PoseEstimate estimateMT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

                if (estimateMT1 == null) {
                    break; // if estimate is null, skip to next limelight (this is to prevent errors/crashes when limelight is not detected for a split second, which happens sometimes when starting robot/building code)
                }

                if((estimateMT1.tagCount == 1 && estimateMT1.rawFiducials.length == 1))
                {   
                    if(estimateMT1.rawFiducials[0].ambiguity > .7) { useThisEstimate = false; }
                    if(estimateMT1.rawFiducials[0].distToCamera > 3) { useThisEstimate = false; }
                }

                if(estimateMT1.tagCount == 0) { // if no tags are visible, ignore vision updates
                    useThisEstimate = false; 
                }

                if(useThisEstimate) {
                    m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,0.01));
                    m_poseEstimator.addVisionMeasurement(
                        estimateMT1.pose,
                        estimateMT1.timestampSeconds);
                }
            }       
        } else if (useMegaTag2) {
            for (String limelightName : Constants.Targeting.CAMERA_NAMES) {   
                boolean useThisEstimate = true;
                // only incorporate Limelight's estimates when more than one tag is visible (tagcount >= 1)
                // line below is required because megatag2 requires the limelight to know the robot's current rotation, as it USES it instead of providing it like in MT1
                LimelightHelpers.SetRobotOrientation(limelightName, m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
                LimelightHelpers.PoseEstimate estimateMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
                
                // sometimes when starting robot/building code, mt2 == null for a split second, so need to check for that or code errors and crashes
                if (estimateMT2 == null) {
                    break;
                }
                
                if (Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720 || estimateMT2.tagCount == 0) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
                {
                    useThisEstimate = false;
                }

                if (useThisEstimate)
                {
                    // n3 (yaw) set to high number because MegaTag2 uses robot's yaw instead of getting it for you
                    m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7, 999999)); // n3 was 9999999 
                    m_poseEstimator.addVisionMeasurement(
                        estimateMT2.pose,
                        estimateMT2.timestampSeconds);
                }
            }
        }
    }

    public double calculateTargetingPID (double HUBX, double HUBY) {
      // CALCULATED THE ANGLE TO FACE THE CENTER OF THE HUB
      Pose2d currentPose = getPose();

    // --------------------------------------------------
    // RATHER HAVE THIS IN TWO SEPERATE COMANDS
    // --------------------------------------------------

    //   boolean shouldPass = false;
    //   var alliance = DriverStation.getAlliance();

    //   if (alliance.isPresent()){
    //     if (alliance.get() == Alliance.Red) {
    //         if (currentPose.getX() < Units.inchesToMeters(Constants.Targeting.RED_ALLIANCE_HUB_CENTER_X - Constants.Targeting.ROBOT_WIDTH_INCHES * 1.5)) {
    //             shouldPass = true;
    //         }
    //     } else {
    //         if (currentPose.getX() > Units.inchesToMeters(Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_X + Constants.Targeting.ROBOT_WIDTH_INCHES * 1.5)) {
    //             shouldPass = true;
    //         }
    //     }
    //   }

    //   if (shouldPass) {
    //     if (alliance.get() == Alliance.Red) {
    //         return pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), 0); // if we have passed the hub, just face backwards
    //     } else {
    //         return pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), Math.toRadians(180)); // if we have passed the hub, just face backwards
    //     }
    //   }

      double dx = HUBX - Units.metersToInches(currentPose.getX());
      double dy = HUBY - Units.metersToInches(currentPose.getY());
      double targetAngle = Math.atan2(dy, dx);

      // USES PID TO ROTATE THE ROBOT EFFECTIVELY
      double pidOutput = pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), targetAngle);
      return pidOutput;
    }

    public double calculateFaceAlliancePID() {
        Pose2d currentPose = getPose();
        double targetAngle;
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            targetAngle = 0; // opposite of blue
        } else {
            targetAngle = Math.toRadians(180); // facing 180 on blue will face towards drivers
        }

        double pidOutput = pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), targetAngle);
        return pidOutput;
    }

    public double calculateTargetingAutoPID(double targetAngle) {
        Pose2d currentPose = getPose();
        double pidOutput = pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), targetAngle);
    
        return pidOutput;
    }

    public double getAngleOfHub (double HUBX, double HUBY) {
        Pose2d currentPose = getPose();
        double dx = HUBX - Units.metersToInches(currentPose.getX());
        double dy = HUBY - Units.metersToInches(currentPose.getY());
        double angle = Math.atan2(dy, dx);

        return angle;
    }

    public double getHubX() {
        return cachedHubX;
    }

    public double getHubY() {
        return cachedHubY;
    }

   public double getDistanceToHub() {

    Optional<Alliance> alliance = DriverStation.getAlliance();
    double HUBX;
    double HUBY;

    if (alliance.isPresent()) {
           if (alliance.get() == Alliance.Red) {
                HUBX = Constants.Targeting.RED_ALLIANCE_HUB_CENTER_X;
                HUBY = Constants.Targeting.RED_ALLIANCE_HUB_CENTER_Y;
           } else {
                HUBX = Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_X;
                HUBY = Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_Y;
           }
           // CALCULATED THE DISTANCE TO THE CENTER OF THE HUB
            Pose2d currentPose = getPose();
            double dx = HUBX - Units.metersToInches(currentPose.getX());
            double dy = HUBY - Units.metersToInches(currentPose.getY());
            double distance = Math.hypot(dy, dx);
        
            return distance;
       }
      return 0;
   }

   // THIS IS A DUPLICATE OF calculateTargetingAutoPID (DONT KNOW??)
   public double pidCalculateAngle (double targetAngle) {
   Pose2d currentPose = getPose();
   double pidOutput = pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), targetAngle);
  
   return pidOutput;
  }

    public double[] getRotationMoving () {
        ChassisSpeeds robotChassisSpeeds = getChassisSpeeds();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotChassisSpeeds, getHeading());


        Pose2d currentPose2d = getPose();


        double hubXMeters = Units.inchesToMeters(getHubX());
        double hubYMeters = Units.inchesToMeters(getHubY());


        double distanceToHubInches = getDistanceToHub();
        double finalDistanceInches = 0;


        double timeOfFlight = Constants.Targeting.timeMap.get(distanceToHubInches);


        Translation2d virtualGoal = new Translation2d();


        for (int i = 0; i < 2; i++) {
            // Virtual goal in meter: For the pose.
            double virtualX = hubXMeters - (fieldSpeeds.vxMetersPerSecond * timeOfFlight);
            double virtualY = hubYMeters - (fieldSpeeds.vyMetersPerSecond * timeOfFlight);


            virtualGoal = new Translation2d(virtualX, virtualY);


            double newDistanceMeters = currentPose2d.getTranslation().getDistance(virtualGoal);


            // Back to inches for the timeMap
            double newDistanceInches = Units.metersToInches(newDistanceMeters);
            timeOfFlight = Constants.Targeting.timeMap.get(newDistanceInches);
            finalDistanceInches = newDistanceInches;
        }


        // is now in radians.
        Rotation2d finalRot = new Rotation2d(
            virtualGoal.getX() - currentPose2d.getX(),
            virtualGoal.getY() - currentPose2d.getY()
        );


        SmartDashboard.putNumber("AimOnMove/VirtualGoalX (m)", virtualGoal.getX());
        SmartDashboard.putNumber("AimOnMove/VirtualGoalY (m)", virtualGoal.getY());
        SmartDashboard.putNumber("AimOnMove/FinalAngle (deg)", finalRot.getDegrees());
        SmartDashboard.putNumber("AimOnMove/DistanceToHub (in)", finalDistanceInches);
        SmartDashboard.putNumber("AimOnMove/FieldVx (m/s)", fieldSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("AimOnMove/FieldVy (m/s)", fieldSpeeds.vyMetersPerSecond);


    double[] returnData = {finalDistanceInches, finalRot.getRadians()};


    return returnData;
    }

       public boolean inNeutralZone() {
       Pose2d currentPose = getPose();

       Optional<Alliance> alliance = DriverStation.getAlliance();

       if (alliance.isPresent()) {
           if (alliance.get() == Alliance.Red) {
               return currentPose.getX() < Units.inchesToMeters(Constants.Targeting.RED_NEUTRAL_TOLERANCE_X);
           } else {
               return currentPose.getX() > Units.inchesToMeters(Constants.Targeting.BLUE_NEUTRAL_TOLERANCE_X);
           }
       }
       return false;
   }

    public double getAllianceWallHeading(double HUBX, double HUBY) {
        Pose2d currentPose = getPose();
        boolean shouldPass = false;
        var alliance = DriverStation.getAlliance();
        // double velocityThreshold = 0.2; 

        // ChassisSpeeds robotSpeeds = getChassisSpeeds();
        // Translation2d fieldRelativeVelocities = new Translation2d(
        //         robotSpeeds.vxMetersPerSecond, 
        //         robotSpeeds.vyMetersPerSecond
        // ).rotateBy(currentPose.getRotation());
        
        // double yVelocity = fieldRelativeVelocities.getY(); 
        double currentY = Units.metersToInches(currentPose.getY());

        if (alliance.isPresent()){
            if (alliance.get() == Alliance.Red) {
                if (currentPose.getX() < Units.inchesToMeters(Constants.Targeting.RED_NEUTRAL_TOLERANCE_X)) {
                    shouldPass = true;
                }
            } else {
                if (currentPose.getX() > Units.inchesToMeters(Constants.Targeting.BLUE_NEUTRAL_TOLERANCE_X)) {
                    shouldPass = true;
                }
            }
        }

        if (shouldPass) {
            
            if (inNeutralMid(Units.metersToInches(currentPose.getY()))) {

                if (alliance.get() == Alliance.Red) {
                    if(nuetralAimingTarget != null){
                        return pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), Math.toRadians(nuetralAimingTarget));
                    } else {
                        if (Math.abs(currentY - Constants.Targeting.RED_NEUTRAL_MID_RIGHT) <= Math.abs(currentY - Constants.Targeting.RED_NEUTRAL_MID_LEFT)){
                            nuetralAimingTarget = -45.0;
                            return pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), Math.toRadians(-45));
                        } else if (Math.abs(currentY - Constants.Targeting.RED_NEUTRAL_MID_RIGHT) > Math.abs(currentY - Constants.Targeting.RED_NEUTRAL_MID_LEFT)) {
                            nuetralAimingTarget = 45.0;
                            return pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), Math.toRadians(45));
                        }
                    }
                } else { 
                    if(nuetralAimingTarget != null){
                        return pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), Math.toRadians(nuetralAimingTarget));
                    } else {
                    if (Math.abs(currentY - Constants.Targeting.BLUE_NEUTRAL_MID_RIGHT) <= Math.abs(currentY - Constants.Targeting.BLUE_NEUTRAL_MID_LEFT)){
                        nuetralAimingTarget = -45.0;
                        return pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), Math.toRadians(-45));
                    } else if (Math.abs(currentY - Constants.Targeting.BLUE_NEUTRAL_MID_RIGHT) > Math.abs(currentY - Constants.Targeting.BLUE_NEUTRAL_MID_LEFT)) {
                        nuetralAimingTarget = 45.0;
                        return pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), Math.toRadians(45));
                    }
                    }
                }
                // // RED ALLIANCE LOGIC
                // if (alliance.get() == Alliance.Red) {
                //     if (yVelocity > velocityThreshold) {
                //         if(currentY > Units.inchesToMeters(Constants.Targeting.RED_NEUTRAL_MID_RIGHT) && currentY < Units.inchesToMeters(Constants.Targeting.RED_NEUTRAL_MID_LEFT)){
                //             return pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), Math.toRadians(-45));
                //         } 
                //     } 
                //     else if (yVelocity < -velocityThreshold) { 
                //         if (currentY < Units.inchesToMeters(Constants.Targeting.RED_NEUTRAL_MID_LEFT) && currentY > Units.inchesToMeters(Constants.Targeting.RED_NEUTRAL_MID_RIGHT)) { 
                //             return pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), Math.toRadians(45));
                //         }
                //     }
                // } 
                // // BLUE ALLIANCE LOGIC
                // else if (alliance.get() == Alliance.Blue) {
                //     if (yVelocity > velocityThreshold) {
                //         if(currentY < Units.inchesToMeters(Constants.Targeting.RED_NEUTRAL_MID_LEFT) && currentY > Units.inchesToMeters(Constants.Targeting.RED_NEUTRAL_MID_RIGHT)){ 
                //             return pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), Math.toRadians(-45));
                //         } 
                //     } 
                //     else if (yVelocity < -velocityThreshold) { 
                //         if (currentY > Units.inchesToMeters(Constants.Targeting.RED_NEUTRAL_MID_RIGHT) && currentY < Units.inchesToMeters(Constants.Targeting.RED_NEUTRAL_MID_LEFT)) { 
                //             return pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), Math.toRadians(45));
                //         }
                //     }
                // }
            }

            nuetralAimingTarget = null;

            // DEFAULT WALL AIMING
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                return pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), 0);
            } else {
                return pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), Math.toRadians(180));
            }
        }
        nuetralAimingTarget = null;
        // DEFAULT HUB AIMING
        double dx = HUBX - Units.metersToInches(currentPose.getX());
        double dy = HUBY - Units.metersToInches(currentPose.getY());
        double targetAngle = Math.atan2(dy, dx);

        double angleDifference = MathUtil.angleModulus(targetAngle - currentPose.getRotation().getRadians());

        if (Math.abs(angleDifference) < Units.degreesToRadians(1.0)) {
            RobotContainer.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
            RobotContainer.auxController.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
        } else {
            RobotContainer.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
            RobotContainer.auxController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
        }

        return pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), targetAngle);
    }

   public boolean inNeutralMid(double yPose) {
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                return yPose > Constants.Targeting.RED_NEUTRAL_MID_RIGHT && 
                       yPose < Constants.Targeting.RED_NEUTRAL_MID_LEFT;
            } else if (alliance.get() == Alliance.Blue) {
                return yPose > Constants.Targeting.BLUE_NEUTRAL_MID_RIGHT && 
                       yPose < Constants.Targeting.BLUE_NEUTRAL_MID_LEFT;
            }
        }
        return false;
    }


    public double getXtoHub(){
    Pose2d currentPose = getPose();

    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
        if (alliance.get() == Alliance.Red) {
            return Math.abs(currentPose.getX() - Constants.Targeting.RED_ALLIANCE_HUB_CENTER_X);
        } else {
            return Math.abs(currentPose.getX() - Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_X);
        }
    }
        return 0.0;
    }

    @Override
    public void periodic(){
        //SmartDashboard.putNumber("limelight standoff fwd", LimelightHelpers.getTargetPose_CameraSpace("limelight")[2]);

        //swerveOdometry.update(getGyroYaw(), getModulePositions());
        MegaTag2UpdateOdometry();
        SmartDashboard.putNumber("auto pivot desired rotation (red)", Units.radiansToDegrees(getAngleOfHub(Constants.Targeting.RED_ALLIANCE_HUB_CENTER_X, Constants.Targeting.RED_ALLIANCE_HUB_CENTER_Y)));
        SmartDashboard.putNumber("auto pivot current rotation (red)", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Distance to hub", getDistanceToHub());
        SmartDashboard.putNumber("** RobotPoseX (Estimator)", Units.metersToInches( m_poseEstimator.getEstimatedPosition().getX()));
        SmartDashboard.putNumber("** RobotPoseY (Estimator)", Units.metersToInches( m_poseEstimator.getEstimatedPosition().getY()));
        SmartDashboard.putNumber("MegaTag2Rotation (Estimator)", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
       SmartDashboard.putBoolean("Is in nuetral", inNeutralZone());
        //System.out.println(swerveOdometry.getPoseMeters().getX() + " " + swerveOdometry.getPoseMeters().getY() + " Rotation: " + swerveOdometry.getPoseMeters().getRotation().getDegrees());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder degrees", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle degrees", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            // Can't use m/s in the key!! SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity m/s", mod.getState().speedMetersPerSecond);
        }

        field.setRobotPose(getPose());
        // var traj = field.getObject("trajectory");
        // traj.setPoses(
        //     getPose(), 
        //     getPose().plus(
        //         new Transform2d(
        //             new Translation2d(5, 0), // 1 meter straight ahead of the robot
        //             new Rotation2d() // same rotation as robot
        //         )
        //     )
        //     ); // example trajectory visualization, replace with actual trajectory if desired
        // SmartDashboard.putData("Field", field);
    }
}