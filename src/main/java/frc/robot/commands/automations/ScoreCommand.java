package frc.robot.commands.automations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.HashMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * A command that automatically aligns the robot to a target pose, actuates the elevator and wrist, scores, and retracts.
 * This command is used for scoring at the reef on any level, from L1-L4.
 */
public class ScoreCommand extends SequentialCommandGroup {

    // Poses
    private Pose2d targetPose;

    // Paths
    private AutoAlignPath path;

    // Profiles
    private TrapezoidProfile.State translationalState = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State rotationalState = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State targetTranslationalState = new TrapezoidProfile.State(0, 0); // The position is actually the error.
    private TrapezoidProfile.State targetRotationalState = new TrapezoidProfile.State(0, 0);

    private TrapezoidProfile.Constraints translationConstraints;
    private TrapezoidProfile.Constraints rotationConstraint;
    private TrapezoidProfile translationProfile;
    private TrapezoidProfile rotationProfile;

    // Subsystems
    private SwerveSubsystem drivetrain;
    private Wrist wrist;
    private Elevator elevator;
    private PipeGrabber grabber;

    // Levels
    ReefSide side;
    ReefIndex location;
    ReefLevel level;

    // Targets
    private double elevatorTarget;
    private double wristTarget;


    /** Constructor for our scoring command */
    public ScoreCommand(Wrist wrist, Elevator elevator, PipeGrabber grabber, SwerveSubsystem drivetrain, 
                            ReefSide side, ReefIndex location, ReefLevel level) {

        this.drivetrain = drivetrain;
        this.wrist = wrist;
        this.elevator = elevator;
        this.grabber = grabber;

        this.side = side;
        this.location = location;
        this.level = level;

        setTargets();

        if(DriverStation.isAutonomous()){
            super.addCommands(new ParallelCommandGroup(getGrabberIntake(), getDriveToPrescore()),
                                new ParallelCommandGroup(getDriveInCommand(), getActuateCommand()).withTimeout(1.5), 
                                getScoreCommand());
        }
        else{
            super.addCommands(new ParallelCommandGroup(getDriveInCommand(), getActuateCommand()).withTimeout(3.5), 
                                getScoreCommand(), getDeactuationCommand());
        }
        super.addRequirements(wrist, elevator, grabber, drivetrain);
    }

    /** Runs a grabber intake during auto, which decreases wait time at the station */
    public Command getGrabberIntake(){
        return new RunCommand(
            () -> grabber.runIntakeMaxSpeed(),
            grabber).withTimeout(0.4)
            .andThen(() -> grabber.stop());
    }

    /** Helper method for following a straight trajectory with a trapezoidal profile */
    public Command getFollowPath(AutoAlignPath path, double finalVelocity){
        final Command followPath = new FunctionalCommand(
            () -> {
            // Resets profiles and states
            translationConstraints = new Constraints(path.maxVelocity, path.maxAcceleration);
            rotationConstraint = new Constraints(path.maxAngularVelocity, path.maxAngularAcceleration);

            translationProfile = new TrapezoidProfile(translationConstraints);
            rotationProfile = new TrapezoidProfile(rotationConstraint);

            translationalState.position = path.getDisplacement().getNorm();
            translationalState.velocity = MathUtil.clamp(((-1) * (drivetrain.getFieldVelocity().vxMetersPerSecond * path.getDisplacement().getX() 
                                                                + drivetrain.getFieldVelocity().vyMetersPerSecond * path.getDisplacement().getY())/ translationalState.position),
                                                                -path.maxVelocity,
                                                                0);

            targetTranslationalState.velocity = finalVelocity;

            targetRotationalState.position = path.endPose.getRotation().getRadians();
            rotationalState.position = drivetrain.getSwerveDrive().getPose().getRotation().getRadians();
            rotationalState.velocity = drivetrain.getSwerveDrive().getRobotVelocity().omegaRadiansPerSecond;
        },
            () -> {
                // Move.
                Setpoint newSetpoint = getNextSetpoint(path);
                drivetrain.followSegment(newSetpoint, path.endPose);
                if(drivetrain.getPose().getTranslation().getDistance(path.endPose.getTranslation()) < 0.02
                    && Math.abs(drivetrain.getPose().getRotation().getRadians() - path.endPose.getRotation().getRadians()) < 0.017
                ){
                    // Stop.
                    drivetrain.drive(new ChassisSpeeds(0, 0, 0));
                }
            },
            (interrupted) -> {},
            () -> drivetrain.getPose().getTranslation().getDistance(path.endPose.getTranslation()) < 0.02
            && Math.abs(drivetrain.getPose().getRotation().getRadians() - path.endPose.getRotation().getRadians()) < 0.017
        );
        return followPath;

    }

    /** Drives close to our target pose during auto */
    public Command getDriveToPrescore(){
        path = new AutoAlignPath(
            drivetrain.getPose(),
            getPrescore(targetPose),
            DrivebaseConstants.kMaxAutoVelocity,
            DrivebaseConstants.kMaxAutoAcceleration,
            DrivebaseConstants.kMaxOmega,
            DrivebaseConstants.kMaxAlpha);
        final Command driveToPrescore = getFollowPath(path, 2.1).until(
            () -> drivetrain.getPose().getTranslation().getDistance(getPrescore(targetPose).getTranslation()) < 0.2
        );
        return driveToPrescore;
    }

    /** Snaps to the reef pose */
    public Command getDriveInCommand(){
        path = new AutoAlignPath(
            drivetrain.getPose(),
            targetPose, //originally just target pose
            DrivebaseConstants.kMaxAlignmentVelocity,
            DrivebaseConstants.kMaxAlignmentAcceleration,
            DrivebaseConstants.kMaxOmega,
            DrivebaseConstants.kMaxAlpha);
        final Command driveIn = getFollowPath(path, 0);
        return driveIn;
    }

    /** Actuates superstructure to our desired level */
    public Command getActuateCommand(){
        final Command actuateToPosition = new FunctionalCommand(
            () -> {
            },
            () -> {
                if(drivetrain.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.95){
                    elevator.setElevatorPosition(elevatorTarget);
                    wrist.setMotorPosition(wristTarget);
                }
            },
            (interrupted) -> {},
            () ->  Math.abs(elevator.getElevatorHeight() - elevatorTarget) < ElevatorConstants.kElevatorTolerance
        );
        return actuateToPosition;
    }

    /** Scores the piece */
    public Command getScoreCommand(){
        final Command dunkAndScore = new FunctionalCommand(
            () -> {
            },
            () -> {
                grabber.runGrabberOuttake();
            },
            (interrupted) -> {
                grabber.stop();
            },
            () -> false
        );
        return dunkAndScore.withTimeout(0.25);
    }

    /** Deactuates the superstructure in teleop for driver QOL */
    public Command getDeactuationCommand(){
        path = new AutoAlignPath(
            drivetrain.getPose(),
            getPrescore(targetPose),
            DrivebaseConstants.kMaxAlignmentVelocity, 
            DrivebaseConstants.kMaxAlignmentAcceleration,
            DrivebaseConstants.kMaxOmega,
            DrivebaseConstants.kMaxAlpha);
        final Command deactuateSuperstructure = new FunctionalCommand(
                        () -> {
                        },
                        () -> { 
                            wrist.setMotorPosition(WristConstants.WristStates.STOW);
                            elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
                        },
                        (interrupted) -> {},
                        () -> (Math.abs(elevator.getElevatorHeight() - ElevatorConstants.ElevatorStates.STOW) < ElevatorConstants.kElevatorTolerance) 
        );
        // final Command backUp = getFollowPath(path, 0);
        return Commands.parallel(deactuateSuperstructure).withTimeout(0.5);
    }

    /** Gets the prescore for a specific Pose2d */
    public Pose2d getPrescore(Pose2d targetPose){
        return new Pose2d(
            targetPose.getX() + 1 * Math.cos(targetPose.getRotation().getRadians()), 
            targetPose.getY() + 1 * Math.sin(targetPose.getRotation().getRadians()), 
            targetPose.getRotation()
        );
    }

    /** Sets the target for the robot, including target pose, elevator height, and arm angle */
    public void setTargets(){
        DriverStation.Alliance alliance;
        if(!DriverStation.getAlliance().isPresent()){
            alliance = DriverStation.Alliance.Blue;
        }
        else alliance = DriverStation.getAlliance().get();

        int multiplier = (side == ReefSide.LEFT) ? -1 : 1;
        final double DISTANCE_FROM_TAG = 0.164;
        HashMap<ReefIndex, Pose2d> targetPoses = new HashMap<ReefIndex, Pose2d>();

        // All target poses are in meters
        targetPoses.put(ReefIndex.BOTTOM_RIGHT, new Pose2d(13.530, 2.614, Rotation2d.fromDegrees(300)));
        targetPoses.put(ReefIndex.FAR_RIGHT, new Pose2d(14.54, 3.720, Rotation2d.fromDegrees(0)));
        targetPoses.put(ReefIndex.TOP_RIGHT, new Pose2d(14.064, 5.155, Rotation2d.fromDegrees(60)));
        targetPoses.put(ReefIndex.TOP_LEFT, new Pose2d(12.640, 5.361, Rotation2d.fromDegrees(120))); 
        targetPoses.put(ReefIndex.FAR_LEFT, new Pose2d(11.784, 4.339, Rotation2d.fromDegrees(180)));
        targetPoses.put(ReefIndex.BOTTOM_LEFT, new Pose2d(12.08, 2.908, Rotation2d.fromDegrees(240)));

        // Puts a HashMap of all possible april tag positions. Notice this is viewed top down with the barge to the left.
        Pose2d aprilTagPose = targetPoses.get(location);

        targetPose = new Pose2d( 
            aprilTagPose.getX() + (DISTANCE_FROM_TAG * Math.cos(aprilTagPose.getRotation().getRadians() + Math.PI/2) * multiplier),
            aprilTagPose.getY() + (DISTANCE_FROM_TAG * Math.sin(aprilTagPose.getRotation().getRadians() + Math.PI/2) * multiplier),
            aprilTagPose.getRotation()
        );
        
        if(alliance == DriverStation.Alliance.Blue){
            targetPose = applyAllianceTransform(targetPose);
        }

        switch (level) {
            case L1:
                wristTarget = WristConstants.WristStates.L1;
                elevatorTarget = ElevatorConstants.ElevatorStates.STOW;
                break;

            case L2:
                wristTarget = WristConstants.WristStates.L2;
                elevatorTarget = ElevatorConstants.ElevatorStates.L2;
                break;

            case L3:
                wristTarget = WristConstants.WristStates.L2;
                elevatorTarget = ElevatorConstants.ElevatorStates.L3;
                break;

            case L4: 
                wristTarget = WristConstants.WristStates.L4;
                elevatorTarget = ElevatorConstants.ElevatorStates.L4;
                break;

            default:
                wristTarget = WristConstants.WristStates.STOW;
                elevatorTarget = ElevatorConstants.ElevatorStates.STOW;
                break;
        }
    }

    /** Transforms red alliance poses to blue by reflecting around the center point of the field*/
    public Pose2d applyAllianceTransform(Pose2d pose){
        return new Pose2d(
            pose.getX() - 2 * (pose.getX() - 8.75),
            pose.getY() - 2 * (pose.getY() - 4.0),
            pose.getRotation().rotateBy(Rotation2d.fromDegrees(180))
        );
    }

    /** Gets a setpoint for the robot PID to follow.
     * @return A Setpoint object representing the next target robot state on a certain auto align path.
     */
    public Setpoint getNextSetpoint(AutoAlignPath path){
        // trans
        State translationSetpoint = translationProfile.calculate(0.02, translationalState, targetTranslationalState);
        translationalState.position = translationSetpoint.position;
        translationalState.velocity = translationSetpoint.velocity;

        // rot wraparound calculations
        double rotationalPose = drivetrain.getSwerveDrive().getPose().getRotation().getRadians();
        double goalRotation = targetRotationalState.position;
        double angularError = MathUtil.angleModulus(goalRotation - rotationalPose);

        targetRotationalState.position = rotationalPose + angularError;
        double setpointError = MathUtil.angleModulus(rotationalState.position - rotationalPose);
        rotationalState.position = rotationalPose + setpointError;
        // rot
        State rotationalSetpoint = rotationProfile.calculate(0.02, rotationalState, targetRotationalState);
        rotationalState.position = rotationalSetpoint.position;
        rotationalState.velocity = rotationalSetpoint.velocity;

        // arc length parametrization for a line
        Translation2d interpolatedTranslation = path.endPose
            .getTranslation()
            .interpolate(path.startPose.getTranslation(), 
            translationSetpoint.position / path.getDisplacement().getNorm());

        return new Setpoint(
            interpolatedTranslation.getX(),
            interpolatedTranslation.getY(),
            rotationalState.position,
            path.getNormalizedDisplacement().getX() * -translationSetpoint.velocity, 
            path.getNormalizedDisplacement().getY() * -translationSetpoint.velocity,
            rotationalState.velocity
        );
    }
}
