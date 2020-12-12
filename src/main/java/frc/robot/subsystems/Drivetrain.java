package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.operator.controllers.BionicF310;

public class Drivetrain extends SubsystemBase {

    private final int TICKS_PER_ROTATION = 2048; //prev. 1440 - Link: http://www.ctr-electronics.com/talon-fx.html#product_tabs_tech_specs 
    private final int TIMEOUT_MS = 30; // The amount of time the Roborio watis before it sends a failed signal back to
                                       // the Driver Station
    private final int PID_INDEX = 0; // The index for witch pid index to use (0 for primary, 1 for auxilliary)
    private final double MOTOR_DEADBAND_PERCENTAGE = 0.001;
    private final double NOMINAL_OUTPUT = 0;
    private final double NOMINAL_OUTPUT_REVERSE = -NOMINAL_OUTPUT; 
    private final double PEAK_OUTPUT = 1;
    private final double PEAK_OUTPUT_REVERSE = -PEAK_OUTPUT;
    private final double WHEEL_CIRCUMFERENCE_FEET = (6 * Math.PI) / 12;
    private final int SENSOR_POS = 0;
    private final int MAX_CRUISE_VELOCITY = 15000;
    private final int MAX_CRUISE_ACCEL = 6000; //TODO test values.
    private final double SENSOR_UNITS = 0;
    private final int MAX_RPM = 6380;

    //Formula from https://readthedocs.org/projects/phoenix-documentation/downloads/pdf/latest/ pg 149
    private final int CRUISE_VELOCITY = (MAX_RPM / 2 / 600) * TICKS_PER_ROTATION;
    private final int CRUISE_ACCELERATION = CRUISE_VELOCITY / 2; // last number dictates how long it'll take us to get to max speed.
    private final int SMOOTHING = 3; //0 -> 8: CHOOSE WITH TESTING. TODO May be controlled by a button.

    private double lastDriveSpeed = 0;

    private final double K_P = 0.25; // A gain of 0.25 maximizes the motor output when the error is one revolution.
    private final double K_I = 0;
    private final double K_D = 0;

    private WPI_TalonFX left_front, left_back, right_front, right_back;
    private SpeedController m_left, m_right;
    private SpeedControllerGroup left_side, right_side;
    private DifferentialDrive diffDrive;
    private PIDController pidController;

    public Drivetrain(int idbl, int idfl, int idfr, int idbr) {
        this.left_front = new WPI_TalonFX(idfl);
        this.right_front = new WPI_TalonFX(idfr);
        this.right_back = new WPI_TalonFX(idbr);
        this.left_back = new WPI_TalonFX(idbl);

        
        // Invert one side of the drivetrain to ensure that positive values indicate
        // forward motion.
        left_back.setInverted(TalonFXInvertType.CounterClockwise);
        left_front.setInverted(TalonFXInvertType.CounterClockwise);
        right_back.setInverted(TalonFXInvertType.Clockwise);
        right_front.setInverted(TalonFXInvertType.Clockwise);
        
        //Make the left back & right back motors follow the left front & right front motors
        left_back.follow(left_front);
        right_back.follow(right_front);

        this.diffDrive = new DifferentialDrive(left_front, right_front);

        // Configure Talon FXs

        // Reset Factory Defaults
        left_back.configFactoryDefault();
        left_front.configFactoryDefault();
        right_back.configFactoryDefault();
        right_front.configFactoryDefault();

        // Config the encoder to be the integrated encoder in the TalonFX
        left_back.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, TIMEOUT_MS);
        left_front.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, TIMEOUT_MS);
        right_back.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, TIMEOUT_MS);
        right_front.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, TIMEOUT_MS);

        // Set deadbands to the default value.
        // This configures the sensitivity of the motors.
        left_back.configNeutralDeadband(MOTOR_DEADBAND_PERCENTAGE);
        left_front.configNeutralDeadband(MOTOR_DEADBAND_PERCENTAGE);
        right_back.configNeutralDeadband(MOTOR_DEADBAND_PERCENTAGE);
        right_front.configNeutralDeadband(MOTOR_DEADBAND_PERCENTAGE);


        // Sets the minimum motor output.
        // Also sets the maximum motor output.
        {
            left_front.configPeakOutputForward(PEAK_OUTPUT);
            left_front.configPeakOutputReverse(PEAK_OUTPUT_REVERSE);
            left_front.configNominalOutputForward(NOMINAL_OUTPUT);
            left_front.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE);

            left_back.configPeakOutputForward(PEAK_OUTPUT);
            left_back.configPeakOutputReverse(PEAK_OUTPUT_REVERSE);
            left_back.configNominalOutputForward(NOMINAL_OUTPUT);
            left_back.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE);

            right_front.configPeakOutputForward(PEAK_OUTPUT);
            right_front.configPeakOutputReverse(PEAK_OUTPUT_REVERSE);
            right_front.configNominalOutputForward(NOMINAL_OUTPUT);
            right_front.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE);

            right_back.configPeakOutputForward(PEAK_OUTPUT);
            right_back.configPeakOutputReverse(PEAK_OUTPUT_REVERSE);
            right_back.configNominalOutputForward(NOMINAL_OUTPUT);
            right_back.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE);
        }
        
        // Sets the P constant given the parameter
        left_front.config_kP(PID_INDEX, K_P);
        left_back.config_kP(PID_INDEX, K_P);
        right_front.config_kP(PID_INDEX, K_P);
        right_back.config_kP(PID_INDEX, K_P);

        // Sets the D constant given the parameter
        left_front.config_kI(PID_INDEX, K_I);
        left_back.config_kI(PID_INDEX, K_I);
        right_front.config_kI(PID_INDEX, K_I);
        right_back.config_kI(PID_INDEX, K_I);

        // Sets the D constant given the parameter
        left_front.config_kD(PID_INDEX, K_D);
        left_back.config_kD(PID_INDEX, K_D);
        right_front.config_kD(PID_INDEX, K_D);
        right_back.config_kD(PID_INDEX, K_D);

        
        //Sets the sensor position to 0
        left_front.setSelectedSensorPosition(SENSOR_POS);
        left_back.setSelectedSensorPosition(SENSOR_POS);
        right_front.setSelectedSensorPosition(SENSOR_POS);
        right_back.setSelectedSensorPosition(SENSOR_POS);

        //Sets acceleration and cruise velocity << motion magic
        left_front.configMotionCruiseVelocity(CRUISE_VELOCITY, TIMEOUT_MS);
        left_back.configMotionCruiseVelocity(CRUISE_VELOCITY, TIMEOUT_MS);
        right_front.configMotionCruiseVelocity(CRUISE_VELOCITY, TIMEOUT_MS);
        right_back.configMotionCruiseVelocity(CRUISE_VELOCITY, TIMEOUT_MS);

        left_front.configMotionAcceleration(CRUISE_ACCELERATION, TIMEOUT_MS);
        left_back.configMotionAcceleration(CRUISE_ACCELERATION, TIMEOUT_MS);
        right_front.configMotionAcceleration(CRUISE_ACCELERATION, TIMEOUT_MS);
        right_back.configMotionAcceleration(CRUISE_ACCELERATION, TIMEOUT_MS);

        left_front.configMotionSCurveStrength(SMOOTHING);
        left_back.configMotionSCurveStrength(SMOOTHING);
        right_front.configMotionSCurveStrength(SMOOTHING);
        right_back.configMotionSCurveStrength(SMOOTHING);
    }

    /*
     * public void driveForward(double speed) { diffDrive.arcadeDrive(speed, 0);
     * left_front.configFactoryDefault(); }
     * 
     * // pid values Branch: Drivetrain Motion profiling, File; DrivetrainSubsystem.
     * public void driveFeet(double speed, double distance) {
     * 
     * driveForward(speed);
     * 
     * }
     */

    //Make the robot move and turn. Used for manual control
    public void driveManual(double speed, double rotation){

        diffDrive.arcadeDrive(speed, rotation);
        
    }

    public void driveManualWithRamp(double speed, double rotation){

        // TODO test the ramping
        if(Math.abs(speed - lastDriveSpeed) >= 0.5){ // TODO make a constant
            speed += Math.signum(speed - lastDriveSpeed) * 0.5;
        }
        
        diffDrive.arcadeDrive(speed, rotation);
        //setting the last speed (20 ms before) of the drivetrain to lastDriveSpeed
        lastDriveSpeed = speed;
    }
    

    //Make the robot drive a certain amout of feet forward along a given arc - UNUSED
    public void driveDistanceArc(double speed, double rotation_deg, double distance) { // Distance is in feet
        
        // Create known arc.
        double totalRevolutions = distance / WHEEL_CIRCUMFERENCE_FEET;
        double ticksToMove = totalRevolutions * TICKS_PER_ROTATION;
        
    }
    
    public void driveDistance(double distance) {

        double totalRevolutions = distance / WHEEL_CIRCUMFERENCE_FEET; //This takes the amount of feet to move, and returns the amount of wheel rotations
        int ticksToMove = (int)(totalRevolutions * TICKS_PER_ROTATION); //This takes the amount of wheel rotations and returns the amount of ticks to move
        int lCurrentEncoderPos = left_front.getSelectedSensorPosition(); //This returns the amount of ticks the encoder is on
        int rCurrentEncoderPos = right_front.getSelectedSensorPosition(); //This returns the amount of ticks the encoder is on
        
        left_front.set(ControlMode.Position, lCurrentEncoderPos + ticksToMove);
        right_front.set(ControlMode.Position, rCurrentEncoderPos + ticksToMove);
    }

    public void driveDistanceMagic(double distance) {

        double totalRevolutions = distance / WHEEL_CIRCUMFERENCE_FEET; //This takes the amount of feet to move, and returns the amount of wheel rotations
        int ticksToMove = (int)(totalRevolutions * TICKS_PER_ROTATION); //This takes the amount of wheel rotations and returns the amount of ticks to move
        int lCurrentEncoderPos = left_front.getSelectedSensorPosition(); //This returns the amount of ticks the encoder is on
        int rCurrentEncoderPos = right_front.getSelectedSensorPosition(); //This returns the amount of ticks the encoder is on

        //TODO see if we can implement motion magic control
        
        left_front.set(ControlMode.MotionMagic, lCurrentEncoderPos + ticksToMove);
        right_front.set(ControlMode.MotionMagic, rCurrentEncoderPos + ticksToMove);
    }

    @Override
    public void periodic() {

        super.periodic();

        // drive the distance configured in driveDistance
        if(left_front.getControlMode() == TalonFXControlMode.Position.toControlMode()) {
            //TODO Find out how close we need to be in terms of ticks
            //Error is in ticks
            if (Math.abs(left_front.getClosedLoopError()) < TICKS_PER_ROTATION * 1) {
                // error is in acceptable range
                //Stops the motor by setting the voltage to 0
                left_front.set(ControlMode.PercentOutput, 0);
                right_front.set(ControlMode.PercentOutput, 0);
            } else {
                // not there yet
                //feed the motor safety watchdog
                left_front.feed();
                right_front.feed();
            }
            
        } else {
            //TODO ? Arcade drive periodic

            //do what the joystick says
            // Scheduler scheduler = Scheduler.getInstance();
            this.driveManual(Robot.driverGamepad.getThresholdAxis(BionicF310.LY), Robot.driverGamepad.getThresholdAxis(BionicF310.LX));
            // returns null
            // Command currentCommand = CommandScheduler.getInstance().requiring(this);
            // CommandScheduler.getInstance().
            // https://docs.wpilib.org/en/stable/docs/software/commandbased/command-scheduler.html
            
        }
    
    }



}