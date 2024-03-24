package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

  private int inputDelayCtr;
  private double kMaxOutput, kMinOutput;
  public boolean flyWheelsRunning;
  
  private static double leftCmdWheelRate;
  private static double rightCmdWheelRate; 

  private CANSparkMax leftLaunchWheel;
  private CANSparkMax rightLaunchWheel;

  private SparkPIDController m_leftLancherPIDCtrl;
  private RelativeEncoder m_leftEncoder;
  public static double kLP, kLI, kLD, kLIz, kLFF ;

  private SparkPIDController m_rightLancherPIDCtrl;
  private RelativeEncoder m_rightEncoder;
  public double kRP, kRI, kRD, kRIz, kRFF ;
  
  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {
    // Set private holding variables: -----------------------------------------------------|
    // launcher status
    ShuffleboardTab LaunchTab = Shuffleboard.getTab("Launch Subsystem");
    GenericEntry FWRuningEntry = LaunchTab.add("Fly Wheels Running", false).getEntry();

    flyWheelsRunning = false;
    inputDelayCtr = 0;

    // current running power
    leftCmdWheelRate = Constants.Launcher.kLeftCmdRate; // RPM
    rightCmdWheelRate = Constants.Launcher.kRightCmdRate;

    // Set SparkMax motor limits
    kMaxOutput = 1; 
    kMinOutput = -1;

    // initialize the left motor PID coefficients
    kLP = 6e-5; 
    kLI = 0;
    kLD = 0; 
    kLIz = 0; 
    kLFF = 1.0; 

    // initialize the right motor PID coefficients
    kRP = 6e-5; 
    kRI = 0;
    kRD = 0; 
    kRIz = 0; 
    kRFF = 0.0004; 

    // create two new SPARK MAXs and configure them ---------------------------------------|
    // 1 motor for the left launcher wheel: -------------------------------------------||
    leftLaunchWheel =
        new CANSparkMax(Constants.Launcher.kLTSCanId, CANSparkLowLevel.MotorType.kBrushless);

    leftLaunchWheel.setInverted(false);
    leftLaunchWheel.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
    leftLaunchWheel.setIdleMode(IdleMode.kBrake);
    
    // create a PID controller for the left launcher motor 
    m_leftLancherPIDCtrl = leftLaunchWheel.getPIDController();

    // encoder object created to display position and velocity values for the left motor
    m_leftEncoder = leftLaunchWheel.getEncoder();
    m_leftLancherPIDCtrl.setFeedbackDevice(m_leftEncoder);
    m_leftEncoder.setPositionConversionFactor(1.0); // rotations
    m_leftEncoder.setVelocityConversionFactor(1.0); // rpm

    // configure the left motor PID controller
    m_leftLancherPIDCtrl.setP(kLP);
    m_leftLancherPIDCtrl.setI(kLI);
    m_leftLancherPIDCtrl.setD(kLD);
    m_leftLancherPIDCtrl.setIZone(kLIz);
    m_leftLancherPIDCtrl.setFF(kLFF);
    m_leftLancherPIDCtrl.setOutputRange(kMinOutput, kMaxOutput);

    // Push left motor configuration to the left motor flash memory.
    leftLaunchWheel.burnFlash();

    // 1 motor for the right launcher wheel: -------------------------------------------||
    rightLaunchWheel =
    new CANSparkMax(Constants.Launcher.kRTSCanId, CANSparkLowLevel.MotorType.kBrushless);

    rightLaunchWheel.setInverted(true);
    rightLaunchWheel.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
    rightLaunchWheel.setIdleMode(IdleMode.kBrake);

    // create a PID controller for the left launcher motor 
    m_rightLancherPIDCtrl = rightLaunchWheel.getPIDController();

    // encoder object created to display position and velocity values for the right motor
    m_rightEncoder = rightLaunchWheel.getEncoder();
    m_rightLancherPIDCtrl.setFeedbackDevice(m_rightEncoder);
    m_rightEncoder.setPositionConversionFactor(1.0); // rotations
    m_rightEncoder.setVelocityConversionFactor(1.0); // rpm

    // configure the left motor PID controller
    m_rightLancherPIDCtrl.setP(kRP);
    m_rightLancherPIDCtrl.setI(kRI);
    m_rightLancherPIDCtrl.setD(kRD);
    m_rightLancherPIDCtrl.setIZone(kRIz);
    m_rightLancherPIDCtrl.setFF(kRFF);
    m_rightLancherPIDCtrl.setOutputRange(kMinOutput, kMaxOutput);

    // push left motor configuration to the left motor flash memory.
    rightLaunchWheel.burnFlash();
  }

/**
* Constructs a command that starts the launcher and then runs the Intake feeder
* motor to put the Note up to the spinning launcher wheels. After a few more seconds
* the command will shutdown the Launcher Wheels and an the Intake feeder. This
* command takes control of the intake subsystem to make sure the feeder keeps
* running during the launch sequence.
*
* @param _intake The instance of the intake subsystem
* @return The launch command
*/

public Command launchNote(IntakeSubsystem _Intake) {
    Command launching =
        new Command() {
          
          private Timer m_timer;

          @Override
          public void initialize() {
            /* Start the Launcher Wheels and the Launch timer. */
            flyWheelsRunning = true;
            m_timer = new Timer();
            m_timer.start();
          }

          @Override
          public void execute() {
            /* Wait until the Launcher Wheels get up to speed,
             * then start the Intake Feeder to push the Note up
             * into the Launcher Wheels.
             */
            flyWheelsRunning = true;

             if(m_timer.get() > Constants.Launcher.kTimeToLaunch){
              _Intake.moveNote(Constants.Launcher.kFeederSpeed);
            }
          }

          @Override
          public boolean isFinished() {
            /* The Launcher and Intake feeder will stop after an
             * appropriate delay.
             */
            return m_timer.get() > Constants.Launcher.kTimeToStop;
          }

          @Override
          public void end(boolean interrupted) {
            /* Stop both the Launcher and the Intake feeder */
            flyWheelsRunning = false;
            _Intake.stopFeeder();
          }
        };

        return launching;
  }

/**
* Constructs a command that raises the commanded set point for 
* the left flywheel motor by 1000 rpm.
*
* @return The riseUp1000 command
*/
public void raiseLCR1000(){
  // Raise the Left Wheel Rate by 1000 RPM
  leftCmdWheelRate += 1000; 
};

/**
* Constructs a command that raises the commanded set point for 
* the left flywheel motor by 100 rpm.
*
* @return The riseUp1000 command
*/
public Command raiseLCR100(){
  Command riseUp100 = 
    new Command() {
      @Override
      public void initialize() {
        /* Start the Launcher Wheels and the Launch timer. */
        leftCmdWheelRate += 100; 
       }

      @Override
      public boolean isFinished() {
        return true;
      }
    };

    return riseUp100;
};

/**
* Constructs a command that lowers the commanded set point for 
* the left flywheel motor by 1000 rpm.
*
* @return The riseUp1000 command
*/
public Command downLC1000(){
  Command downL1000 = 
    new Command() {
      @Override
      public void initialize() {
        /* Start the Launcher Wheels and the Launch timer. */
        leftCmdWheelRate -= 1000; 
       }

      @Override
      public boolean isFinished() {
        return true;
      }
    };

    return downL1000;
};

/**
* Constructs a command that lowers the commanded set point for 
* the left flywheel motor by 100 rpm.
*
* @return The down1000 command
*/
public Command downLC100(){
  Command down100 = 
    new Command() {
      @Override
      public void initialize() {
        /* Start the Launcher Wheels and the Launch timer. */
        leftCmdWheelRate -= 100; 
       }

      @Override
      public boolean isFinished() {
        return true;
      }
    };

    return down100;
};

/**
* Constructs a command that raises the commanded set point for 
* the Right flywheel motor by 1000 rpm.
*
* @return The riseUp1000 command
*/
public Command raiseRC1000(){
  Command riseUp1000 = 
    new Command() {
      @Override
      public void initialize() {
        /* Start the Launcher Wheels and the Launch timer. */
        rightCmdWheelRate += 1000; 
       }

      @Override
      public boolean isFinished() {
        return true;
      }
    };

    return riseUp1000;
};

/**
* Constructs a command that raises the commanded set point for 
* the Right flywheel motor by 100 rpm.
*
* @return The riseUp100 command
*/
public Command raiseRC100(){
  Command riseUp100 = 
    new Command() {
      @Override
      public void initialize() {
        /* Start the Launcher Wheels and the Launch timer. */
        rightCmdWheelRate += 100; 
       }

      @Override
      public boolean isFinished() {
        return true;
      }
    };

    return riseUp100;
};

/**
* Constructs a command that lowers the commanded set point for 
* the right flywheel motor by 1000 rpm.
*
* @return The down1000 command
*/
public Command downRC1000(){
  Command down1000 = 
    new Command() {
      @Override
      public void initialize() {
        /* Start the Launcher Wheels and the Launch timer. */
        rightCmdWheelRate -= 1000; 
       }

      @Override
      public boolean isFinished() {
        return true;
      }
    };

    return down1000;
};

/**
* Constructs a command that lowers the commanded set point for 
* the right flywheel motor by 100 rpm.
*
* @return The down1000 command
*/
public Command downRC100(){
  Command down100 = 
    new Command() {
      @Override
      public void initialize() {
        /* Start the Launcher Wheels and the Launch timer. */
        rightCmdWheelRate -= 100; 
       }

      @Override
      public boolean isFinished() {
        return true;
      }
    };

    return down100;
};

public Command testFlyWheels() {
    Command startTest =
        new Command() {
          
          private Timer m_timer;

          @Override
          public void initialize() {
            /* Start the Launcher Wheels and the Launch timer. */
            flyWheelsRunning = true;
            m_timer = new Timer();
            m_timer.start();

          }

          @Override
          public void execute() {
            /* Wait until the Launcher Wheels get up to speed,
             * then start the Intake Feeder to push the Note up
             * into the Launcher Wheels.
             */
            flyWheelsRunning = true;
          }

          @Override
          public boolean isFinished() {
            /* The Launcher and Intake feeder will stop after an
             * appropriate delay.
             */
            return m_timer.get() > Constants.Launcher.kFlyWheelStopTime;
          }

          @Override
          public void end(boolean interrupted) {
            /* Stop both the Launcher and the Intake feeder */
            flyWheelsRunning = false;
          }
        };

        return startTest;
  }

  /**
   * Turns the launcher off. Can be run once and the launcher will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void stopLauncher() {
    flyWheelsRunning = false;
  }

  @Override
  public void periodic() { // this method will be called once per scheduler run
    // set the launcher motor powers based on whether the launcher is on or not

    double leftSetPoint = leftCmdWheelRate;
    double rightSetPoint;

    double max = 1.0; //SmartDashboard.getNumber("Max Output", 0);
    double min = -1.0; //SmartDashboard.getNumber("Min Output", 0);
    // read PID coefficients from SmartDashboard
    //    double lp = SmartDashboard.getNumber("P Gain", 0);
    //    double li = SmartDashboard.getNumber("I Gain", 0);
    //    double ld = SmartDashboard.getNumber("D Gain", 0);
    //    double liz = SmartDashboard.getNumber("I Zone", 0);
    //    double lff = SmartDashboard.getNumber("Feed Forward", 0);
    //    double max = SmartDashboard.getNumber("Max Output", 0);
    //    double min = SmartDashboard.getNumber("Min Output", 0);

    double lp = 0.00007; //SmartDashboard.getNumber("P Gain", 0);
    double li = 0.0000007; //SmartDashboard.getNumber("I Gain", 0);
    double ld = 0; //SmartDashboard.getNumber("D Gain", 0);
    double liz = 0; //SmartDashboard.getNumber("I Zone", 0);
    double lff = 0.000007; //SmartDashboard.getNumber("Feed Forward", 0);
//    SmartDashboard.putNumber("L P Gain", lp);
//    SmartDashboard.putNumber("L I Gain", li);
//    SmartDashboard.putNumber("L D Gain", ld);
//    SmartDashboard.putNumber("L I Zone", liz);
//    SmartDashboard.putNumber("L Feed Forward", lff);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((lp != kLP)) { m_leftLancherPIDCtrl.setP(lp); kLP = lp; }
    if((li != kLI)) { m_leftLancherPIDCtrl.setI(li); kLI = li; }
    if((ld != kLD)) { m_leftLancherPIDCtrl.setD(ld); kLD = ld; }
    if((liz != kLIz)) { m_leftLancherPIDCtrl.setIZone(liz); kLIz = liz; }
    if((lff != kLFF)) { m_leftLancherPIDCtrl.setFF(lff); kLFF = lff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_leftLancherPIDCtrl.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    // read PID coefficients from SmartDashboard
    double rp = 0.00007; //double rp = SmartDashboard.getNumber("P Gain", 0);
    double ri = 0.0000007; //double ri = SmartDashboard.getNumber("I Gain", 0);
    double rd = 0; //double rd = SmartDashboard.getNumber("D Gain", 0);
    double riz = 0; //double riz = SmartDashboard.getNumber("I Zone", 0);
    double rff = 0.000007;
    //double rff = SmartDashboard.getNumber("R Feed Forward", .2);
//    leftSetPoint = SmartDashboard.getNumber("Left Command Velocity", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((rp != kRP)) { m_rightLancherPIDCtrl.setP(rp); kRP = rp; }
    if((ri != kRI)) { m_rightLancherPIDCtrl.setI(ri); kRI = ri; }
    if((rd != kRD)) { m_rightLancherPIDCtrl.setD(rd); kRD = rd; }
    if((riz != kRIz)) { m_rightLancherPIDCtrl.setIZone(riz); kRIz = riz; }
    if((rff != kRFF)) { m_rightLancherPIDCtrl.setFF(rff); kRFF = rff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_rightLancherPIDCtrl.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    
    if (flyWheelsRunning) {

      leftSetPoint = leftCmdWheelRate; // leftCmdWheelRate/maxMotorRPM;
      rightSetPoint = rightCmdWheelRate; //Constants.NeoMotorConstants.kFreeSpeedRpm; // rightCmdWheelRate/maxMotorRPM;

    } else {
      leftSetPoint = 0;
      rightSetPoint = 0;
    }

    m_leftLancherPIDCtrl.setReference(leftSetPoint, CANSparkMax.ControlType.kVelocity);
    m_rightLancherPIDCtrl.setReference(rightSetPoint, CANSparkMax.ControlType.kVelocity);

    ShuffleboardTab LaunchTab = Shuffleboard.getTab("Launcher Subsystem");
    
//    Shuffleboard.getTab("Launcher Subsystem").add("Left RPM Setting", leftCmdWheelRate);
    SmartDashboard.putNumber("Left RPM", leftCmdWheelRate);
    SmartDashboard.putNumber("Right RPM", rightCmdWheelRate);
//    Shuffleboard.getTab("Launcher Subsystem").add("Left Current RPM", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Left Current RPM", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Current RPM", m_rightEncoder.getVelocity());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // Publish encoder distances to telemetry.
    builder.addBooleanProperty("Running", this::getRunning, this::setRunning);

    builder.addDoubleProperty("L_P_Gain", this::getLPGain, this::setLPGain);
     
    builder.addDoubleProperty("leftWheelRate", this::getLeftWheelRate, this::setLeftWheelRate);
    builder.addDoubleProperty("RightWheelRate", LauncherSubsystem::getRightWheelRate, null);
  }

  public boolean getRunning(){
    return flyWheelsRunning;
  }

  public void setRunning(Boolean _running){
    flyWheelsRunning = _running;
  }

  public double getLPGain(){
    return kLP;
  }

  public void setLPGain(double _kLP){
    kLP = _kLP;
  }

  public double getLeftWheelRate(){
    return leftCmdWheelRate;
  }

  public void setLeftWheelRate(double _leftCmdWheelRate){
    leftCmdWheelRate = _leftCmdWheelRate;
  }

  public static double getRightWheelRate(){
    return rightCmdWheelRate;
  }
}
