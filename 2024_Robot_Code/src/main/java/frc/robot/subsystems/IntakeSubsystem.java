package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private VictorSPX topRoller;
    private VictorSPX feedRollers;
    private double top_power;
    private double feed_power;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {

        topRoller = new VictorSPX(6);
        topRoller.setInverted(true);
        
        feedRollers = new VictorSPX(7);
        feedRollers.setInverted(true);

        top_power = 0.0;
        feed_power = 0.0;
        }
  /**
   * Set the power to spin the motor at. This only applies outside of position mode.
   *
   * @param _power The power to apply to the motor (from -1.0 to 1.0).
   */
  public void setPower(double _tpower, double _fpower) {
    top_power = _tpower;
    feed_power = _fpower;
  }

  public void moveNote(double volumeLevel) {
    feed_power = volumeLevel;
  }

  public void stopFeeder() {
    feed_power = 0.0;
  }

/**
* Constructs a command that starts the launcher and then runs the Intake feeder
* motor to put the Note up to the spinning launcher wheels. After a few more seconds
* the command will shutdown the Launcher Wheels and an the Intake feeder. This
* command takes control of the intake subsystem to make sure the feeder keeps
* running during the launch sequence.
*
* @return The launch command
*/

public Command pickupNote() {
    Command pickingUp =
        new Command() {
          
          private Timer m_timer;

          @Override
          public void initialize() {
            /* Start the Feed Rollers. */
            m_timer = new Timer();
            m_timer.start();
            topRoller.set(ControlMode.PercentOutput, top_power);
            feedRollers.set(ControlMode.PercentOutput, feed_power);
          }

          @Override
          public void execute() {
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
            topRoller.set(ControlMode.PercentOutput, 0.0);
            feedRollers.set(ControlMode.PercentOutput, 0.0);
          }
        };

        return pickingUp;
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
    // if we've reached the position target, drop out of position mode
    // update the motor power based on mode and setpoint
    topRoller.set(ControlMode.PercentOutput, top_power);
    feedRollers.set(ControlMode.PercentOutput, feed_power);
    
  }


}
