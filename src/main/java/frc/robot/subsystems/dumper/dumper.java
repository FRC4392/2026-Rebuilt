

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeceiverRobotState;

import org.littletonrobotics.junction.Logger;public class dumper extends SubsystemBase{
    private final DumperIO dumperIO;
    //private final DumperIO IOInputsAutoLogged inputs = new DumpMeSilly(); //not very important
    private final DeceiverRobotState robotState;

      /** I... I think This makes a dumpa */
  public Climber(DumperIO IO) {
    DumperIO = IO;
    robotState = DeceiverRobotState.getInstance();
  }
    private final Alert DumperDisconnected = new Alert("Dumpa no dump", AlertType.kError);

  
    @Override
    public void periodic() {
        dumperIO.updateInputs(inputs);
        Logger.processInputs("Dumper", inputs);

        climberMotorDisconnectedAlert.set(!inputs.motorConnected);
    }

    public void setVoltage(Voltage volts) {
        dumperIO.setVoltage(volts);
    }

    public Command runTestVoltage() {
        return this.runEnd(() -> setVoltage(Volts.of(6)), () -> setVoltage(Volts.of(0)));
    }
}