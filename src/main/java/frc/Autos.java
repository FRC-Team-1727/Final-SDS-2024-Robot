package frc;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Autos {
    public static Command preload() {
        return Commands.sequence(
            NamedCommands.getCommand("start_shooter"));
      }
}
