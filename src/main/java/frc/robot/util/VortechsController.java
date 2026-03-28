package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class VortechsController extends CommandXboxController {


    private boolean isRumbling = false;

    public VortechsController(int port) {
        super(port);
    }
    
}
