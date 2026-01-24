package frc.robot.util.simulation;

import java.util.ArrayList;
import java.util.List;

public class SimulationManager {
  public static List<VisualSimulator> simulations = new ArrayList<>();

  public static void addSimulationMechanism(VisualSimulator simulator) {
    simulations.add(simulator);
  }

  public static void updateSim() {
    for (VisualSimulator visualSimulator : simulations) {
      visualSimulator.periodic();
    }
  }
}
