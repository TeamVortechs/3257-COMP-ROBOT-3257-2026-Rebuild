package frc.robot.util.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class VisualSimulator {

  private final LoggedMechanism2d panel;
  private final LoggedMechanismRoot2d root;
  private final LoggedMechanismLigament2d ligament;

  private Translation2d startPoint;

  private DoubleSupplier height;
  private DoubleSupplier angle;

  private String name;

  private VisualSimulator parent;

  private Supplier<Color8Bit> colorSupplier;

  public VisualSimulator(
      Translation2d startPoint,
      DoubleSupplier angle,
      DoubleSupplier height,
      double width,
      Color8Bit color,
      String name) {

    this.startPoint = new Translation2d(startPoint.getX(), startPoint.getY());

    this.panel = new LoggedMechanism2d(width, height.getAsDouble());

    this.root = panel.getRoot(name, startPoint.getX(), startPoint.getY());

    this.ligament =
        root.append(new LoggedMechanismLigament2d(name, Units.inchesToMeters(25), 90, 6, color));

    this.height = height;
    this.angle = angle;

    this.name = name;

    SimulationManager.addSimulationMechanism(this);
  }

  // updates the simulation, is handled with the simulatiom manager
  public void periodic() {
    // if the parent is set then set your position on the parent. Uses sin math because it needs to
    // with the angle changes
    if (parent != null) {
      double x = parent.getStartPoint().getX();
      double y = parent.getStartPoint().getY();

      x += Math.cos(parent.getAngle().getRadians()) * parent.getHeight();
      y += Math.sin(parent.getAngle().getRadians()) * parent.getHeight();

      setStartPoint(new Translation2d(x, y));
    }

    ligament.setAngle(angle.getAsDouble());
    ligament.setLength(height.getAsDouble());

    // if the color supplier is not null then set the color
    if (colorSupplier != null) {
      ligament.setColor(colorSupplier.get());
    }

    Logger.recordOutput(name, panel);
  }

  // changing the position of a root node
  public void setStartPoint(Translation2d startPoint) {
    this.startPoint = startPoint;

    root.setPosition(startPoint.getX(), startPoint.getY());
  }

  public void setX(double x) {
    setStartPoint(new Translation2d(x, startPoint.getY()));
  }

  public void setY(double y) {
    setStartPoint(new Translation2d(startPoint.getX(), y));
  }

  public Translation2d getStartPoint() {
    return startPoint;
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(angle.getAsDouble());
  }

  public double getHeight() {
    return height.getAsDouble();
  }

  // sets the color
  public void setColor(Color8Bit color) {
    // exit out if there is already a color supplier that is setting it
    if (colorSupplier != null) return;

    ligament.setColor(color);
  }

  // sets a supplier to automatically update the color. If this is not called the supplier will not
  // utilized.
  public void setColorSupplier(Supplier<Color8Bit> colorSupplier) {
    this.colorSupplier = colorSupplier;
  }

  // updates the child and makes position follow that of the parent. Setting the angle too must be
  // done in the constructor
  public void setParent(VisualSimulator parent) {
    this.parent = parent;
  }
}
