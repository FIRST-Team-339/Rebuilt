// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems.leds;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import java.util.function.Supplier;

public class LEDs extends SubsystemBase {
  private static Map<Number, Color> maskSteps =
      Map.of(0, Color.kWhite, 0.1, Color.kBlack, 0.5, Color.kWhite, 0.6, Color.kBlack);
  private static LEDPattern mask =
      LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(35));
  private static Color kilroyOrange = new Color(1.0f, 0.07f, 0.0f);

  private AddressableLED led = new AddressableLED(0);
  private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(98);

  public static enum LEDMode {
    kOff(() -> LEDPattern.kOff),
    kAlliance(
        () ->
            LEDPattern.solid(
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? Color.kFirstBlue
                        : Color.kRed)
                .mask(mask)),
    kIntaking(() -> LEDPattern.solid(kilroyOrange).mask(mask)),
    kNotGoodToLaunch(() -> LEDPattern.solid(Color.kDarkMagenta)),
    kGoodToLaunch(() -> LEDPattern.solid(Color.kGreen));

    public final Supplier<LEDPattern> pattern;

    private LEDMode(Supplier<LEDPattern> pattern) {
      this.pattern = pattern;
    }
  }

  private LEDMode mode = LEDMode.kAlliance;

  /** Creates a new LEDs. */
  public LEDs() {
    led.setColorOrder(ColorOrder.kGRB);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  public void setMode(LEDMode newMode) {
    mode = newMode;
  }

  @Override
  public void periodic() {
    mode.pattern.get().applyTo(ledBuffer);
    // LEDPattern.solid(Color.kBlue).applyTo(ledBuffer);
    led.setData(ledBuffer);
  }
}
