// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeceiverRobotState;
import frc.robot.DeceiverRobotState.FeederStatus;
import frc.robot.HubShiftUtil;
import frc.robot.HubShiftUtil.ShiftInfo;

@SuppressWarnings("unused")
public class Leds extends SubsystemBase {
  // LED data
  // private final AddressableLED leds;
  // private final AddressableLEDBuffer buffer;

  private final CANdle candle = new CANdle(0);
  private static final int length = 35;

  private static final StrobeAnimation noConnect =
      new StrobeAnimation(8, 8 + length)
          .withColor(RGBWColor.fromHSV(0, 1.0, 1.0))
          .withSlot(0)
          .withFrameRate(5);
  private static final RainbowAnimation rainbow = new RainbowAnimation(8, length + 8).withSlot(0);
  private static final LarsonAnimation bounce =
      new LarsonAnimation(8, length + 8)
          .withBounceMode(LarsonBounceValue.Front)
          .withColor(new RGBWColor(Color.kBlue))
          .withSlot(0)
          .withSize(5);
  private static final StrobeAnimation strobePurple =
      new StrobeAnimation(8, length + 8)
          .withColor(new RGBWColor(Color.kPurple))
          .withSlot(0)
          .withFrameRate(10);
  private static final StrobeAnimation strobeGreen =
      new StrobeAnimation(8, length + 8)
          .withColor(new RGBWColor(Color.kGreen))
          .withSlot(0)
          .withFrameRate(10);
  private static final StrobeAnimation strobeYellow =
      new StrobeAnimation(8, length + 8)
          .withColor(new RGBWColor(Color.kYellow))
          .withSlot(0)
          .withFrameRate(10);
  private static final StrobeAnimation strobeRed =
      new StrobeAnimation(8, length + 8)
          .withColor(new RGBWColor(Color.kRed))
          .withSlot(0)
          .withFrameRate(10);
  private static final SolidColor purpleColor =
      new SolidColor(8, length + 8).withColor(new RGBWColor(Color.kPurple));
  private static final SolidColor greenColor =
      new SolidColor(8, length + 8).withColor(new RGBWColor(Color.kGreen));
  private static final SolidColor yellowColor =
      new SolidColor(8, length + 8).withColor(new RGBWColor(Color.kYellow));
  private static final SolidColor redColor =
      new SolidColor(8, length + 8).withColor(new RGBWColor(Color.kRed));

  // Pattern Constants
  private static final double strobeFastDuration = 0.1;
  private static final double strobeSlowDuration = 0.25;
  private static final double breathDuration = 1.0;
  private static final double rainbowCycleLength = 25.0;
  private static final double rainbowDuration = 1.0;
  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;
  private static final double waveSlowCycleLength = 25.0;
  private static final double waveSlowDuration = 3.0;
  private static final double waveAllianceCycleLength = 15.0;
  private static final double waveAllianceDuration = 2.0;
  private static final double autoFadeTime = 2.5;
  private static final double autoFadeMaxTime = 5.0;
  private static final int stripeLength = 3;
  private static final double stripeDuration = 1.0;

  private static final RGBWColor kGreen = new RGBWColor(0, 217, 0, 0);
  private static final RGBWColor kWhite = new RGBWColor(Color.kWhite).scaleBrightness(0.5);
  private static final RGBWColor kViolet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);
  private static final RGBWColor kRed = RGBWColor.fromHex("#D9000000").orElseThrow();

  // Startup notifier
  // private final Notifier loadingNotifier;

  private final DeceiverRobotState state;

  /** Creates a new Leds. */
  public Leds() {
    // Configure LED strip
    // leds = new AddressableLED(6);
    // leds.setColorOrder(ColorOrder.kRGB);
    // buffer = new AddressableLEDBuffer(length);
    // leds.setLength(length);
    // leds.setData(buffer);
    // leds.start();

    state = DeceiverRobotState.getInstance();

    // // Start pattern while robot is booting
    // loadingNotifier =
    //     new Notifier(
    //         () -> {
    //           synchronized (this) {
    //             breath(
    //                 Section.FULL,
    //                 Color.kBlue,
    //                 Color.kWhite,
    //                 strobeSlowDuration,
    //                 System.currentTimeMillis() / 1000.0);
    //             leds.setData(buffer);
    //           }
    //         });

    // loadingNotifier.startPeriodic(0.02);

    CANdleConfiguration config = new CANdleConfiguration();
    config.CANdleFeatures.withEnable5VRail(Enable5VRailValue.Enabled)
        .withVBatOutputMode(VBatOutputModeValue.On)
        .withStatusLedWhenActive(StatusLedWhenActiveValue.Disabled);
    config
        .LED
        .withBrightnessScalar(0.5)
        .withStripType(StripTypeValue.RGB)
        .withLossOfSignalBehavior(LossOfSignalBehaviorValue.KeepRunning);

    candle.getConfigurator().apply(config);

    for (int i = 0; i < 8; ++i) {
      candle.setControl(new EmptyAnimation(i));
    }

    candle.setControl(new SolidColor(0, 3).withColor(kGreen));
    candle.setControl(new SolidColor(4, 7).withColor(kWhite));

    candle.setControl(
        new ColorFlowAnimation(8, length + 8)
            .withColor(new RGBWColor(Color.kBlue))
            .withDirection(AnimationDirectionValue.Forward)
            .withFrameRate(10)
            .withSlot(0));
  }

  @Override
  public void periodic() {

    // Stop loading pattern after it has booted
    // loadingNotifier.stop();

    if (!DriverStation.isDSAttached()) {
      // No driverstation attached
      // strobe(Section.FULL, Color.kRed, strobeSlowDuration);
      candle.setControl(noConnect);
    } else if (DriverStation.isDisabled()) {
      // Disabled
      // stripes(Section.FULL, List.of(Color.kWhite, Color.kBlue), stripeLength, stripeDuration);
      candle.setControl(bounce);
    } else if (DriverStation.isAutonomous()) {
      // In Autonomous
      // rainbow(Section.FULL, rainbowCycleLength, rainbowDuration);
      candle.setControl(rainbow);
    } else {
      ShiftInfo shiftInfo = HubShiftUtil.getOfficialShiftInfo();

      if (shiftInfo.remainingTime() < 5.0) {
        if (shiftInfo.active()) {
          // active end
          if (state.getFeederStatus() == FeederStatus.Feeding) {
            // strobe(Section.FULL, Color.kPurple, strobeSlowDuration);
            candle.setControl(strobePurple);
          } else {
            // strobe(Section.FULL, Color.kGreen, strobeSlowDuration);
            candle.setControl(strobeGreen);
          }
        } else {
          // inactive end
          if (state.getFeederStatus() == FeederStatus.Feeding) {
            // strobe(Section.FULL, Color.kPurple, strobeSlowDuration);
            candle.setControl(strobeYellow);
          } else {
            // strobe(Section.FULL, Color.kRed, strobeSlowDuration);
            candle.setControl(strobeRed);
          }
        }
      } else {
        if (shiftInfo.active()) {
          // active during
          if (state.getFeederStatus() == FeederStatus.Feeding) {
            //   stripes(
            //       Section.FULL, List.of(Color.kPurple, Color.kGreen), stripeLength,
            // stripeDuration);
            candle.setControl(new EmptyAnimation(0));
            candle.setControl(purpleColor);
          } else {
            // solid(Section.FULL, Color.kGreen);
            candle.setControl(new EmptyAnimation(0));
            candle.setControl(greenColor);
          }
        } else {
          // inactive during
          if (state.getFeederStatus() == FeederStatus.Feeding) {
            // stripes(Section.FULL, List.of(Color.kPurple, Color.kRed), stripeLength,
            // stripeDuration);
            candle.setControl(new EmptyAnimation(0));
            candle.setControl(yellowColor);
          } else {
            // solid(Section.FULL, Color.kRed);
            candle.setControl(new EmptyAnimation(0));
            candle.setControl(redColor);
          }
        }
      }
    }

    // leds.setData(buffer);
  }

  // // Display a solid color
  // private void solid(Section section, Color color) {
  //   if (color != null) {
  //     for (int i = section.start(); i < section.end(); i++) {
  //       buffer.setLED(i, color);
  //     }
  //   }
  // }

  // // Strobe color on and off
  // private void strobe(Section section, Color color, double duration) {
  //   boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
  //   solid(section, on ? color : Color.kBlack);
  // }

  // // Breath between two colors (fad in and out)
  // private void breath(Section section, Color c1, Color c2, double duration) {
  //   breath(section, c1, c2, duration, Timer.getFPGATimestamp());
  // }

  // // Breath between two colors (fad in and out)
  // private void breath(Section section, Color c1, Color c2, double duration, double timestamp) {
  //   double x = ((timestamp % breathDuration) / breathDuration) * 2.0 * Math.PI;
  //   double ratio = (Math.sin(x) + 1.0) / 2.0;
  //   double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
  //   double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
  //   double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
  //   solid(section, new Color(red, green, blue));
  // }

  // // Display a moving rainbow
  // private void rainbow(Section section, double cycleLength, double duration) {
  //   double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
  //   double xDiffPerLed = 180.0 / cycleLength;
  //   for (int i = 0; i < section.end(); i++) {
  //     x += xDiffPerLed;
  //     x %= 180.0;
  //     if (i >= section.start()) {
  //       buffer.setHSV(i, (int) x, 255, 255);
  //     }
  //   }
  // }

  // // Display a moving wave between two colors (gradient between two colors that moves)
  // private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
  //   double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
  //   double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
  //   for (int i = 0; i < section.end(); i++) {
  //     x += xDiffPerLed;
  //     if (i >= section.start()) {
  //       double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
  //       if (Double.isNaN(ratio)) {
  //         ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
  //       }
  //       if (Double.isNaN(ratio)) {
  //         ratio = 0.5;
  //       }
  //       double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
  //       double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
  //       double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
  //       buffer.setLED(i, new Color(red, green, blue));
  //     }
  //   }
  // }

  // // Display stipes of multiple colors
  // private void stripes(Section section, List<Color> colors, int length, double duration) {
  //   int offset = (int) (Timer.getFPGATimestamp() % duration / duration * length * colors.size());
  //   for (int i = section.start(); i < section.end(); i++) {
  //     int colorIndex =
  //         (int) (Math.floor((double) (i - offset) / length) + colors.size()) % colors.size();
  //     colorIndex = colors.size() - 1 - colorIndex;
  //     buffer.setLED(i, colors.get(colorIndex));
  //   }
  // }

  // private static enum Section {
  //   FULL;

  //   private int start() {
  //     switch (this) {
  //       case FULL:
  //         return 0;
  //       default:
  //         return 0;
  //     }
  //   }

  //   private int end() {
  //     switch (this) {
  //       case FULL:
  //         return length;
  //       default:
  //         return length;
  //     }
  //   }
  // }
}
