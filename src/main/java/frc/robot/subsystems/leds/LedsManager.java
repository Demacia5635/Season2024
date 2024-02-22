// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import java.util.Arrays;
import java.util.stream.IntStream;
import java.util.List;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.utils.IndividualLed;
import frc.robot.subsystems.leds.utils.LedsGeometry;

public final class LedsManager extends SubsystemBase {
    // private final Colors[] off;
    private static LedsManager instance;

    private final LedsGeometry ledsGeometry;
    private Color[] leds;

    private LedsManager() {
        ledsGeometry = new LedsGeometry(LedConstants.LED_STRIPS);
        // off = IntStream.range(0, ledsGeometry.totalLength).mapToObj((i)-> new IndividualLed(i, Color.kBlack)).toList();
        leds = new Color[ledsGeometry.totalLength];
        setDefaultColor();
    }

    private void setDefaultColor() {
        Arrays.fill(leds, Color.kBlack);
    }

    public static LedsManager getInstance() {
        if (instance == null) {
            instance = new LedsManager();
        }
        return instance;
    }

    public void update(IndividualLed... individualLeds) {
        Arrays.stream(individualLeds).forEach((led) -> leds[led.index] = led.color);
    }

    private void setChanges() {
        ledsGeometry.setColor(leds);
    }

    public Color[] getColors(int startIndex, int size) {
        return IntStream.range(startIndex, size + startIndex).mapToObj((i) -> leds[i]).toArray(Color[]::new);
    }

    @Override
    public void periodic() {
        setChanges();
    }
}
