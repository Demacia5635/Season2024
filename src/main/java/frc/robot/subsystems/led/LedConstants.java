// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;
import frc.robot.subsystems.led.utils.IntPair;
/** Add your docs here. */
public class LedConstants {
    public static final IntPair[] LED_STRIPS = {
        new IntPair(0, 60)
    };
    public static final double MAX_ANGLE = 50;
    public static final double EPSILON = 3;
    /** time to wait between switch from blank to color in blink. 
     * in miliseconds 
     */
    public static final double BLINK_WAIT_TIME = 300;
}
