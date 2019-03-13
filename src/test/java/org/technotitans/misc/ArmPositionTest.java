package org.technotitans.misc;

import frc.robot.movements.arm.ArmPosition;
import org.junit.Test;

import static junit.framework.TestCase.assertEquals;

public class ArmPositionTest {
    @Test
    public void armPositionShouldCalculateDistance() {
        ArmPosition position = ArmPosition.LOW_HATCH;
        assertEquals(40, position.getHorizontalArmLength(), 5);
    }
}
