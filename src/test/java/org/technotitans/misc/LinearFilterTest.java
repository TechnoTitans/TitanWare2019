package org.technotitans.misc;

import frc.robot.motor.LinearFilter;
import org.junit.Test;

import static junit.framework.TestCase.assertEquals;

public class LinearFilterTest {

    @Test
    public void filterRateShouldPreventSuddenChanges() {
        final double rate = 0.4, newValue = 0.9;
        LinearFilter filter = new LinearFilter(rate);
        assertEquals(filter.getValue(), 0, 0);
        filter.update(newValue);
        assertEquals(filter.getValue(), rate, 1e-5);
        filter.update(newValue);
        assertEquals(filter.getValue(), rate * 2, 1e-5);
        filter.update(newValue);
        assertEquals(filter.getValue(), newValue, 1e-5);
        filter.update(0);
        assertEquals(filter.getValue(), newValue - rate, 1e-5);
    }

    @Test
    public void filterRateShouldSetValue() {
        final double rate = 0.01, newValue = 1.3;
        LinearFilter filter = new LinearFilter(rate);
        assertEquals(filter.getValue(), 0, 0);
        filter.setValue(newValue);
        assertEquals(filter.getValue(), newValue, 0);


    }
}
