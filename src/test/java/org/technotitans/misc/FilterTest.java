package org.technotitans.misc;


import frc.robot.motor.Filter;
import org.junit.Test;

import static junit.framework.TestCase.assertEquals;


public class FilterTest {

    public final double TEST_SENSITIVITY = 0.5;


    @Test
    public void testInitialSensitivity() {
        Filter filter = new Filter(TEST_SENSITIVITY);
        assertEquals(filter.getValue(), 0, 0); // delta is 0 because it should be the exact same as set
    }

    @Test
    public void testSingleUpdate() {
        // spec:
        // our implementation filter basically smooths out rapid updates of values
        // it does this by taking a percentage of the updated value (newValue)
        // and allowing the rest of the percentage to be determined by the old value
        // value = sensitivity * newValue + (1-sensitivity) * value;
        double testInput = .75;
        double targetOutput = 0.375;

        Filter filter = new Filter(TEST_SENSITIVITY); // 0.5
        filter.update(testInput);
        assertEquals(filter.getValue(), targetOutput, 1e-3); // TODO choose a thought out delta for this test
    }


}
