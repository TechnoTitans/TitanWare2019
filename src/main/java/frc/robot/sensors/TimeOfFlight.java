/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import java.nio.ByteBuffer;
import java.util.Deque;

import edu.wpi.first.wpilibj.CircularBuffer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.util.CircularByteBuffer;

/**
 * TF Mini Sensor
 */
public class TimeOfFlight {
    private CircularByteBuffer buffer;

    private Deque<Byte> dequeBuffer;
    private SerialPort serial;

    private boolean aligned = false;

    public TimeOfFlight() {
        serial = new SerialPort(115200, SerialPort.Port.kMXP);
        buffer = new CircularByteBuffer(50);
        // serial.setTimeout(0.01);
    }

    public void update() {
        int bytesToRead = serial.getBytesReceived();
        if (buffer.available() > 18) {
            buffer.clear();
        }
        buffer.put(serial.read(bytesToRead));
        parse();
    }

    private void parse() {
        if (!aligned) {
            align();
            aligned = true;
            return;
        }
        byte[] data = new byte[9];
        buffer.get(data);
        if (verify(data)) {
            int distance = parse2Bytes(data[2], data[3]);
            int strength = parse2Bytes(data[4], data[5]);
            SmartDashboard.putNumber("TF Distance", distance);
        } else {
            aligned = false;
        }
        SmartDashboard.putBoolean("TF Distance sensor aligned", aligned);
        SmartDashboard.putRaw("TF Byte", data);
    }

    private int parse2Bytes(byte lo, byte hi) {
        int lo_i = (lo & 0xFF);
        int hi_i = (hi & 0xFF);
        return lo_i + hi_i * 256;
    }

    private boolean verify(byte[] data) {
        if (data[0] != 0x59 || data[1] != 0x59) return false;
        int total = 0;
        for (int i = 0; i < 8; ++i) {
            total += (data[i] & 0xFF);
        }
        if ((total & 0xFF) != (data[8] & 0xFF)) return false;
        return true;
    }

    private void align() {
        while (buffer.available() > 1 && (buffer.peek() != 0x59 || buffer.peekSecond() != 0x59)) {
            buffer.skip(1);
        }
    }




}
