package frc.robot.sensors.util;

import edu.wpi.first.wpilibj.I2C;

import java.nio.ByteOrder;

import static java.util.Objects.requireNonNull;

/**
 * This is a collection of pure static functions that help make dealing with I2C a little bit easier
 */
public final class I2CUtils {

    /**
     * Returns a 2 byte value (word) that starts at the specified address.
     * @param connection - The I2C connection of the desired device
     * @param address - The starting address of the word
     * @param order - The endianness of the bytes that will be read.
     * @return the 2 byte value that was requested at the specified address
     */
    public static int readWord(I2C connection, int address, ByteOrder order) {
        requireNonNull(connection);
        requireNonNull(order);

        // todo check endianness/order of reading
        // TODO Refactor this
//        byte[] highByte = new byte[1];
//        byte[] lowByte = new byte[1];
//
//        if (order.equals(ByteOrder.BIG_ENDIAN)) {
//            connection.read(address, 1, highByte);
//            connection.read(address + 1, 1, lowByte);
//        } else if (order.equals(ByteOrder.LITTLE_ENDIAN)){
//            connection.read(address, 1, lowByte);
//            connection.read(address + 1, 1, highByte);
//        }
        byte[] bts = new byte[2];
        connection.read(address, 2, bts);


        int high = bts[0] & 0xFF; // (<byte> & 0xFF) converts a signed byte into an unsigned byte
        int low = bts[1] & 0xFF;

        return (high << 8) + low; // move high bits to high bits position, and add in low bits
                                  // this essentially creates a 2 byte value (referred to as a word)

    }

    public static int readWord(I2C connection, int address) {
        return readWord(connection, address, ByteOrder.BIG_ENDIAN); // High -> Low
    }

    public static short asTwosComplement(int word) {
        return (short) word;
    }

}
