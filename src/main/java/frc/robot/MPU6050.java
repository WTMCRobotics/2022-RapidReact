package frc.robot;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.wpilibj.I2C;

public class MPU6050 {
    I2C bus;
    long position = 0;

    public MPU6050(int address, I2C.Port port) {
        bus = new I2C(port, address);
        bus.write(107, 0); // disable sleep
        bus.write(27, 0x10); // set sensitivity level to 2
    }

    public MPU6050(int address) {
        this(address, I2C.Port.kOnboard);
    }

    public MPU6050() {
        this(0x68, I2C.Port.kOnboard);
    }

    public void tick() {
        ByteBuffer buf = ByteBuffer.allocate(2);
        buf.order(ByteOrder.BIG_ENDIAN);
        bus.read(71, 2, buf); // read Z gyro registers (big endian 16-bit)
        short data = buf.getShort(); // convert from bytes to 16-bit number
        position += data;
    }

    public long getPosition() {
        return position;
    }

    public void reset() {
        position = 0;
    }
}
