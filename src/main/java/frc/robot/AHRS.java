package frc.robot;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import java.lang.AutoCloseable;

public class AHRS extends java.lang.Object {
public AHRS(Port kmxp) { 
    AHRS ahrs = new AHRS(kmxp);
}
public void reset(AHRS x){ 
reset(x);
}

public double getAngle(AHRS x){ 
    return getAngle(x);
}
}   
