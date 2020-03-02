package bbb.path;

public class Gains {
	public final double kP;
	public final double kI;
	public final double kD;
    public final double kF;
    public final double kV;
    public final double ka;
	
	public Gains(double _kP, double _kI, double _kD, double _kF, double _kV, double _ka){
		kP = _kP;
		kI = _kI;
		kD = _kD;
        kF = _kF;
        kV = _kV;
        ka = _ka;
	}

	public Gains(double _kP, double _kI, double _kD, double _kF){
		kP = _kP;
		kI = _kI;
		kD = _kD;
        kF = _kF;
        kV = 0.0;
        ka = 0.0;
	}
}