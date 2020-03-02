package bbb.path;

public class PathFollowerSettings {
    public final double kEncoderCountsPerRotation;

    public final Gains leftGains;
    public final Gains rightGains;
    public final Gains turnGains;

    public final double period;

    public PathFollowerSettings(double _kEncoderCountsPerRotation, Gains _leftGains, Gains _rightGains, Gains _turnGains, double _period) {
        kEncoderCountsPerRotation = _kEncoderCountsPerRotation;

        leftGains = _leftGains;
        rightGains = _rightGains;
        turnGains = _turnGains;

        period = _period;
    }
}