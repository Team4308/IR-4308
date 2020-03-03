package bbb.path;

public class PathFollowerSettings {
    public final double kEncoderCountsPerRotation;
    public final double kGearRatio;

    public final Gains leftGains;
    public final Gains rightGains;
    public final Gains turnGains;

    public final int profileSlot;
    public final double period;

    public PathFollowerSettings(double _kEncoderCountsPerRotation, double _kGearRatio, Gains _leftGains, Gains _rightGains, Gains _turnGains, int _profileSlot, double _period) {
        kEncoderCountsPerRotation = _kEncoderCountsPerRotation;
        kGearRatio = _kGearRatio;

        leftGains = _leftGains;
        rightGains = _rightGains;
        turnGains = _turnGains;

        profileSlot = _profileSlot;
        period = _period;
    }
}