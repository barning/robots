package de.hfkbremen.robots.challenge.FUD;

public class FixtureUserData {

    public static enum FixtureUserDataType {

        FUD_ROBOT_TIRE,
        FUD_ROBOT_BODY,
        FUD_ROBOT_SENSOR,
        FUD_GROUND_AREA,
        FUD_OBSTACLE_WALL,
        FUD_OBSTACLE_BALL
    }

    protected final FixtureUserDataType m_type;

    public FixtureUserData(FixtureUserDataType type) {
        m_type = type;
    }

    public FixtureUserDataType getType() {
        return m_type;
    }
}
