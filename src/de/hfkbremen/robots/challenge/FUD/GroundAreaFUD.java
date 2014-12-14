package de.hfkbremen.robots.challenge.FUD;

import static de.hfkbremen.robots.challenge.FUD.FixtureUserData.FixtureUserDataType.FUD_GROUND_AREA;

public class GroundAreaFUD extends FixtureUserData {

    public float frictionModifier;
    public boolean outOfCourse;

    public GroundAreaFUD(float fm, boolean ooc) {
        super(FUD_GROUND_AREA);
        frictionModifier = fm;
        outOfCourse = ooc;
    }
}
