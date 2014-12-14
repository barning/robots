package de.hfkbremen.robots.challenge.environment;

import de.hfkbremen.robots.challenge.Wall;
import java.util.ArrayList;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.World;

public class EnvironmentMapEmpty implements EnvironmentMap {

    public final ArrayList<Wall> mWalls = new ArrayList<Wall>();
    protected final Vec2 mBox;

    public EnvironmentMapEmpty(World pWorld, Vec2 pScale) {
        mBox = new Vec2(pScale);
        final float mWallThickness = 1;
        mWalls.add(new Wall(pWorld, 0, mBox.y / 2 + -mWallThickness / 2, mBox.x, mWallThickness));
        mWalls.add(new Wall(pWorld, 0, -mBox.y / 2 + mWallThickness / 2, mBox.x, mWallThickness));
        mWalls.add(new Wall(pWorld, mBox.x / 2 + -mWallThickness / 2, 0, mWallThickness, mBox.y));
        mWalls.add(new Wall(pWorld, -mBox.x / 2 + mWallThickness / 2, 0, mWallThickness, mBox.y));
    }

    public ArrayList<Wall> walls() {
        return mWalls;
    }

    public Body grounds() {
        return null;
    }
}
