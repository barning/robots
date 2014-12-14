package de.hfkbremen.robots.challenge.environment;

import de.hfkbremen.robots.challenge.FUD.GroundAreaFUD;
import de.hfkbremen.robots.challenge.Wall;
import java.util.ArrayList;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;
import processing.core.PApplet;

import static de.hfkbremen.robots.challenge.Constants.DEGTORAD;

public class EnvironmentMapSimple implements EnvironmentMap {

    public final ArrayList<Wall> mWalls;

    private final Body mGround;

    public EnvironmentMapSimple(PApplet pParent, World pWorld) {
        /* walls */
        mWalls = new ArrayList<Wall>();
        final float mWallThickness = 2;
        final float mWallScale = 4;
        mWalls.add(new Wall(pWorld,
                            0, pParent.height / 2,
                            pParent.width, mWallThickness));
        mWalls.add(new Wall(pWorld,
                            0, pParent.height / -2,
                            pParent.width, mWallThickness));
        mWalls.add(new Wall(pWorld,
                            pParent.width / 2, 0,
                            mWallThickness, pParent.height));
        mWalls.add(new Wall(pWorld,
                            pParent.width / -2, 0,
                            mWallThickness, pParent.height));

        /* ground */
        BodyDef bodyDef = new BodyDef();
        mGround = pWorld.createBody(bodyDef);

        PolygonShape polygonShape = new PolygonShape();
        FixtureDef fixtureDef = new FixtureDef();
        fixtureDef.shape = polygonShape;
        fixtureDef.isSensor = true;

        polygonShape.setAsBox(9, 7, new Vec2(-10, 15), 20 * DEGTORAD);
        Fixture groundAreaFixture = mGround.createFixture(fixtureDef);
        groundAreaFixture.setUserData(new GroundAreaFUD(0.5f, false));

        polygonShape.setAsBox(9, 5, new Vec2(5, 20), -40 * DEGTORAD);
        groundAreaFixture = mGround.createFixture(fixtureDef);
        groundAreaFixture.setUserData(new GroundAreaFUD(0.2f, false));
    }

    public ArrayList<Wall> walls() {
        return mWalls;
    }

    public Body grounds() {
        return mGround;
    }
}
