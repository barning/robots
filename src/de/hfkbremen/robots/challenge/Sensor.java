package de.hfkbremen.robots.challenge;

import de.hfkbremen.robots.challenge.FUD.RobotSensorFUD;
import org.jbox2d.collision.RayCastInput;
import org.jbox2d.collision.RayCastOutput;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.World;

public class Sensor {

    private final Body mBody;
    private final World mWorld;
    private final Robot mHost;

    private boolean mTriggerState;
    private final Vec2 mObstaclePosition;
    private float mDistanceToObstacleNormalized;

    private static final boolean TRIGGER_BY_COLLISION = false;

    Sensor(World pWorld, Robot pHost) {
        mWorld = pWorld;
        mHost = pHost;
        mObstaclePosition = new Vec2();

        BodyDef bodyDef = new BodyDef();
        bodyDef.type = BodyType.DYNAMIC;
        mBody = mWorld.createBody(bodyDef);
        mBody.setUserData(new BodyUserData(this, BodyUserData.ShapeType.SENSOR));

        CircleShape mShape = new CircleShape();
        mShape.setRadius(0.5f);

        Fixture fixture = mBody.createFixture(mShape, 1);
        fixture.setSensor(true);
        fixture.setUserData(new RobotSensorFUD());

        mTriggerState = false;
    }

    Body body() {
        return mBody;
    }

    Robot host() {
        return mHost;
    }

    public Vec2 position() {
        return body().getPosition();
    }

    public void setTriggerByCollision(boolean pState) {
        if (TRIGGER_BY_COLLISION) {
            mTriggerState = pState;
        }
    }

    public boolean triggered() {
        return mTriggerState;
    }

    public void update() {
        if (!TRIGGER_BY_COLLISION) {
            mObstaclePosition.set(castRay(mHost.body().getPosition(), body().getPosition()));
        }
    }

    public Vec2 obstacle() {
        return mObstaclePosition;
    }

    public float obstacleDistance() {
        /* distance in percent */
        return mDistanceToObstacleNormalized;
    }

    private Vec2 castRay(Vec2 p1, Vec2 p2) {
        //set up input
        final RayCastInput mInput = new RayCastInput();
        mInput.p1.set(p1);
        mInput.p2.set(p2);
        mInput.maxFraction = 1;

        /* check every fixture of every body to find closest */
        float mClosestFraction = Float.MAX_VALUE;

        Body mOtherBody = mWorld.getBodyList();
        while (mOtherBody != null) {
            Fixture mFixture = mOtherBody.getFixtureList();
            while (mFixture != null) {
                /* filter out own body */
                if (mHost.isBody(mOtherBody)) {
                    mFixture = mFixture.getNext();
                    continue;
                }

                /* filter out sensor fixtures */
                if (mFixture.isSensor()) {
                    mFixture = mFixture.getNext();
                    continue;
                }

                final RayCastOutput mOutput = new RayCastOutput();
                if (!mFixture.raycast(mOutput, mInput, 0)) {
                    mFixture = mFixture.getNext();
                    continue;
                }

                if (mOutput.fraction < mClosestFraction) {
                    mClosestFraction = mOutput.fraction;
                }
                mFixture = mFixture.getNext();
            }
            mOtherBody = mOtherBody.getNext();
        }

        final Vec2 intersectionPoint = p1.add(p2.sub(p1).mul(mClosestFraction));
        mTriggerState = mClosestFraction < Float.MAX_VALUE;
        mDistanceToObstacleNormalized = mClosestFraction;
        mObstaclePosition.set(intersectionPoint);

        return intersectionPoint;
    }
}
