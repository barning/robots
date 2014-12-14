package de.hfkbremen.robots.challenge;

import de.hfkbremen.robots.challenge.FUD.RobotSensorFUD;
import org.jbox2d.collision.RayCastInput;
import org.jbox2d.collision.RayCastOutput;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.MassData;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import processing.core.PApplet;

public class Sensor {
    
    private final Body mBody;
    private final World mWorld;
    private final Robot mHost;
    
    private boolean mTriggerState;
    private final Vec2 mObstaclePosition;
    private float mDistanceToObstacleNormalized;
    private float mAngle;
    private float mRadius;
    private final RevoluteJoint mJoint;
    
    private static final boolean TRIGGER_BY_COLLISION = false;
    
    Sensor(World pWorld, Robot pHost, final float pAngle, final float pRadius) {
        mWorld = pWorld;
        mHost = pHost;
        mAngle = pAngle;
        mRadius = pRadius;
        mObstaclePosition = new Vec2();

        /* - */
        BodyDef mBodyDef = new BodyDef();
        mBodyDef.type = BodyType.DYNAMIC;
        mBody = mWorld.createBody(mBodyDef);
        mBody.setUserData(new BodyUserData(this, BodyUserData.ShapeType.SENSOR));
        
        CircleShape mShape = new CircleShape();
        mShape.setRadius(0.5f);
        
        Fixture mFixture = mBody.createFixture(mShape, 1);
        mFixture.setSensor(true);
        mFixture.setUserData(new RobotSensorFUD());
        
        mTriggerState = false;

        /* - */
        RevoluteJointDef mJointDef = new RevoluteJointDef();
        mJointDef.enableLimit = true;
        mJointDef.lowerAngle = mAngle;
        mJointDef.upperAngle = mAngle;
        mJointDef.bodyA = mHost.body();
        mJointDef.localAnchorA.setZero();
        mJointDef.bodyB = mBody;

        mJoint = (RevoluteJoint) mWorld.createJoint(mJointDef);
        angle(pAngle);
    }
    
    Body body() {
        return mBody;
    }
    
    Robot host() {
        return mHost;
    }

    /**
     * DO NOT MODIFY. returns the position of the sensors
     *
     * @return
     */
    public Vec2 position() {
        return body().getPosition();
    }

    /**
     * DO NOT USE.
     *
     * @param pState
     */
    public void setTriggerByCollision(boolean pState) {
        if (TRIGGER_BY_COLLISION) {
            mTriggerState = pState;
        }
    }

    /**
     * return the current sensor state. 'true' if sensor detected obstacle.
     *
     * @return
     */
    public boolean triggered() {
        return mTriggerState;
    }

    /**
     * DO NOT USE.
     */
    public void update() {
        if (!TRIGGER_BY_COLLISION) {
            mObstaclePosition.set(castRay(mHost.body().getPosition(), body().getPosition()));
        }
    }

    /**
     * returns the postion of the detected obstacle. only up to date if
     * triggered() is true.
     *
     * @return
     */
    public Vec2 obstacle() {
        return mObstaclePosition;
    }

    /**
     * returns the distance to the current obstacle. only up to date if
     * triggered() is true.
     *
     * @return
     */
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

    /**
     * returns current angle of the sensor in relation to the robots forward
     * direction.
     *
     * @return angle in radians
     */
    public float angle() {
        return mAngle;
    }

    /**
     * sets the current angle of the sensor in relation to the robots forward
     * direction.
     *
     * @param pAngle angle in radians
     */
    public final void angle(float pAngle) {
        mAngle = pAngle;
        
        Vec2 mPosition = new Vec2(PApplet.sin(mAngle), PApplet.cos(mAngle));
        mPosition.mulLocal(mRadius);
        mJoint.getLocalAnchorA().set(mPosition);
    }
}
