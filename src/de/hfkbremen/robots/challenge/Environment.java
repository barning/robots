package de.hfkbremen.robots.challenge;

import java.util.ArrayList;
import org.jbox2d.callbacks.ContactImpulse;
import org.jbox2d.callbacks.ContactListener;
import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.collision.shapes.ShapeType;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.contacts.Contact;
import processing.core.PGraphics;
import processing.core.PApplet;
import de.hfkbremen.robots.challenge.FUD.*;
import de.hfkbremen.robots.challenge.environment.EnvironmentMap;
import de.hfkbremen.robots.challenge.environment.EnvironmentMapBaelleBad;
import de.hfkbremen.robots.challenge.environment.EnvironmentMapSimple;
import de.hfkbremen.robots.challenge.environment.EnvironmentMapEmpty;
import de.hfkbremen.robots.challenge.environment.EnvironmentMapRound_1;
import org.jbox2d.dynamics.BodyDef;

import static de.hfkbremen.robots.challenge.FUD.FixtureUserData.FixtureUserDataType.*;

public class Environment implements ContactListener {

    public static final int MAP_EMPTY = 0;
    public static final int MAP_SIMPLE = 1;
    public static final int MAP_ROUND_1 = 2;
    public static final int MAP_BAELLEBAD = 3;

    private final EnvironmentMap mMap;

    private final ArrayList<Robot> mRobots;

    private final World mWorld;

    public Environment(PApplet pParent, int pEnvMap) {
        mWorld = new World(new Vec2(0, 0));
        mWorld.setWarmStarting(true);
        mWorld.setContinuousPhysics(true);

        BodyDef bodyDef = new BodyDef();
        Body groundBody = mWorld.createBody(bodyDef);

        mWorld.setContactListener(this);

        mMap = createEnvMap(pParent, mWorld, pEnvMap);
        mRobots = new ArrayList<Robot>();
    }

    public void add(Robot pRobot) {
        mRobots.add(pRobot);
    }

    public World world() {
        return mWorld;
    }

    private static final EnvironmentMap createEnvMap(PApplet pParent, World pWorld, int pEnvMap) {
        final EnvironmentMap mNewMap;
        switch (pEnvMap) {
            case MAP_EMPTY: {
                mNewMap = new EnvironmentMapEmpty(pWorld, new Vec2(1024 / 5.0f, 768 / 5.0f));
            }
            break;
            case MAP_SIMPLE: {
                mNewMap = new EnvironmentMapSimple(pParent, pWorld);
            }
            break;
            case MAP_BAELLEBAD: {
                mNewMap = new EnvironmentMapBaelleBad(pWorld, 30);
            }
            break;
            case MAP_ROUND_1: {
                mNewMap = new EnvironmentMapRound_1(pWorld);
            }
            break;
            default:
                mNewMap = null;
        }
        return mNewMap;
    }

    public void update() {
        float timeStep = 1.0f / 60f;
        mWorld.step(timeStep, 10, 8);
        mWorld.clearForces();

        /* handle robots */
        for (Robot mRobot : mRobots) {
            mRobot._update();
        }
    }

    public void draw(PGraphics g) {
        beginDraw(g);
        endDraw(g);
    }

    public void draw(PGraphics g, final float pScale, Vec2 pCenter) {
        beginDraw(g, pScale, pCenter);
        endDraw(g);
    }

    public void beginDraw(PGraphics g) {
        beginDraw(g, 1, new Vec2());
    }

    public void beginDraw(PGraphics g, final float pScale, Vec2 pCenter) {
        /* draw all shapes */
        g.strokeWeight(1.0f / pScale);
        g.stroke(0);
        g.noFill();
        g.pushMatrix();
        g.translate(g.width / 2, g.height / 2);
        g.scale(1, -1);
        g.scale(pScale);
        g.translate(-pCenter.x, -pCenter.y);

        drawWorld(g, mWorld);

        for (Robot mRobot : mRobots) {
            final float mAngle = mRobot.body().getAngle();
            final Vec2 mPosition = mRobot.body().getPosition();
            g.pushMatrix();
            g.translate(mPosition.x, mPosition.y);
            g.rotate(mAngle);
            mRobot.draw(g);
            g.popMatrix();

            for (Sensor mSensor : mRobot.sensors()) {
                g.stroke(0, 127, 255, 127);
                g.noFill();
                g.line(mSensor.position().x, mSensor.position().y,
                       mSensor.host().position().x, mSensor.host().position().y);
                if (mSensor.triggered()) {
                    g.stroke(0, 127, 255);
                    g.noFill();
                    Vec2 mHit = mSensor.position().sub(mSensor.host().position());
                    mHit.mulLocal(mSensor.obstacleDistance());
                    mHit.addLocal(mSensor.host().position());
                    g.pushMatrix();
                    g.translate(mHit.x, mHit.y);
                    g.ellipse(0, 0, 2 * mSensor.obstacleDistance(), 2 * mSensor.obstacleDistance());
                    g.popMatrix();
                }
            }
        }
    }

    public void endDraw(PGraphics g) {
        g.popMatrix();
    }

    /* --------------------------------- */
    public static void drawWorld(PGraphics g, World pWorld) {
        Body mBody = pWorld.getBodyList();
        while (mBody != null) {

            setColorFromUserData(g, mBody);

            Fixture mFixture = mBody.getFixtureList();
            while (mFixture != null) {
                final Shape mShape = mFixture.getShape();
                if (mShape.m_type == ShapeType.POLYGON) {
                    final PolygonShape mPolygonShape = (PolygonShape) mShape;
                    final float mAngle = mBody.getAngle();
                    final Vec2 mPosition = mBody.getPosition();
                    final Vec2[] mVertices = mPolygonShape.getVertices();
                    g.pushMatrix();
                    g.translate(mPosition.x, mPosition.y);
                    g.rotate(mAngle);
                    g.beginShape(PGraphics.POLYGON);
                    for (int i = 0; i < mPolygonShape.getVertexCount(); i++) {
                        Vec2 mWorldVertex = mVertices[i];
                        g.vertex(mWorldVertex.x, mWorldVertex.y);
                    }
                    g.endShape(PGraphics.CLOSE);
                    g.popMatrix();
                } else if (mShape.m_type == ShapeType.CIRCLE) {
                    final CircleShape mCircleShape = (CircleShape) mShape;
                    final float mAngle = mBody.getAngle();
                    final float mRadius = mCircleShape.getRadius();
                    final Vec2 mPosition = mBody.getPosition();
                    g.pushMatrix();
                    g.translate(mPosition.x, mPosition.y);
                    g.rotate(mAngle);
                    g.ellipse(0, 0, mRadius * 2, mRadius * 2);
                    g.popMatrix();
                } else {
                    System.out.println("unimplemented shape: " + mShape.m_type);
                }
                mFixture = mFixture.getNext();
            }
            mBody = mBody.getNext();
        }
    }

    private static void setColorFromUserData(PGraphics g, Body mBody) {
        Object mBodyUserData = mBody.getUserData();
        if (mBodyUserData != null && mBodyUserData instanceof BodyUserData) {
            final BodyUserData mType = (BodyUserData) mBodyUserData;
            if (mType.shape_type == BodyUserData.ShapeType.ROBOT) {
                g.noFill();
                g.stroke(0, 127, 255);
            } else if (mType.shape_type == BodyUserData.ShapeType.TIRE) {
                g.noFill();
                g.stroke(127);
            } else if (mType.shape_type == BodyUserData.ShapeType.WALL) {
                g.fill(0);
                g.noStroke();
            } else if (mType.shape_type == BodyUserData.ShapeType.OBSTACLE_BALL) {
                g.fill(0);
                g.noStroke();
            } else if (mType.shape_type == BodyUserData.ShapeType.SENSOR) {
                Sensor mSensor = (Sensor) mType.reference;
                if (mSensor.triggered()) {
                    g.fill(0, 127, 255);
                    g.noStroke();
                } else {
                    g.stroke(0, 127, 255);
                    g.noFill();
                }
            }
        } else {
            g.stroke(0);
            g.noFill();
        }
    }

    public static void handleContact(Contact pContact, boolean pBegan) {
        Fixture mFix_A = pContact.getFixtureA();
        Fixture mFix_B = pContact.getFixtureB();
        FixtureUserData mFUD_A = (FixtureUserData) mFix_A.getUserData();
        FixtureUserData mFUD_B = (FixtureUserData) mFix_B.getUserData();

        if (mFUD_A == null || mFUD_B == null) {
            return;
        }

        if (mFUD_A.getType() == FUD_ROBOT_TIRE && mFUD_B.getType() == FUD_GROUND_AREA) {
            tire_vs_groundArea(mFix_A, mFix_B, pBegan);
        } else if (mFUD_A.getType() == FUD_GROUND_AREA && mFUD_B.getType() == FUD_ROBOT_TIRE) {
            tire_vs_groundArea(mFix_B, mFix_A, pBegan);
        } else if (mFUD_A.getType() == FUD_ROBOT_SENSOR) {
            triggerSensor(mFix_A, pBegan);
        } else if (mFUD_B.getType() == FUD_ROBOT_SENSOR) {
            triggerSensor(mFix_B, pBegan);
        }

    }

    private static void triggerSensor(Fixture mFix_A, boolean pBegan) {
        final BodyUserData mData = (BodyUserData) mFix_A.getBody().getUserData();
        final Sensor mSensor = (Sensor) mData.reference;
        mSensor.setTriggerByCollision(pBegan);
    }

    private static void tire_vs_groundArea(Fixture tireFixture, Fixture groundAreaFixture, boolean began) {
        BodyUserData mData = (BodyUserData) tireFixture.getBody().getUserData();
        Tire tire = (Tire) mData.reference;
        GroundAreaFUD gaFud = (GroundAreaFUD) groundAreaFixture.getUserData();
        if (began) {
            tire.addGroundArea(gaFud);
        } else {
            tire.removeGroundArea(gaFud);
        }
    }

    public void beginContact(Contact contact) {
        handleContact(contact, true);
    }

    public void endContact(Contact contact) {
        handleContact(contact, false);
    }

    public void preSolve(Contact contact, Manifold oldManifold) {
    }

    public void postSolve(Contact contact, ContactImpulse impulse) {
    }

//    public Box2DProcessing box2D() {
//        return mBox2D;
//    }
}
