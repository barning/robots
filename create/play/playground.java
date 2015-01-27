package play;

import de.hfkbremen.robots.challenge.Sensor;
import org.jbox2d.common.Vec2;
import processing.core.PApplet;
import processing.core.PGraphics;
import de.hfkbremen.robots.challenge.*;

import java.util.*;


public class playground extends PApplet{

    Environment mEnvironment;
    MyRobot mRobot;
    float mScale = 0.5f;
    Trace mTrace;
    OSD mOSD;

    public void setup() {
        size(1024, 768);


        mEnvironment = new Environment(this);//, Environment.MAP_RACE_TEST);
        EnvironmentMap mEnvironmentMap = new MyEnvironmentMap(this, mEnvironment);
        mEnvironment.map(mEnvironmentMap);

        //mEnvironment = new Environment(this, Environment.MAP_RANDOM_WALLS);

        mRobot = new MyRobot(mEnvironment);
        mEnvironment.add(mRobot);
        mRobot.position(30, -20);
        mTrace = new Trace(mRobot);
        mOSD = new OSD(this, mEnvironment);
    }

    class MyEnvironmentMap extends EnvironmentMapEditor {
        MyEnvironmentMap(PApplet pParent, Environment pEnvironment) {
            super(pParent, pEnvironment);

            pEnvironment.target().position().x = -380;
            pEnvironment.target().position().y = 270;
        }

        @Override
        public void fillCornersAndBalls() {
            addCorner(0.33984375f, 0.5292969f);
            addCorner(0.42382812f, 0.5566406f);
            addCorner(0.51171875f, 0.5449219f);
            addCorner(0.578125f, 0.51171875f);
            addCorner(0.58203125f, 0.42578125f);
            addCorner(0.5175781f, 0.41601562f);
            addCorner(0.4609375f, 0.37890625f);
            addCorner(0.40234375f, 0.34375f);
            addCorner(0.41601562f, 0.30078125f);
            addCorner(0.47460938f, 0.30273438f);
            addCorner(0.58203125f, 0.3671875f);
            addCorner(0.63671875f, 0.2890625f);
            addCorner(0.609375f, 0.22070312f);
            addCorner(0.56640625f, 0.1328125f);
            addCorner(0.49804688f, 0.095703125f);
            addCorner(0.43164062f, 0.064453125f);
            addCorner(0.34765625f, 0.05078125f);
            addCorner(0.26953125f, 0.052734375f);
            addCorner(0.16601562f, 0.068359375f);
            addCorner(0.087890625f, 0.091796875f);
            addCorner(0.052734375f, 0.1796875f);
            addCorner(0.10546875f, 0.23632812f);
            addCorner(0.17773438f, 0.20898438f);
            addCorner(0.2265625f, 0.14257812f);
            addCorner(0.31445312f, 0.14453125f);
            addCorner(0.4140625f, 0.14648438f);
            addCorner(0.47460938f, 0.17773438f);
            addCorner(0.36328125f, 0.234375f);
            addCorner(0.31835938f, 0.29296875f);
            addCorner(0.21289062f, 0.40039062f);
            addCorner(0.19726562f, 0.49609375f);
            addCorner(0.40039062f, 0.5449219f);

            addBall(0.41992188f, 0.43945312f);
            addBall(0.38085938f, 0.45703125f);
            addBall(0.36914062f, 0.38476562f);
            addBall(0.28125f, 0.44726562f);
            addBall(0.38476562f, 0.26757812f);
            addBall(0.46289062f, 0.27539062f);
            addBall(0.5683594f, 0.30664062f);
            addBall(0.56640625f, 0.21875f);
            addBall(0.51171875f, 0.203125f);
            addBall(0.48828125f, 0.13085938f);
            addBall(0.30273438f, 0.1015625f);
            addBall(0.15625f, 0.103515625f);
            addBall(0.3984375f, 0.080078125f);
            addBall(0.34960938f, 0.05859375f);
            addBall(0.34765625f, 0.125f);
            addBall(0.40429688f, 0.13671875f);
            addBall(0.33007812f, 0.4765625f);
            addBall(0.25195312f, 0.47851562f);
            addBall(0.23046875f, 0.45703125f);
            addBall(0.22851562f, 0.47460938f);
            addBall(0.23828125f, 0.44726562f);
            addBall(0.23828125f, 0.421875f);
            addBall(0.26171875f, 0.46289062f);
            addBall(0.28320312f, 0.49023438f);
            addBall(0.34179688f, 0.5097656f);
            addBall(0.5488281f, 0.44335938f);
            addBall(0.515625f, 0.4296875f);
            addBall(0.46679688f, 0.41992188f);

        }
    }

    public void draw() {
        background(255);
        mEnvironment.update();

        mTrace.update(mEnvironment.delta_time());
        mEnvironment.beginDraw(g, mScale, mRobot.position());
        stroke(0);
        noFill();
        mTrace.draw(g);
        mEnvironment.endDraw(g);
        mOSD.draw(g);
    }

    /**
     * Satus des Robots. Der Roboter kann jeweils nur FORWARD oder BACKWARD einnehmen und LEFT_DIRECTION oder RIGHT_DIRECTION
     */
    public enum ROBO_STATUS {
        FORWARD("forward"), BACKWARD("backward"), STANDING("standing"), LEFT_DIRECTION("left"), RIGHT_DIRECTION("right"),
        NO_STEERING("forward driving");

        private String name;

        ROBO_STATUS(final String name) {
            this.name = name;
        }

        public String toString() {
            return name;
        }
    }

    class MyRobot extends Robot {

        final int FORWARD = 0;
        final int BACKWARD = 1;
        final int STANDING = 2;
        final int LEFT_DIRECTION = 3;
        final int RIGHT_DIRECTION = 4;
        final int NO_STEERING = 5;

        /**
         * Knoten eines Rasters. Wird für A* verwendet.
         */
        class Quad {

            private float mapX;
            private float mapY;

            /**
             * Speichert zusätzlich einen Betrag an Aufwand, der auf die Heuristik in A* addiert wird.
             */
            private float wall;
            private float ball;
            private float unknown;

            /**
             * Heuristic für A*. Speichert die Luftlinie.
             */
            private float h;

            /**
             * f(x) = distToStart + h
             */
            private float f;

            /**
             * Gibt an ob der Node ein Obstacle ist.
             */
            private boolean obstacale;

            /**
             * Vorgängerknoten.
             */
            private Quad predecessor;

            /**
             * Distanz zum Startpunkt.
             */
            private float distToStart;

            /**
             * Farbwert eines Nodes
             */
            private float[] farbe = new float[3];

            /**
             * Zähler für den Knoten, ob es der erste … n-te Knoten ist.
             */
            private int id;

            /**
             * Erzeugt einen neuen Node.
             * @param h Luftlinie + Gefahrenzuschlag
             * @param obstacale Hinderniss ja/nein
             */
            public Quad(final float h, final boolean obstacale, final float mapX,final float mapY) {
                if (h < 0 || mapX < 0 || mapY < 0) throw new IllegalArgumentException();
                this.h = h;
                this.mapX = mapX;
                this.mapY = mapY;
                this.obstacale = obstacale;
                predecessor = null;
                distToStart = 0;
                id = 0;
                f = Float.MAX_VALUE;
                wall = 0;
                ball = 0;
                unknown = 0;
                //Lege Farbe fest
                farbe[0] = 0;
                farbe[1] = 0;
                farbe[2] = 0;
            }

            public int getId() {
                return id;
            }

            public void setId(int id) {
                this.id = id;
            }

            public float getMapX() {
                return mapX;
            }

            public void setMapX(float mapX) {
                this.mapX = mapX;
            }

            public float getMapY() {
                return mapY;
            }

            public void setMapY(float mapY) {
                this.mapY = mapY;
            }

            public float getF() {
                return f;
            }

            public void setF(float f) {
                if (f < 0) throw new IllegalArgumentException();
                this.f = f;
            }

            public float getH() {
                return h;
            }

            public void setH(final float h) {
                if (h < 0) throw new IllegalArgumentException();
                this.h = h;
            }

            public boolean isObstacale() {
                return obstacale;
            }

            public void setObstacale(final boolean obstacale) {
                this.obstacale = obstacale;
            }

            public Quad getPredecessor() {
                return predecessor;
            }

            public void setPredecessor(final Quad predecessor) {
                this.predecessor = predecessor;
            }

            public float getDistToStart() {
                return distToStart;
            }

            public void setDistToStart(final float distToStart) {
                if (distToStart < 0) throw new IllegalArgumentException();
                this.distToStart = distToStart;
            }

            public float getWall() {
                return wall;
            }

            public void setWall(float wall) {
                this.wall = wall;
            }

            public float getBall() {
                return ball;
            }

            public void setBall(float ball) {
                this.ball = ball;
            }

            public float getUnknown() {
                return unknown;
            }

            public void setUnknown(float unknown) {
                this.unknown = unknown;
            }

            /**
             * Gibt Array zurück, der verändert werden darf.
             * @return Array zum verändern der Farbwerte.
             */
            public float[] getFarbe() {
                return farbe;
            }

            @Override
            public String toString() {
                return "Quad{" +
                        "f=" + f +
                        '}';
            }
        }

        /**
         * Gibt die größe eines Rasterfelds an. Gibt auch implizit an, welche abstände zwischen den Knoten existieren.
         */
        private final float QUAD_SIZE_MIN = 1;
        /**
         * Diagonale Länge
         */
        private final float QUAD_SIZE_MAX = sqrt(QUAD_SIZE_MIN*QUAD_SIZE_MIN + QUAD_SIZE_MIN*QUAD_SIZE_MIN);

        /**
         * Breite der A* Map. Muss durch 2 ohne Rest teilbar sein.
         */
        private final int MAP_WIDTH = (int) (1024/ QUAD_SIZE_MIN);
        /**
         * Höhe der A* Map. Muss durch 2 ohne Rest teilbar sein.
         */
        private final int MAP_HEIGHT = (int) (1024/ QUAD_SIZE_MIN);

        /**
         * Raster für A*
         */
        private final Quad[][] map;

        /**
         * Enthält die Nodes, welche noch nicht bearbeitet wurden.
         * ColsedList (Liste der bearbeiteten Nodes) wird indirekt implementiert.
         */
        private final PriorityQueue<Quad> openList;
        /**
         * Ziel Quadrant.
         */
        private Quad targetQuad;

        /**
         * Fahrziel
         */
        private Quad wayPoint;

        /**
         * Id des WayPoints
         */
        private final int MAX_WAY_POINT_ID = 24;

        /**
         * Ziel als Vec2
         */
        private final Vec2 targetPosition;

        private final float RADIUS = 20;
        private final float DEBUG_SCREEN = 80;

        private final float RED = 255;
        private final float GREEN = 255;

        /**
         * Wahl der Zahl: Florian
         */
        private final float EFFORT = 2.8f;
        private final float EFFORT_BALL = 2.8f;
        private final float EFFORT_UNKOWN = 2.8f;

        private final float EFFORT_REDUCTION = 0.08f;

        /**
         * Comparator für PriorityQueue
         */
        private final class QuadComparator implements Comparator<Quad> {

            /**
             * Der Quadrant mit dem kleinsten f Wert hat Priorität.
             * Wenn {@code null} für eines der beiden Node-Objekte übergeben wird, fliegt eine {@link java.lang.IllegalArgumentException}
             *
             * @param quad1 Knoten, der mit Knoten quad2 verglichen werden soll.
             * @param quad2 Knoten, der mit Knoten quad1 verglichen werden soll.
             * @return f von quad1 == f von quad2 -> 0 wird zurückgegeben.
             * f von quad1 < f von quad2 -> ein Wert echt kleiner 0 wird zurückgegeben
             * f von quad1 > f von quad2 -> ein Wert echt größer 0 wird zurückgegeben
             *
             **/
            @Override
            public int compare(Quad quad1, Quad quad2) {
                if (quad1 == null || quad2 == null) throw new IllegalArgumentException();
                float result = quad1.getF() - quad2.getF();
                if (result < 0) {
                    return -1;
                } else if (result > 0) {
                    return 1;
                } else {
                    return 0;
                }
            }
        }

        /**
         * Container für nützliche Daten über den Roboter aus dem letzten Frame
         */
        class LastRobo {

            private float steer;
            private float speed;
            private Vec2 lastPosition;

            public float getSteer() {
                return steer;
            }

            public Vec2 getPosition() {
                return lastPosition;
            }

            public float getSpeed() {
                return speed;
            }

            public LastRobo(final float steer, final Vec2 lastPosition, final float speed) {
                if (lastPosition == null) {
                    throw new IllegalArgumentException();
                }
                this.steer = steer;
                this.lastPosition = new Vec2 (lastPosition);
                this.speed = speed;
            }
        }

        /**
         * RingBuffer für Roboter Daten. Hat feste Größe. NUR push verwenden!!!, wenn man hinzufügen möchte.
         */
        class RingBuffer <E> extends ArrayDeque<E>{

            private final int maxCapacity;

            public RingBuffer(final int maxCapacity) {
                super(maxCapacity);
                if (maxCapacity <= 0) throw new IllegalArgumentException();
                this.maxCapacity = maxCapacity;
            }

            @Override
            public void push(E e) {
                super.push(e);
                if (size() > maxCapacity) {
                    removeLast();
                }
            }
        }

        /**
         * Winkel für Drehung der einzelnen Sensoren
         */
        private float mAngle;

        /**
         * Speichert, ob der Roboter links (LEFT_DIRECTION) oder rechts(RIGHT_DIRECTION) einlenkt.
         * Andere Stati sind nicht zulässig.
         */
        private int direction;

        /**
         * Speichert den Bewegungsstatus FORWARD, BACKWARD.
         * Andere Stati sind nicht zulässig.
         */
        private int status;

        private boolean standing;

        /**
         * Sensoren des Roboters
         */
        private final Sensor[] sensors;

        /**
         * Größe für lastFrameRobos. Entspricht LAST_ROBO_SIZE  Frames.
         */
        private int LAST_ROBO_SIZE = 2;

        /**
         * Die letzten drei Roboter aus den letzten drei Frames.
         */
        private final RingBuffer<LastRobo> lastRobos;

        /**
         * Zählt, wie viele Frames der Roboter steht.
         */
        private int standingCounter;

        /**
         * Die letzen Wegpunkte, die nicht null sind.
         */
        private final ArrayDeque<Quad> wayPoints;

        /**
         * Epsilon für das isStanding-Prädikat
         */
        private final float EPS = 0.02f;

        MyRobot(Environment pEnvironment) {
            super(pEnvironment);

            sensors = new Sensor[5];
            lastRobos = new RingBuffer<>(LAST_ROBO_SIZE);
            map = new Quad[MAP_WIDTH][MAP_HEIGHT];
            openList = new PriorityQueue<>(MAP_WIDTH*MAP_HEIGHT, new QuadComparator());
            targetPosition = mEnvironment.target().position();
            fillMap();
            wayPoint = null;
            wayPoints = new ArrayDeque<>();
            time = 0;
            startTime = 0;
            endTime = 0;
            frame = 0;
            run = true;
            pathQuad = null;

            sensors[0] = addSensor(-PI/6    , maxSensorRange);
            sensors[1] = addSensor(PI/6     , maxSensorRange);
            sensors[2] = addSensor(-PI/2.5f , maxSensorRange);
            sensors[3] = addSensor(PI/2.5f  , maxSensorRange);
            //tentacle
            sensors[4] = addSensor(0, maxSensorRange);


            direction = NO_STEERING;
            status = -1;
            standing = false;
        }

        /**
         * Füllt die Map und initialisiert die einzelnen Quadranten
         */
        private void fillMap() {
            Vec2 targetQuadVec = worldToQuadpoint(targetPosition);

            for (int width = 0; width < MAP_WIDTH; width ++) {
                for (int height = 0; height < MAP_HEIGHT; height++) {
                    map[width][height] = new Quad( targetQuadVec.sub(new Vec2(width, height)).length(), false, width, height);
                }
            }

            targetQuad = map[(int) targetQuadVec.x][(int) targetQuadVec.y];
            targetQuad.setH(0);
        }

        /**
         * Wandelt einen Punkt in map Koordinaten um, sprich den Quadrantenpunkt.
         *
         * @param point muss im Bereich zwischen width und height liegen
         * @return Quadrantenpunkt.
         */
        private Vec2 worldToQuadpoint(Vec2 point) {
            if (point == null) {
                throw new IllegalArgumentException();
            }
            float x = Math.round(point.x) + MAP_WIDTH/2;
            float y = (-1)*((point.y < 0 ? Math.round(point.y) : Math.round(point.y)) - MAP_HEIGHT/2);
            if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) {
                throw new IllegalArgumentException();
            }
            return new Vec2(x,y);
        }

        /**
         * Wandelt einen Punkt in MapKoordinaten in Weltkoordinaten.
         *
         * @param point
         * @return
         */
        private Vec2 quadToWorldpoint(Vec2 point) {
            if (point == null) {
                throw new IllegalArgumentException();
            }

            float x = point.x - MAP_WIDTH/2;
            float y = (-1)*point.y + MAP_HEIGHT/2;
            return new Vec2(x, y);
        }

        /**
         * Timer für Aktionen über Zeit
         */
        private float time;
        private float startTime;
        private float endTime;
        private int frame;
        private Quad pathQuad;
        private boolean run;



        public void update(float pDeltaTime) {

            mAngle += pDeltaTime * 20;

            switch (status) {
                case FORWARD:
                    sensors[0].angle(sin(mAngle * 0.3f) -PI/6);
                    sensors[1].angle(sin(-mAngle * 0.3f) + PI/6);
                    sensors[2].angle(sin(-mAngle * 0.3f) -PI/2.5f);
                    sensors[3].angle(sin(mAngle * 0.3f) + PI/2.5f);
                    sensors[4].angle(sin(mAngle) * 1/2 - steer());
                    break;
                case BACKWARD:
                    sensors[0].angle(sin(mAngle * 0.3f) -5*PI/6);
                    sensors[1].angle(sin(-mAngle * 0.3f) + 5*PI/6);
                    sensors[2].angle(sin(-mAngle * 0.3f) -PI/2f);
                    sensors[3].angle(sin(mAngle * 0.3f) + PI/2f);
                    sensors[4].angle(sin(mAngle) * 1/2 - steer() + PI);
                    break;
                default:
                    sensors[0].angle(sin(mAngle * 0.3f) -PI/6);
                    sensors[1].angle(sin(-mAngle * 0.3f) + PI/6);
                    sensors[2].angle(sin(-mAngle * 0.3f) -PI/2.5f);
                    sensors[3].angle(sin(mAngle * 0.3f) + PI/2.5f);
                    sensors[4].angle(sin(mAngle) * 1/2);
                    break;
            }

            for (Sensor sensor : sensors) {
                if (sensor.triggered()) {
                    //addObstacle(sensor.obstacle(), Sensor.WALL);//sensor.obstacleType());

                    //addObstacle(sensor.obstacle(), sensor.obstacleType());

                    addObstacle(sensor.obstacle(), sensor.obstacleType() == Sensor.UNKNOWN ? Sensor.WALL :
                            sensor.obstacleType() == Sensor.BALL ? Sensor.BALL : Sensor.WALL);//sensor.obstacleType()); //TODO Walls in eigener Map sind nicht immer Walls…
                    // TODO Läuft perfekt wenn diese so wäre */
                }
            }

            Random random = new Random(System.currentTimeMillis());
            if (frame%6 == 0) {
                pathQuad = aStar();
            }
            //pathQuad darf null sein
            int maxWaxPoints = generatePath(pathQuad);
            int delta = Math.round(MAX_WAY_POINT_ID /
                    (speed() < 0 ? maxBackwardSpeed : maxForwardSpeed) * speed());
            int wayPointId = maxWaxPoints - delta - random.nextInt(5);

            float angleToGoal = 0;
            if (!wayPoints.isEmpty()) {
                for (Quad q : wayPoints) {
                    if (q.getId() == wayPointId) {
                        wayPoint = q;
                        angleToGoal = angleToGoal(quadToWorldpoint(new Vec2(wayPoint.getMapX(), wayPoint.getMapY())));
                        steer(status == BACKWARD ? -angleToGoal : angleToGoal);
                        break;
                    }
                }
            }


            run = startTime + endTime < time;
            println(run);
            println("Standing:" + standingCounter);
            if (standingCounter >= 5 && run) {
                println("### Reagiere!!!");
                standing = true;
                startTime = time;
                endTime = 1.5f;
                status = status < 1 ? FORWARD : BACKWARD;
                steer(-steer());
            }
            if (run) {
                if (angleToGoal < -PI/2) {
                    println("hinten");
                    startTime = time;
                    endTime = 1.5f;
                    status = BACKWARD;
                } else if (angleToGoal > PI/2) {
                    println("hinten");
                    startTime = time;
                    endTime = 1.5f;
                    status = BACKWARD;
                } else {
                    startTime = time;
                    endTime = random(2.5f, 4.0f);
                    status = FORWARD;
                }
            }
            println("### startTime: " + startTime);
            println("### time: " + time);
            println("### endTime: " + endTime);
            println("### timer: " + (endTime + startTime));


            Sensor sensor;
            switch (status) {
                case FORWARD:
                    sensor = mostDisturbingFrontObstacle(sensors);

                    if (sensor != null) {
                        speed(maxForwardSpeed * sensor.obstacleDistance() * 2.8f);
                    } else {
                        speed(maxForwardSpeed);
                    }

                    break;
                case BACKWARD:
                    sensor = mostDisturbingBackObstacle(sensors);

                    if (sensor != null) {
                        speed(maxBackwardSpeed * sensor.obstacleDistance() * 1.8f);
                    } else {
                        speed(maxBackwardSpeed);
                    }

                    break;
            }

            /* steer robot and controll its motor */
            if (keyPressed) {
                switch (key) {
                    case 'a':
                        steer(steer() + 0.1f);
                        break;
                    case 'd':
                        steer(steer() - 0.1f);
                        break;
                    case 'w':
                        speed(50);
                        status = FORWARD;
                        break;
                    case 's':
                        speed(0);
                        steer(0);
                        break;
                    case  'x':
                        speed(maxBackwardSpeed);
                        status = BACKWARD;
                        break;
                }
            }

            //Save LastFrameRobo -> Before State Update !!!
            lastRobos.push(new LastRobo(steer(), position(), speed()));
            //Update States
            updateStates();
            time += pDeltaTime; //Auf Overflow achten !!!
            frame++;

        }

        /**
         * Erzeugt ein Route.
         *
         * @param quad
         */
        private int generatePath(Quad quad) {
            wayPoints.clear();

            int id = 0;
            while (quad != null) {
                quad.setId(id++);
                wayPoints.push(quad);
                quad = quad.getPredecessor();
            }
            return id;
        }

        /**
         * Setzt Daten zurück, die von A* verändert wurden.
         */
        private void clearMap() {
            for (int i = 0; i < MAP_WIDTH; i++) {
                for (int j = 0; j < MAP_HEIGHT; j++) {
                    Quad quad = map[i][j];
                    quad.setF(MAX_FLOAT);
                    quad.setPredecessor(null);
                    quad.setDistToStart(0);
                    quad.setId(0);

                    quad.setBall(quad.getBall() - EFFORT_REDUCTION * 0.6f);
                    if (quad.getBall() < 0) {
                        quad.setBall(0);
                    }
                    quad.setUnknown(quad.getUnknown() - EFFORT_REDUCTION * 1.9f);
                    if (quad.getUnknown() < 0) {
                        quad.setUnknown(0);
                    }
                }
            }
        }

        /**
         * Führt A* aus. Target muss erreichbar sein.
         *
         */
        public Quad aStar() {

            clearMap();
            openList.clear();

            Vec2 startPoint = worldToQuadpoint(position());
            Quad startQuad = map[(int) startPoint.x][(int) startPoint.y];
            startQuad.setF(0);
            openList.offer(startQuad);

            while(!openList.isEmpty()) {
                Quad quad = openList.poll();


                //Weg zum Ziel gefunden
                if (quad == targetQuad) {
                    return quad;
                }

                //println("noch in Arbeit");
                expandPath(quad);
            }
            //Wenn kein Ziel gefunden werden konnte
            return null;
        }

        /**
         * Erweitert die openList.
         * @param quad
         */
        private void expandPath(final Quad quad) {
            int mapX = (int) quad.getMapX();
            int mapY = (int) quad.getMapY();

            Quad successor;
            if (validQuad(mapX, ++mapY)) {
                successor = map[mapX][mapY];
                updateSucessor(successor, QUAD_SIZE_MIN, quad);

            }
            if (validQuad(--mapX, mapY)) {
                successor = map[mapX][mapY];
                updateSucessor(successor, QUAD_SIZE_MAX, quad);

            }
            if (validQuad(mapX, --mapY)) {
                successor = map[mapX][mapY];
                updateSucessor(successor, QUAD_SIZE_MIN, quad);

            }
            if (validQuad(mapX, --mapY)) {
                successor = map[mapX][mapY];
                updateSucessor(successor, QUAD_SIZE_MAX, quad);

            }
            if (validQuad(++mapX, mapY)) {
                successor = map[mapX][mapY];
                updateSucessor(successor, QUAD_SIZE_MIN, quad);

            }
            if (validQuad(++mapX, mapY)) {
                successor = map[mapX][mapY];
                updateSucessor(successor, QUAD_SIZE_MAX, quad);

            }
            if (validQuad(mapX, ++mapY)) {
                successor = map[mapX][mapY];
                updateSucessor(successor, QUAD_SIZE_MIN, quad);

            }
            if (validQuad(mapX, ++mapY)) {
                successor = map[mapX][mapY];
                updateSucessor(successor, QUAD_SIZE_MAX, quad);
            }
        }

        /**
         * Checkt ob der Quad noch in der Map liegt.
         *
         * @param mapX
         * @param mapY
         * @return
         */
        private boolean validQuad(final int mapX, final int mapY) {
            if ((mapX < 0 || mapX >= MAP_WIDTH) || (mapY < 0 || mapY >= MAP_HEIGHT)) {
                return false;
            }
            return true;
        }

        /**
         * Update Successor. Nur hinzufügen, wenn der neue f Wert kleiner als der Alte ist.
         *
         * @param successor
         * @param costsInDist
         * @param possiblePredecessor
         */
        private void updateSucessor(final Quad successor, final float costsInDist, final Quad possiblePredecessor) {
            float tentativeDistToStart = successor.getDistToStart() + costsInDist;
            float f = tentativeDistToStart + successor.getH() + successor.getWall() + successor.getBall() + successor.getUnknown();

            if ((!openList.contains(successor) || tentativeDistToStart < successor.getDistToStart())
                    && successor.getF() > f && !successor.isObstacale()) {
                successor.setPredecessor(possiblePredecessor);
                successor.setDistToStart(tentativeDistToStart);
                successor.setF(f);
                if (!openList.contains(successor)) {
                    openList.offer(successor);
                }
            }
        }

        /**
         * Fügt ein Obstacle in die Map ein. In einem Radius um ein Obstacle, wird die Heuristic der Quadranten verändert.
         *
         * @param opstaclePosition Position des Hindernisses
         */
        public void addObstacle(final Vec2 opstaclePosition, final int obstacleTyp) {
            if (opstaclePosition == null) throw new IllegalArgumentException();

            Vec2 obstacleMapPoint = worldToQuadpoint(opstaclePosition);
            Quad obstacle = map[(int) obstacleMapPoint.x][(int) obstacleMapPoint.y];

            int WEST  = (int) ((obstacleMapPoint.x - RADIUS) < 0 ? 0 : (obstacleMapPoint.x - RADIUS));
            int NORTH = (int) ((obstacleMapPoint.y - RADIUS) < 0 ? 0 : (obstacleMapPoint.y - RADIUS));
            int EAST  = (int) ((obstacleMapPoint.x + RADIUS) >= MAP_WIDTH  ? MAP_WIDTH  - 1 : (obstacleMapPoint.x + RADIUS));
            int SOUTH = (int) ((obstacleMapPoint.y + RADIUS) >= MAP_HEIGHT ? MAP_HEIGHT - 1 : (obstacleMapPoint.y + RADIUS));

            //Aufwandsberechnung für Hindernisse:
            //Wände werden NICHT durch andere Hindernisse überlagert
            //Unknown überlagert Ball

            switch (obstacleTyp) {
                case Sensor.UNKNOWN:

                    obstacle.setUnknown(RADIUS * EFFORT_UNKOWN - obstacle.getWall() - obstacle.getBall());

                    for (int i = WEST; i <= EAST; i++) {
                        for (int j = NORTH; j <= SOUTH; j++) {
                            float lenghtToObstacle = new Vec2(i, j).sub(obstacleMapPoint).length();
                            Quad quad = map[i][j];
                            if (lenghtToObstacle <= RADIUS) {

                                float newEffort = (RADIUS-lenghtToObstacle) * EFFORT_UNKOWN - quad.getWall() - quad.getBall();
                                float sumEffortOld = quad.getWall() + quad.getBall() + quad.getUnknown();
                                if (newEffort > sumEffortOld) {
                                    quad.setUnknown(newEffort);
                                }
                            }
                        }
                    }
                    break;
                case Sensor.WALL:
                    obstacle.setWall(RADIUS*EFFORT);
                    obstacle.setBall(0);
                    obstacle.setUnknown(0);
                    obstacle.setObstacale(true);

                    for (int i = WEST; i <= EAST; i++) {
                        for (int j = NORTH; j <= SOUTH; j++) {
                            float lenghtToObstacle = new Vec2(i, j).sub(obstacleMapPoint).length();
                            Quad quad = map[i][j];
                            if (lenghtToObstacle <= RADIUS) {

                                if (lenghtToObstacle <= QUAD_SIZE_MIN || lenghtToObstacle <= QUAD_SIZE_MAX) {
                                    quad.setObstacale(true);
                                }

                                //Der eigentlich Effort ohne die Abzüge der anderen Werte, wird bei Wall verrechnet,
                                // da Wall beständig sein soll
                                float newEffort = (RADIUS-lenghtToObstacle) * EFFORT;
                                float oldWall = quad.getWall();

                                if (newEffort > quad.getWall()) {
                                    quad.setWall(newEffort);

                                    //Ball und Unknown korrigieren:
                                    //Wert ist immer korrigiert nach w+b+u <= EFFORT*RADIUS
                                    float difOldNew = quad.getWall()-oldWall;
                                    quad.setBall(quad.getBall() - difOldNew/2);
                                    quad.setUnknown(quad.getUnknown() - difOldNew / 2);
                                }
                            }
                        }
                    }
                    break;
                case Sensor.BALL:
                    obstacle.setBall(RADIUS * EFFORT_BALL - obstacle.getWall() - obstacle.getUnknown());

                    for (int i = WEST; i <= EAST; i++) {
                        for (int j = NORTH; j <= SOUTH; j++) {
                            float lenghtToObstacle = new Vec2(i, j).sub(obstacleMapPoint).length();
                            Quad quad = map[i][j];
                            if (lenghtToObstacle <= RADIUS) {

                                float newEffort = (RADIUS-lenghtToObstacle) * EFFORT_BALL - quad.getWall() - quad.getUnknown();
                                float sumEffortOld = quad.getWall() + quad.getBall() + quad.getUnknown();
                                if (newEffort > sumEffortOld) {
                                    quad.setBall(newEffort);
                                }
                            }
                        }
                    }
                    break;
            }
        }

        /**
         * Zeichnet alle Obstacles ein und ihre Feldmanipulation.
         */
        public void drawMap() {
            Vec2 roboMapPoint = worldToQuadpoint(position());

            int WEST  = (int) ((roboMapPoint.x - DEBUG_SCREEN) < 0 ? 0 : (roboMapPoint.x - DEBUG_SCREEN));
            int NORTH = (int) ((roboMapPoint.y - DEBUG_SCREEN) < 0 ? 0 : (roboMapPoint.y - DEBUG_SCREEN));
            int EAST  = (int) ((roboMapPoint.x + DEBUG_SCREEN) >= MAP_WIDTH  ? MAP_WIDTH  - 1 : (roboMapPoint.x + DEBUG_SCREEN));
            int SOUTH = (int) ((roboMapPoint.y + DEBUG_SCREEN) >= MAP_HEIGHT ? MAP_HEIGHT - 1 : (roboMapPoint.y + DEBUG_SCREEN));

            noStroke();
            for (int i = WEST; i < EAST; i++) {
                for (int j = NORTH; j < SOUTH; j++) {
                    Quad quad = map[i][j];
                    float sumEffort =  quad.getWall() + quad.getBall() + quad.getUnknown();
                    if (sumEffort > 0) {
                        quad.getFarbe()[0] = RED/(RADIUS * EFFORT) * sumEffort;
                        quad.getFarbe()[1] = GREEN - RED/(RADIUS * EFFORT) * sumEffort;
                        fill(quad.getFarbe()[0], quad.getFarbe()[1], quad.getFarbe()[2]);
                        Vec2 positionInWorld = quadToWorldpoint(new Vec2(i, j));
                        rect(positionInWorld.x, positionInWorld.y, QUAD_SIZE_MIN, QUAD_SIZE_MIN);
                    }
                    if (quad.isObstacale()) {
                        drawQuad(quad, 0, 0, 0);
                    }
                }
            }
        }

        /**
         * Zeichent ein spezielles Quad.
         *
         * Werte müssen valide sein.
         * @param quad
         * @param r
         * @param g
         * @param b
         */
        public void drawQuad(final Quad quad, final float r, final float g, final float b){
            fill(r, g, b);
            Vec2 positionInWorld = quadToWorldpoint(new Vec2(quad.getMapX(), quad.getMapY()));
            rect(positionInWorld.x, positionInWorld.y, QUAD_SIZE_MIN, QUAD_SIZE_MIN);
        }

        /**
         * Zeichnet Strecke von A* beginnt mit dem Ziel
         * @param pathStart
         */
        public void draw_A_STAR(final Quad pathStart) {
            noStroke();
            Quad quad = pathStart;
            while(quad != null) {
                drawQuad(quad, 0, 0, 255);
                quad = quad.getPredecessor();
            }
        }

        /**
         * Upadate der einzelnen States.
         */
        private void updateStates() {

            float middleVelocity = 0;
            int countLeftDirection = 0;
            int countRightDirection = 0;

            //Hier werden alle letzten Roboter geprüft. Hängt von LAST_ROBO_SIZE ab.
            /*
            Iterator<LastRobo> iter = lastRobos.iterator();
            LastRobo lastRobo = iter.hasNext() ? iter.next() : null;
            while (iter.hasNext() && lastRobo != null) {
                LastRobo newerRobo = iter.next();

                middleVelocity += newerRobo.getPosition().sub(lastRobo.getPosition()).length();

                lastRobo = newerRobo;
            }
            middleVelocity /= lastRobos.size(); */

            //Es sollen nur die ersten drei letzten Roboter gezählt werden.

            middleVelocity = lastRobos.getLast().getPosition().sub(lastRobos.getFirst().getPosition()).length();

            Iterator<LastRobo> iter = lastRobos.iterator();
            int count = 0;
            while (iter.hasNext() && count < 4) {
                LastRobo robo = iter.next();
                count++;
                if (robo.getSteer() <= 0) {
                    countLeftDirection++;
                } else {
                    countRightDirection++;
                }
            }

            //Is Stending nach Statustest
            if (middleVelocity < EPS && lastRobos.size() == LAST_ROBO_SIZE) {
                standingCounter++;
            } else {
                standingCounter = 0;
            }


            //Direction test
            if (countLeftDirection > countRightDirection) {
                direction = LEFT_DIRECTION;
            } else if (countLeftDirection < countRightDirection) {
                direction = RIGHT_DIRECTION;
            } else {
                direction = NO_STEERING;
            }

        }

        /**
         * returns the angle to the target. if the target is not defined this method
         * always returns 0.
         *
         * Für einen beliebigen Punkt.
         */
        public float angleToGoal(final Vec2 goal) {
            if (goal != null) {
                Vec2 target = goal;
                Vec2 pos = position();
                Vec2 relTarget = target.sub(pos);
                float relAngle = (float) Math.atan2(relTarget.y, relTarget.x);
                float ownAngle = clamp(body().getAngle());
                float goalAngle = clamp(relAngle - ownAngle - PApplet.PI / 2);
                return goalAngle;
            }
            return 0;
        }

        private float clamp(float goalAngle) {
            while (goalAngle > Math.PI) {
                goalAngle -= 2 * Math.PI;
            }
            while (goalAngle < -Math.PI) {
                goalAngle += 2 * Math.PI;
            }
            return goalAngle;
        }


        private Sensor mostDisturbingFrontObstacle(final Sensor[] sensors) {
            if (sensors == null) throw  new IllegalArgumentException();

            Sensor mostImportantSensor = null;
            float distanceToObstacle = MAX_FLOAT;
            float angleToObstacle = PI; // Ab der Hälfte muss getestet werden

            for (Sensor s : sensors) {
                if (s.triggered()) {
                    float angleToTest = s.angle() % (2 * PI);

                    //Ist der Winkel überhaupt relevant ?
                    if (angleToTest >= PI/2 && angleToTest <= 3*PI/2) {
                        continue;
                    }

                    if (angleToTest > 3*PI/2) {
                        angleToTest = 2*PI - angleToTest;
                    }

                    if (s.obstacleDistance() < distanceToObstacle && angleToTest < angleToObstacle) {
                        distanceToObstacle = s.obstacleDistance();
                        angleToObstacle = angleToTest;
                        mostImportantSensor = s;
                    }
                }
            }
            return mostImportantSensor;
        }

        private Sensor nearestObstacle(final Sensor[] sensors) {
            if (sensors == null) throw  new IllegalArgumentException();

            Sensor mostImportantSensor = null;
            float distanceToObstacle = MAX_FLOAT;

            for (Sensor s : sensors) {
                if (s.triggered()) {

                    if (s.obstacleDistance() < distanceToObstacle) {
                        distanceToObstacle = s.obstacleDistance();
                        mostImportantSensor = s;
                    }
                }
            }
            return mostImportantSensor;
        }

        private Sensor mostDisturbingBackObstacle(final Sensor[] sensors) {

            Sensor mostImportantSensor = null;
            float distanceToObstacle = MAX_FLOAT;
            float angleToObstacle = 2*PI; // Ab der Hälfte muss getestet werden

            for (Sensor s : sensors) {
                if (s.triggered()) {
                    float angleToTest = s.angle() % (2 * PI);

                    //Ist der Winkel überhaupt relevant ?
                    if (angleToTest < PI/2 && angleToTest > 3*PI/2) {
                        continue;
                    }

                    if (angleToTest > PI) {
                        angleToTest = angleToTest - PI;
                    }

                    if (angleToTest <= PI) {
                        angleToTest = PI - angleToTest;
                    }

                    if (s.obstacleDistance() < distanceToObstacle && angleToTest < angleToObstacle) {
                        distanceToObstacle = s.obstacleDistance();
                        angleToObstacle = angleToTest;
                        mostImportantSensor = s;
                    }
                }
            }
            return mostImportantSensor;
        }

        public void draw(PGraphics g) {
        /* draw in robot s local coordinate space */
            rectMode(CENTER);
            noStroke();
            fill(0, 127, 255);


            switch (direction) {
                case LEFT_DIRECTION:
                    triangle(2, 2, 4, 0, 2, -2);
                    break;
                case RIGHT_DIRECTION:
                    triangle(-2, 2, -4, 0, -2, -2);
                    break;
                case NO_STEERING:
                    //Draw Nothing
                    break;
            }

            if (standingCounter >= 5) {
                if (status == FORWARD) {
                    triangle(2, 2, 0, 4, -2, 2);
                } else if (status == BACKWARD) {
                    triangle(2, -2, 0, -4, -2, -2);
                }
            }



            stroke(0);
            //draw a line pointing towards the goal
            float angle = angleToGoal() + PI / 2;
            float distance = distanceToGoal();

            Vec2 mGoalPosition = new Vec2(cos(angle), sin(angle));
            mGoalPosition.mulLocal(5);
            stroke(255, 127, 0);
            line(0, 0, mGoalPosition.x, mGoalPosition.y);
        }

        public void draw_global(PGraphics g) {
            // draw in the environment s coordinate space
            stroke(30);
            Vec2 target = targetPosition;
            line(target.x, target.y, position().x, position().y);

            drawMap();
            draw_A_STAR(targetQuad);
            if (wayPoint != null) {
                drawQuad(wayPoint, 200, 0, 100);
            }
        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[]{playground.class.getName()});
    }
}
