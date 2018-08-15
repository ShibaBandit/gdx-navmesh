package com.shibabandit.gdx_navmesh.examples;

import com.badlogic.gdx.ApplicationListener;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputAdapter;
import com.badlogic.gdx.ai.msg.Telegram;
import com.badlogic.gdx.ai.msg.Telegraph;
import com.badlogic.gdx.ai.pfa.Connection;
import com.badlogic.gdx.ai.pfa.DefaultGraphPath;
import com.badlogic.gdx.backends.lwjgl.LwjglApplication;
import com.badlogic.gdx.backends.lwjgl.LwjglApplicationConfiguration;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Polygon;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.scenes.scene2d.Stage;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.FloatArray;
import com.badlogic.gdx.utils.Pools;
import com.badlogic.gdx.utils.ShortArray;
import com.badlogic.gdx.utils.viewport.StretchViewport;
import com.shibabandit.gdx_navmesh.coll.CollUtil;
import com.shibabandit.gdx_navmesh.path.*;
import org.poly2tri.Poly2Tri;
import org.poly2tri.geometry.polygon.PolygonPoint;
import org.poly2tri.triangulation.TriangulationPoint;
import org.poly2tri.triangulation.delaunay.DelaunayTriangle;

public class NavMeshTest implements ApplicationListener, Telegraph {


    //
    // Visual test params
    //

    private static final float VIEWPORT_WIDTH = 500f;
    private static final float VIEWPORT_HEIGHT = 500f;
    private static final float WALKABLE_BUFFER = 5f;
    private static final Color COLOR_WALKABLE = new Color(0f, 1f, 0f, 1f);
    private static final Color COLOR_OBS = new Color(1f, 0f, 0f, 1f);
    private static final Color COLOR_CLICKED = new Color(.5f, 1f, 1f, 1f);
    private static final Color COLOR_PORTAL = new Color(0f, .5f, .5f, 1f);
    private static final float AGENT_RADIUS = 10f;


    //
    // Message types
    //

    /** Message type for path request */
    private final static int PF_REQUEST = 1;

    /** Message type for path response */
    private final static int PF_RESPONSE = 2;


    public static void main(String[] argv) {
        LwjglApplicationConfiguration config = new LwjglApplicationConfiguration();
        config.title = "navmesh test";
        config.width = 800;
        config.height = 480;
        config.samples = 4;
        config.depth = 0;
        config.vSyncEnabled = true;

        config.fullscreen = false;
        new LwjglApplication(new NavMeshTest(), config);
    }

    private Stage stage;
    private OrthographicCamera camera;
    private ShapeRenderer shapeRenderer;

    private FloatArray points;
    private DefaultGraphPath<NavMeshPathNode> latestPath;

    org.poly2tri.geometry.polygon.Polygon ptWalkable;
    Array<org.locationtech.jts.geom.Polygon> obstacles;
    Array<org.poly2tri.geometry.polygon.Polygon> walkables;

    private Vector2 startPosWorld, endPosWorld;

    private NavMeshPathFinder navMeshPathFinder;
    private NavMeshStringPuller navMeshStringPuller;

    private Array<Vector2> pathPts;

    @Override
    public void create() {
        camera = new OrthographicCamera(VIEWPORT_WIDTH, VIEWPORT_HEIGHT);

        stage = new Stage(new StretchViewport(VIEWPORT_WIDTH, VIEWPORT_HEIGHT, camera));

        camera.position.set(VIEWPORT_WIDTH / 2f, VIEWPORT_HEIGHT / 2f, 0);
        camera.update();

        shapeRenderer = new ShapeRenderer();
        shapeRenderer.setAutoShapeType(true);

        pathPts = null;

        points = new FloatArray();

        addRandomPoints(points, 100);


        ptWalkable = new org.poly2tri.geometry.polygon.Polygon(
                new PolygonPoint[] {
                        new PolygonPoint(VIEWPORT_WIDTH-WALKABLE_BUFFER, VIEWPORT_HEIGHT-WALKABLE_BUFFER),
                        new PolygonPoint(VIEWPORT_WIDTH-WALKABLE_BUFFER, 0f+WALKABLE_BUFFER),
                        new PolygonPoint(0f+WALKABLE_BUFFER, 0f+WALKABLE_BUFFER),
                        new PolygonPoint(0f+WALKABLE_BUFFER, VIEWPORT_HEIGHT-WALKABLE_BUFFER)
                }
        );
        org.poly2tri.geometry.polygon.Polygon ptObsA = new org.poly2tri.geometry.polygon.Polygon(
                new PolygonPoint[] {
                        new PolygonPoint(250f, 400f),
                        new PolygonPoint(400f, 100f),
                        new PolygonPoint(100f, 100f)
                }
        );
        org.poly2tri.geometry.polygon.Polygon ptObsB = new org.poly2tri.geometry.polygon.Polygon(
                new PolygonPoint[] {
                        new PolygonPoint(350f, 500f),
                        new PolygonPoint(500f, 200f),
                        new PolygonPoint(200f, 200f)
                }
        );
        org.poly2tri.geometry.polygon.Polygon ptObsC = new org.poly2tri.geometry.polygon.Polygon(
                new PolygonPoint[] {
                        new PolygonPoint(0f, 250f),
                        new PolygonPoint(100f, 300f),
                        new PolygonPoint(100f, 250f)
                }
        );

        NavMeshClipper navMeshClipper = new NavMeshClipper();
        obstacles = new Array<>();
        obstacles.add(CollUtil.toJtsPoly(ptObsA));
        obstacles.add(CollUtil.toJtsPoly(ptObsB));
        obstacles.add(CollUtil.toJtsPoly(ptObsC));

        walkables = navMeshClipper.clipToWalkables(ptWalkable, obstacles);

        for(org.poly2tri.geometry.polygon.Polygon walkable : walkables) {
            Poly2Tri.triangulate(walkable);
        }

        // Start and end contained within same triangle on left side
        startPosWorld = new Vector2(7.0625f, 11.354166f);
        endPosWorld = new Vector2(36.75f, 34.895836f);

        navMeshPathFinder = new NavMeshPathFinder(new DistSqdHeuristic(), walkables, PF_REQUEST,
                PF_RESPONSE,800f);
        navMeshStringPuller = new NavMeshStringPuller();

        final NavMeshTest self = this;
        Gdx.input.setInputProcessor(new InputAdapter() {
            @Override
            public boolean touchDown(int screenX, int screenY, int pointer, int button) {
                if(button == 0) {
                    endPosWorld.set(screenX, screenY);
                    stage.screenToStageCoordinates(endPosWorld);
                    navMeshPathFinder.findPath(startPosWorld, endPosWorld, AGENT_RADIUS, self);

                } else if(button == 1) {
                    startPosWorld.set(screenX, screenY);
                    stage.screenToStageCoordinates(startPosWorld);
                    navMeshPathFinder.findPath(startPosWorld, endPosWorld, AGENT_RADIUS, self);
                }

                return true;
            }
        });
    }

    private static void addRandomPoints(FloatArray points, int pointCount) {
        MathUtils.random.setSeed(4139368480425561099L);

        for(int i = 0; i < pointCount; i++) {
            float value;
            do {
                value = MathUtils.random(0f, VIEWPORT_WIDTH);
            } while (points.contains(value));
            points.add(value);
            do {
                value = MathUtils.random(0f, VIEWPORT_HEIGHT);
            } while (points.contains(value));
            points.add(value);
        }
    }

    private static void addPolyPoints(FloatArray points, Polygon poly) {
        points.addAll(poly.getVertices());
    }

    @Override
    public void resize(int width, int height) {}

    @Override
    public void render() {
        Gdx.gl.glClearColor(0.3f, 0.3f, 0.3f, 1);
        Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);

        navMeshPathFinder.run(10000L);

        shapeRenderer.setProjectionMatrix(camera.combined);

        if(walkables != null) {
            for(org.poly2tri.geometry.polygon.Polygon walkable : walkables) {
                drawTriangleMesh(walkable, COLOR_WALKABLE);
            }
        }

        if(latestPath != null) {
            draw(latestPath, COLOR_CLICKED, COLOR_PORTAL);
        }

//        for(org.poly2tri.geometry.polygon.Polygon o : obstacles) {
//            draw(o, COLOR_OBS, ShapeRenderer.ShapeType.Line);
//        }
    }

    @Override
    public void pause() {}

    @Override
    public void resume() {}

    @Override
    public void dispose() {}

    /**
     * Draw a colored polygon
     *
     * @param polygon what to draw
     * @param color color to draw poly
     */
    private void draw(Polygon polygon, Color color, ShapeRenderer.ShapeType shapeType) {
        shapeRenderer.begin(shapeType);
        shapeRenderer.setColor(color);
        shapeRenderer.polygon(polygon.getVertices());
        shapeRenderer.end();
    }

    /**
     * Draw a colored polygon
     *
     * @param polygon what to draw
     * @param color color to draw poly
     */
    private void draw(org.poly2tri.geometry.polygon.Polygon polygon, Color color, ShapeRenderer.ShapeType shapeType) {
        shapeRenderer.begin(shapeType);
        shapeRenderer.setColor(color);
        for(int i = 0; i < polygon.getPoints().size(); ++i) {
            final TriangulationPoint ptA = polygon.getPoints().get(i);
            final TriangulationPoint ptB = polygon.getPoints().get((i + 1) % polygon.getPoints().size());
            shapeRenderer.line(ptA.getXf(), ptA.getYf(), ptB.getXf(), ptB.getYf());
        }
        shapeRenderer.end();
    }

    private void drawTriangleMesh(ShortArray triangles, FloatArray points, Color meshColor) {
        shapeRenderer.begin();
        shapeRenderer.setColor(meshColor);
        for(int i = 0; i < triangles.size; i += 3) {
            int p1 = triangles.get(i) * 2;
            int p2 = triangles.get(i + 1) * 2;
            int p3 = triangles.get(i + 2) * 2;
            shapeRenderer.triangle(
                    points.get(p1), points.get(p1 + 1),
                    points.get(p2), points.get(p2 + 1),
                    points.get(p3), points.get(p3 + 1));
        }
        shapeRenderer.end();
    }

    private void drawTriangleMesh(org.poly2tri.geometry.polygon.Polygon p, Color meshColor) {
        shapeRenderer.begin();
        shapeRenderer.setColor(meshColor);
        for(DelaunayTriangle dt : p.getTriangles()) {
            shapeRenderer.triangle(
                    dt.points[0].getXf(), dt.points[0].getYf(),
                    dt.points[1].getXf(), dt.points[1].getYf(),
                    dt.points[2].getXf(), dt.points[2].getYf());
        }
        shapeRenderer.end();
    }

    private void draw(DefaultGraphPath<NavMeshPathNode> path, Color meshColor, Color portalColor) {
        DelaunayTriangle dt;
        shapeRenderer.begin();
        shapeRenderer.setColor(meshColor);
        for(NavMeshPathNode n : path) {
            dt = n.getDelaunayTriangle();
            shapeRenderer.triangle(
                    dt.points[0].getXf(), dt.points[0].getYf(),
                    dt.points[1].getXf(), dt.points[1].getYf(),
                    dt.points[2].getXf(), dt.points[2].getYf());
        }


        // 'Portals' test
        // Have to reconstruct the edges due to API limitations from gdx-ai extension
        NavMeshPathNode nextSrc, nextDest;
        Array<Connection<NavMeshPathNode>> srcConns;
        NavMeshPortal nextPortal;

        shapeRenderer.setColor(portalColor);
        for(int i = 0; i < path.getCount() - 1; ++i) {
            nextSrc = path.get(i);
            nextDest = path.get(i + 1);
            srcConns = nextSrc.getConnections();

            for(Connection<NavMeshPathNode> nextConn : srcConns) {
                if(nextConn.getToNode() == nextDest) {
                    nextPortal = ((NavMeshPathConn) nextConn).getPortal();

                    // Draw portal
                    shapeRenderer.line(nextPortal.getLeft(), nextPortal.getRight());

                    break;
                }
            }
        }


        // String pull test
        if(pathPts != null) {
            shapeRenderer.setColor(Color.ORANGE);

            Vector2 nextPt;
            for(int i = 0; i < pathPts.size; ++i) {
                nextPt = pathPts.get(i);
                shapeRenderer.circle(nextPt.x, nextPt.y, 1f);
            }
        }

        shapeRenderer.setColor(Color.PINK);
        shapeRenderer.circle(startPosWorld.x, startPosWorld.y, 1f);
        shapeRenderer.circle(endPosWorld.x, endPosWorld.y, 1f);

        shapeRenderer.end();
    }

    @Override
    public boolean handleMessage(Telegram telegram) {
        switch (telegram.message) {

            // PathFinderQueue will call us directly, no need to register for this message
            case PF_RESPONSE:
                final NavMeshPathRequest pfr = (NavMeshPathRequest)telegram.extraInfo;
                latestPath = (DefaultGraphPath<NavMeshPathNode>) pfr.resultPath;

                if(latestPath.getCount() > 0) {
                    final NavMeshPortal[] portals = NavMeshStringPuller.pathToPortals(latestPath, startPosWorld, endPosWorld);
                    pathPts = navMeshStringPuller.stringPull(startPosWorld, endPosWorld, portals, AGENT_RADIUS);

                    final StringBuilder sb = new StringBuilder("Path Through Portals: ");
                    for(int i = 0; i < pathPts.size; ++i) {
                        sb.append(pathPts.get(i));
                        sb.append(", ");
                    }
                    System.out.println(sb);

                } else {
                    pathPts = null;
                    System.out.println("Path not found");
                }

                // Release the request
                Pools.free(pfr);
                break;
        }
        return true;
    }
}
