package com.shibabandit.gdx_navmesh.path;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.ai.msg.MessageManager;
import com.badlogic.gdx.ai.msg.Telegraph;
import com.badlogic.gdx.ai.pfa.Heuristic;
import com.badlogic.gdx.ai.pfa.PathFinderQueue;
import com.badlogic.gdx.ai.sched.LoadBalancingScheduler;
import com.badlogic.gdx.math.Intersector;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pools;
import com.shibabandit.gdx_navmesh.util.Angles;
import org.poly2tri.geometry.polygon.Polygon;
import org.poly2tri.triangulation.delaunay.DelaunayTriangle;

import static com.shibabandit.gdx_navmesh.util.DelaunayTriangleUtil.dtContains;
import static com.shibabandit.gdx_navmesh.util.DelaunayTriangleUtil.dtGetEdge;

/**
 * High level path finding interface. Utilizes scheduling and messaging to fulfill path finding requests. Must use the
 * {@link #run(long)} method to satisfy path finding requests. Many fields have been left protected for potential
 * subclassing.
 */
public class NavMeshPathFinder {

    /** Used with A* for distance to goal */
    protected final Heuristic<NavMeshPathNode> heuristic;

    /** Graph to search */
    protected final NavMeshGraph navMeshGraph;

    /** The path finder backend for A* search */
    protected final IndexedNavMeshAStarPathFinder pathFinder;

    /** Path finding request queue */
    protected final PathFinderQueue<NavMeshPathNode> pathFinderQueue;

    /** Scheduler for path finding search */
    protected final LoadBalancingScheduler scheduler;

    /** Message code to use for path-finding requests */
    protected final int requestCode;

    /** Message code to use for path-finding responses */
    protected final int responseCode;

    /** Distance threshold from a point to search for geometry when locating close path nodes */
    protected final float nearbyWalkableTriMaxDist;

    /** Results from 'contained in triangle' search */
    protected final Array<NavMeshGraph.QtTriNode> containedResults;

    /** Results from 'nearby triangle' search */
    protected final Array<NavMeshGraph.QtTriNode> nearbyResults;


    //
    // Reuse vectors
    //

    protected final Vector2 ptA, ptB, offset;

    /**
     * Builds the navigation mesh using the walkable polygons, constructs messaging and scheduling.
     *
     * @param heuristic used with A* for distance to goal
     * @param walkablePolys list of walkable surfaces
     * @param requestCode message code to use for path-finding requests
     * @param responseCode message code to use for path-finding responses
     * @param nearbyWalkableTriMaxDist distance threshold from a point to search for geometry when
     *                                 locating close path nodes
     */
    public NavMeshPathFinder(
            Heuristic<NavMeshPathNode> heuristic,
            Array<Polygon> walkablePolys,
            int requestCode,
            int responseCode,
            float nearbyWalkableTriMaxDist) {

        this.heuristic = heuristic;
        this.requestCode = requestCode;
        this.responseCode = responseCode;
        this.nearbyWalkableTriMaxDist = nearbyWalkableTriMaxDist;
        this.navMeshGraph = new NavMeshGraph(walkablePolys);

        this.pathFinder = new IndexedNavMeshAStarPathFinder(navMeshGraph, true);

        this.pathFinderQueue = new PathFinderQueue<>(pathFinder);
        MessageManager.getInstance().addListener(pathFinderQueue, requestCode);

        this.scheduler = new LoadBalancingScheduler(100);
        this.scheduler.add(pathFinderQueue, 1, 0);

        this.containedResults = new Array<>(10);
        this.nearbyResults = new Array<>(400);

        this.ptA = new Vector2();
        this.ptB = new Vector2();
        this.offset = new Vector2();
    }

    /**
     * @param timeToRun maximum time in nanoseconds path finding should run
     */
    public void run(long timeToRun) {
        scheduler.run(timeToRun);
    }

    /**
     * Request a path finding solution to be received by '{@code telegraph}'.
     *
     * @param startPos starting world position
     * @param endPos ending world position
     * @param agentRadius radius of agent in world units
     * @param telegraph listener for path finding responses
     * @return true if the path finding request was accepted, false if an error occurred
     */
    public boolean findPath(Vector2 startPos, Vector2 endPos, float agentRadius, Telegraph telegraph) {
        boolean success = false;

        final NavMeshGraph.QtTriNode startPosNode = getContainingNode(startPos);
        final NavMeshGraph.QtTriNode endPosNode = getContainingNode(endPos);

        if(startPosNode == null) {
            Gdx.app.debug(NavMeshPathFinder.class.getName(), "Start pos node null for pos: " + startPos);

        } else if(endPosNode == null) {
            Gdx.app.debug(NavMeshPathFinder.class.getName(),"End pos node null for pos: " + endPos);

        } else {

            // Find path
            final NavMeshPathRequest pfRequest = Pools.get(NavMeshPathRequest.class)
                    .obtain().init(heuristic, startPosNode, endPosNode, agentRadius, startPos, endPos, responseCode);

            MessageManager.getInstance().dispatchMessage(telegraph, requestCode, pfRequest);

            success = true;
        }

        return success;
    }

    /**
     * <p>Find the closest walkable point to the supplied '{@code pos}' input. The result is stored in the
     * '{@code result}' input. True is returned if a walkable could be found and '{@code result}' should be used.
     * Otherwise, false is returned and '{@code result}' should NOT be used.</p>
     *
     * <p>SIDE EFFECT: {@code result} may be modified.</p>
     *
     * @param pos position to find walkable point near
     * @param result closest walkable point is stored in this input
     * @param agentRadius radius of agent in world units
     * @param maxAllowedDist maximum distance allowed from '{@code pos}' for an acceptable result
     * @return true if the nearest walkable point could be found
     */
    public boolean getNearestWalkablePoint(Vector2 pos, Vector2 result, float agentRadius, float maxAllowedDist) {
        DelaunayTriangle nextTriangle, bestTriangle = null;
        float nextDist, bestDist = Float.MAX_VALUE;
        int bestEdge = -1;

        nearbyResults.clear();
        navMeshGraph.nodesQt.itemsInRange(pos, nearbyWalkableTriMaxDist, nearbyResults);

        NavMeshGraph.QtTriNode n;
        for(int ni = 0; ni < nearbyResults.size; ++ni) {
            n = nearbyResults.get(ni);

            nextTriangle = n.getDt();
            for(int i = 0; i < 3; ++i) {

                // Only consider constrained edges (edges of non-walkable area)
                if(!nextTriangle.cEdge[i]) {
                    continue;
                }

                // Load next triangle edge
                if(dtGetEdge(nextTriangle, i, ptA, ptB)) {

                    // Compared distance from line to point
                    nextDist = Intersector.distanceSegmentPoint(ptA, ptB, pos);
                    if (nextDist < bestDist && nextDist < maxAllowedDist) {

                        Intersector.nearestSegmentPoint(ptA, ptB, pos, result);

                        // Inset by agent radius
                        final float angleTo = Angles.dumbNormAngle(result.angle(pos));
                        offset.set(1f, 0f).setAngle(angleTo).scl(agentRadius);
                        result.add(offset);

                        // Must be walkable to update best
                        if (isWalkable(result)) {
                            bestTriangle = nextTriangle;
                            bestEdge = i;
                            bestDist = nextDist;
                        }
                    }
                }
            }
        }

        // If a value was found, return true
        return bestTriangle != null && bestEdge > -1;
    }

    /**
     * @param pos position to find containing triangle for
     * @return the containing triangle node for the position, or null if one could not be found
     */
    protected NavMeshGraph.QtTriNode getContainingNode(Vector2 pos) {
        NavMeshGraph.QtTriNode containingNode = null;

        containedResults.clear();
        navMeshGraph.getNodesQt().itemsInRange(pos, 10f, containedResults); // TODO: 10f constant should be param...

        for(NavMeshGraph.QtTriNode n : containedResults) {
            if(dtContains(n.getDt(), pos.x, pos.y)) {
                containingNode = n;
                break;
            }
        }

        return containingNode;
    }

    /**
     * Query internal navigation mesh quadtree index for the containing triangle. If a containing walkable
     * triangle is found, true is returned. Otherwise, false is returned when a containing triangle could not
     * be found.
     *
     * @param pos position to query for 'walkability'
     * @return true if the position is walkable
     */
    public boolean isWalkable(Vector2 pos) {
        return getContainingNode(pos) != null;
    }

    /**
     * @return the underlying navigation mesh graph used for path finding
     */
    public NavMeshGraph getNavMeshGraph() {
        return navMeshGraph;
    }

    /**
     * @return the underlying navigation mesh path finder
     */
    public IndexedNavMeshAStarPathFinder getPathFinder() {
        return pathFinder;
    }
}
