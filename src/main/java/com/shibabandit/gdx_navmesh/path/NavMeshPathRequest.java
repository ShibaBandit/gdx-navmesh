package com.shibabandit.gdx_navmesh.path;

import com.badlogic.gdx.ai.pfa.DefaultGraphPath;
import com.badlogic.gdx.ai.pfa.Heuristic;
import com.badlogic.gdx.ai.pfa.PathFinderRequest;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Pool;

/**
 * Adds agent {@link #agentRadius} to pathfinding request and intended for use
 * with {@link com.badlogic.gdx.utils.Pool.Poolable}. Has request and pathfinding context needed
 * for search.
 */
public class NavMeshPathRequest extends PathFinderRequest<NavMeshPathNode> implements Pool.Poolable {

    public static final int START_NODE_INDEX = -998;
    public static final int END_NODE_INDEX = -999;

    /** Radius of agent in world units */
    private float agentRadius;

    /** Starting position (valid non-node pos) */
    private Vector2 startPos;

    /** Ending position (valid non-node pos) */
    private Vector2 endPos;

    private NavMeshPathNode startDynNode;
    private NavMeshPathNode endDynNode;

    private IndexedNavMeshAStarPathFinder.NodeRecord<NavMeshPathNode> startNodeRec;
    private IndexedNavMeshAStarPathFinder.NodeRecord<NavMeshPathNode> endNodeRec;

    public NavMeshPathRequest() {
        this.resultPath = new DefaultGraphPath<>();
        this.startPos = new Vector2();
        this.endPos = new Vector2();
        this.startDynNode = new NavMeshPathNode(START_NODE_INDEX, new NavMeshPortal());
        this.endDynNode = new NavMeshPathNode(END_NODE_INDEX, new NavMeshPortal());
        this.startNodeRec = new IndexedNavMeshAStarPathFinder.NodeRecord<>();
        this.endNodeRec = new IndexedNavMeshAStarPathFinder.NodeRecord<>();
    }

    /**
     * Initialize pooled object.
     *
     * @param heuristic A* heuristic for request
     * @param startTriNode beginning path node
     * @param endTriNode ending path node
     * @param agentRadius radius of agent in world units
     * @param startPos starting position (valid non-node pos)
     * @param endPos ending position (valid non-node pos)
     * @param responseCode the message code to use for the path finding response
     * @return initialized poolable object
     */
    public NavMeshPathRequest init(
            Heuristic<NavMeshPathNode> heuristic,
            NavMeshGraph.QtTriNode startTriNode,
            NavMeshGraph.QtTriNode endTriNode,
            float agentRadius,
            Vector2 startPos,
            Vector2 endPos,
            int responseCode) {

        this.responseMessageCode = responseCode;
        this.startNode = startDynNode;
        this.endNode = endDynNode;
        this.heuristic = heuristic;
        this.agentRadius = agentRadius;
        this.startPos.set(startPos);
        this.endPos.set(endPos);


        this.startDynNode.getPortal().init(startPos, startPos, true);
        this.startDynNode.getConnections().clear();
        for(NavMeshPathNode n : startTriNode.getNodes()) {
            this.startDynNode.getConnections().add(new NavMeshPathConn(startDynNode, n));
        }

        this.endDynNode.getPortal().init(endPos, endPos, true);
        this.endDynNode.getConnections().clear();
        for(NavMeshPathNode n : endTriNode.getNodes()) {
            this.endDynNode.getConnections().add(new NavMeshPathConn(endDynNode, n));
        }

        if(startTriNode == endTriNode) {
            this.startDynNode.getConnections().add(new NavMeshPathConn(this.startDynNode, this.endDynNode));
            this.endDynNode.getConnections().add(new NavMeshPathConn(this.endDynNode, this.startDynNode));
        }

        startNodeRec.node = startNode;
        startNodeRec.category = IndexedNavMeshAStarPathFinder.UNVISITED;
        endNodeRec.node = endNode;
        endNodeRec.category = IndexedNavMeshAStarPathFinder.UNVISITED;

        return this;
    }

    @Override
    public boolean initializeSearch(long timeToRun) {
        resultPath.clear();
        return true;
    }

    @Override
    public boolean finalizeSearch(long timeToRun) {
        return true;
    }

    @Override
    public void reset () {
        this.startNode = null;
        this.endNode = null;
        this.heuristic = null;
        this.client = null;
        this.agentRadius = 0f;
    }

    /**
     * @return radius of agent in world units
     */
    public float getAgentRadius() {
        return agentRadius;
    }

    /**
     * @return Starting position (valid non-node pos)
     */
    public Vector2 getStartPos() {
        return startPos;
    }

    /**
     * @return Ending position (valid non-node pos)
     */
    public Vector2 getEndPos() {
        return endPos;
    }

    public IndexedNavMeshAStarPathFinder.NodeRecord<NavMeshPathNode> getStartNodeRec() {
        return startNodeRec;
    }

    public IndexedNavMeshAStarPathFinder.NodeRecord<NavMeshPathNode> getEndNodeRec() {
        return endNodeRec;
    }
}
