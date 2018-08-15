package com.shibabandit.gdx_navmesh.path;

import com.badlogic.gdx.ai.pfa.DefaultGraphPath;
import com.badlogic.gdx.ai.pfa.Heuristic;
import com.badlogic.gdx.ai.pfa.PathFinderRequest;
import com.badlogic.gdx.utils.Pool;

/**
 * Adds agent {@link #agentRadius} to pathfinding request and intended for use
 * with {@link com.badlogic.gdx.utils.Pool.Poolable}.
 */
public class NavMeshPathRequest extends PathFinderRequest<NavMeshPathNode> implements Pool.Poolable {

    /** Radius of agent in world units */
    private float agentRadius;

    public NavMeshPathRequest() {
        this.resultPath = new DefaultGraphPath<>();
    }

    /**
     * Initialize pooled object.
     *
     * @param heuristic A* heuristic for request
     * @param startNode beginning path node
     * @param endNode ending path node
     * @param agentRadius radius of agent in world units
     * @param responseCode the message code to use for the path finding response
     * @return initialized poolable object
     */
    public NavMeshPathRequest init(
            Heuristic<NavMeshPathNode> heuristic,
            NavMeshPathNode startNode,
            NavMeshPathNode endNode,
            float agentRadius,
            int responseCode) {

        this.responseMessageCode = responseCode;
        this.startNode = startNode;
        this.endNode = endNode;
        this.heuristic = heuristic;
        this.agentRadius = agentRadius;
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
}
