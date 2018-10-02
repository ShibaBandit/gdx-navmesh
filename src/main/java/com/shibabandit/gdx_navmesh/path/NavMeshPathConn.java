package com.shibabandit.gdx_navmesh.path;

import com.badlogic.gdx.ai.pfa.DefaultConnection;

/**
 * A connection between two graph nodes. The {@link #cost} represents a straight line distane from
 * one node {@link NavMeshPortal#getMidpoint()} to another.
 */
public class NavMeshPathConn extends DefaultConnection<NavMeshPathNode> {

    /** Distance between portal midpoints */
    private float cost;

    public NavMeshPathConn() {
        super(null, null);
    }

    public NavMeshPathConn(NavMeshPathNode fromNode, NavMeshPathNode toNode) {
        super(fromNode, toNode);
        init(fromNode, toNode);
    }

    public NavMeshPathConn init(NavMeshPathNode fromNode, NavMeshPathNode toNode) {
        this.fromNode = fromNode;
        this.toNode = toNode;
        this.cost = fromNode.getPortal().getMidpoint().dst(toNode.getPortal().getMidpoint());
        return this;
    }

    @Override
    public float getCost() {
        return cost;
    }
}