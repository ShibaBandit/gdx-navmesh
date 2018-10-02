package com.shibabandit.gdx_navmesh.path;

import com.badlogic.gdx.ai.pfa.Heuristic;

/**
 * A* heuristic using the distance between the portal midpoints.
 */
public final class PortalMidpointDistHeuristic implements Heuristic<NavMeshPathNode> {

    @Override
    public float estimate(NavMeshPathNode node, NavMeshPathNode endNode) {
        return node.getPortal().getMidpoint().dst(endNode.getPortal().getMidpoint());
    }
}
