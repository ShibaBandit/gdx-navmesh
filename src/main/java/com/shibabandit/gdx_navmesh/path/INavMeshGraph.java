package com.shibabandit.gdx_navmesh.path;

import com.badlogic.gdx.ai.pfa.indexed.IndexedGraph;

/**
 * @param <N> 
 */
public interface INavMeshGraph<N extends NavMeshPathNode> extends IndexedGraph<NavMeshPathNode> {

    /**
     * @param portal
     * @return the navigation mesh node associated with the provided portal
     */
    N getNode(NavMeshPortal portal);
}
