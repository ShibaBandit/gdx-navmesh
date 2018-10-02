/*******************************************************************************
 * Copyright 2014 See AUTHORS file.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

package com.shibabandit.gdx_navmesh.path;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.ai.pfa.*;
import com.badlogic.gdx.ai.pfa.indexed.IndexedAStarPathFinder;
import com.badlogic.gdx.ai.pfa.indexed.IndexedGraph;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.BinaryHeap;
import com.badlogic.gdx.utils.TimeUtils;

import static com.shibabandit.gdx_navmesh.path.NavMeshPathRequest.END_NODE_INDEX;
import static com.shibabandit.gdx_navmesh.path.NavMeshPathRequest.START_NODE_INDEX;

/**
 * <p>Based on the {@link IndexedAStarPathFinder} by davebaol. Adapted due to OOP inheritance issues.</p>
 * <p>
 *     Changes:<br>
 *     <ul>
 *         <li>Only supports NavMesh search.</li>
 *         <li>visitChildren() and search() require NavMeshPortalRequest to propagate agent radius.</li>
 *         <li>Different guard checks in search calls and create paths using connections.</li>
 *         <li>Refactored loop in search() to always check the openList size, replaced do/while with while.</li>
 *     </ul>
 *
 * @author davebaol
 * @author ShibaBandit
 */
public class IndexedNavMeshAStarPathFinder implements PathFinder<NavMeshPathNode> {

    protected IndexedGraph<NavMeshPathNode> graph;
    protected NodeRecord<NavMeshPathNode>[] nodeRecords;
    protected BinaryHeap<NodeRecord<NavMeshPathNode>> openList;
    protected NodeRecord<NavMeshPathNode> current;
    public Metrics metrics;

    /** The unique ID for each search run. Used to mark nodes. */
    private int searchId;

    /** The current request context used for start and end node data. */
    private NavMeshPathRequest currReq;

    /** Avoid re-allocating connection arrays for dynamic start and end locations. */
    private DynamicConnections dynConns;

    public IndexedNavMeshAStarPathFinder(IndexedGraph<NavMeshPathNode> graph) {
        this(graph, false);
    }

    @SuppressWarnings("unchecked")
    public IndexedNavMeshAStarPathFinder(IndexedGraph<NavMeshPathNode> graph, boolean calculateMetrics) {
        this.graph = graph;
        this.nodeRecords = (NodeRecord<NavMeshPathNode>[])new NodeRecord[graph.getNodeCount()];
        this.openList = new BinaryHeap<>();
        if (calculateMetrics) this.metrics = new Metrics();
        this.dynConns = new DynamicConnections();
    }

    @Override
    public boolean searchConnectionPath(NavMeshPathNode startNode,
                                        NavMeshPathNode endNode,
                                        Heuristic<NavMeshPathNode> heuristic,
                                        GraphPath<Connection<NavMeshPathNode>> outPath) {

        // Perform AStar
        search(startNode, endNode, 0f, heuristic);

        // We're here if we've either found the goal, or if we've no more nodes to search, find which
        if (current.node != endNode) {

            // We've run out of nodes without finding the goal, so there's no solution
            return false;
        }

        generateConnectionPath(startNode, outPath);

        return true;
    }

    @Override
    public boolean searchNodePath(NavMeshPathNode startNode,
                                  NavMeshPathNode endNode,
                                  Heuristic<NavMeshPathNode> heuristic,
                                  GraphPath<NavMeshPathNode> outPath) {

        // Perform AStar
        search(startNode, endNode, 0f, heuristic);

        // We're here if we've either found the goal, or if we've no more nodes to search, find which
        if (current.node != endNode) {
            // We've run out of nodes without finding the goal, so there's no solution
            return false;
        }

        generateNodePath(startNode, outPath);

        return true;
    }

    protected void search(NavMeshPathNode startNode,
                          NavMeshPathNode endNode,
                          float agentRadius,
                          Heuristic<NavMeshPathNode> heuristic) {

        initSearch(startNode, endNode, heuristic);

        // Iterate through processing each node
        do {

            // Retrieve the node with smallest estimated total cost from the open list
            current = openList.pop();
            current.category = CLOSED;

            // Terminate if we reached the goal node
            if (current.node == endNode) return;

            visitChildren(endNode, heuristic, agentRadius);

        } while (openList.size > 0);
    }

    @Override
    public boolean search(PathFinderRequest<NavMeshPathNode> request, long timeToRun) {

        if(request instanceof NavMeshPathRequest) {
            final NavMeshPathRequest navMeshPathRequest = (NavMeshPathRequest) request;
            currReq = navMeshPathRequest;

            long lastTime = TimeUtils.nanoTime();

            // We have to initialize the search if the status has just changed
            if (request.statusChanged) {
                initSearch(request.startNode, request.endNode, request.heuristic);
                request.statusChanged = false;
            }

            // Iterate through processing each node
            while (openList.size > 0) {

                // Check the available time
                long currentTime = TimeUtils.nanoTime();
                timeToRun -= currentTime - lastTime;
                if (timeToRun <= PathFinderQueue.TIME_TOLERANCE) return false;

                // Retrieve the node with smallest estimated total cost from the open list
                current = openList.pop();
                current.category = CLOSED;

                // Terminate if we reached the goal node; we've found a path.
                if (current.node == request.endNode) {

                    request.pathFound = true;
                    generateNodePath(request.startNode, request.resultPath);
                    return true;
                }

                // Visit current node's children
                visitChildren(request.endNode, request.heuristic, navMeshPathRequest.getAgentRadius());

                // Store the current time
                lastTime = currentTime;
            }

        } else {
            Gdx.app.error(IndexedNavMeshAStarPathFinder.class.getName(), "Invalid request class - " + request.getClass().getCanonicalName());
        }

        // The open list is empty and we've not found a path.
        request.pathFound = false;
        return true;
    }

    protected void initSearch(NavMeshPathNode startNode,
                              NavMeshPathNode endNode,
                              Heuristic<NavMeshPathNode> heuristic) {

        if (metrics != null) metrics.reset();

        // Increment the search id
        if (++searchId < 0) searchId = 1;

        // Initialize the open list
        openList.clear();

        // Initialize the record for the start node and add it to the open list
        NodeRecord<NavMeshPathNode> startRecord = getNodeRecord(startNode);
        startRecord.node = startNode;
        startRecord.connection = null;
        startRecord.costSoFar = 0;
        addToOpenList(startRecord, heuristic.estimate(startNode, endNode));

        current = null;
    }

    protected void visitChildren(NavMeshPathNode endNode,
                                 Heuristic<NavMeshPathNode> heuristic,
                                 float agentRadius) {

        // Get current node's outgoing connections
        dynConns.init(current, graph, currReq);
        final Array<Connection<NavMeshPathNode>> connections = dynConns.connections;


        // Loop through each connection in turn
        for (int i = 0; i < connections.size; i++) {


            // Skip connection if agent fits through the portal
            final NavMeshPathConn connection = (NavMeshPathConn) connections.get(i);
            if(agentRadius > connection.getToNode().getPortal().getLengthDiv2() && !connection.getToNode().getPortal().isIgnorePortalLength()) {
                continue;
            }


            if (metrics != null) metrics.visitedNodes++;

            // Get the cost estimate for the node
            NavMeshPathNode node = connection.getToNode();
            float nodeCost = current.costSoFar + connection.getCost();

            float nodeHeuristic;
            NodeRecord<NavMeshPathNode> nodeRecord = getNodeRecord(node);
            if (nodeRecord.category == CLOSED) { // The node is closed

                // If we didn't find a shorter route, skip
                if (nodeRecord.costSoFar <= nodeCost) continue;

                // We can use the node's old cost values to calculate its heuristic
                // without calling the possibly expensive heuristic function
                nodeHeuristic = nodeRecord.getEstimatedTotalCost() - nodeRecord.costSoFar;
            } else if (nodeRecord.category == OPEN) { // The node is open

                // If our route is no better, then skip
                if (nodeRecord.costSoFar <= nodeCost) continue;

                // Remove it from the open list (it will be re-added with the new cost)
                openList.remove(nodeRecord);

                // We can use the node's old cost values to calculate its heuristic
                // without calling the possibly expensive heuristic function
                nodeHeuristic = nodeRecord.getEstimatedTotalCost() - nodeRecord.costSoFar;
            } else { // the node is unvisited

                // We'll need to calculate the heuristic value using the function,
                // since we don't have a node record with a previously calculated value
                nodeHeuristic = heuristic.estimate(node, endNode);
            }

            // Update node record's cost and connection
            nodeRecord.costSoFar = nodeCost;
            nodeRecord.connection = connection; // TODO: May need to copy conn contents for dyn links...

            // Add it to the open list with the estimated total cost
            addToOpenList(nodeRecord, nodeCost + nodeHeuristic);
        }

    }

    protected void generateConnectionPath(NavMeshPathNode startNode, GraphPath<Connection<NavMeshPathNode>> outPath) {

        // Work back along the path, accumulating connections
        // outPath.clear();
        while (current.node != startNode) {
            outPath.add(current.connection);
            current = nodeRecords[current.connection.getFromNode().getIndex()];
        }

        // Reverse the path
        outPath.reverse();
    }

    protected void generateNodePath(NavMeshPathNode startNode, GraphPath<NavMeshPathNode> outPath) {

        // Work back along the path, accumulating nodes
        // outPath.clear();
        while (current.connection != null) {
            outPath.add(current.node);
            int index = current.connection.getFromNode().getIndex();
            if(index == NavMeshPathRequest.START_NODE_INDEX) {
                current = currReq.getStartNodeRec();

            } else if(index == NavMeshPathRequest.END_NODE_INDEX) {
                current = currReq.getEndNodeRec();

            } else {
                current = nodeRecords[index];
            }
        }
        outPath.add(startNode);

        // Reverse the path
        outPath.reverse();
    }

    protected void addToOpenList(NodeRecord<NavMeshPathNode> nodeRecord, float estimatedTotalCost) {
        openList.add(nodeRecord, estimatedTotalCost);
        nodeRecord.category = OPEN;
        if (metrics != null) {
            metrics.openListAdditions++;
            metrics.openListPeak = Math.max(metrics.openListPeak, openList.size);
        }
    }

    protected NodeRecord<NavMeshPathNode> getNodeRecord(NavMeshPathNode node) {
        NodeRecord<NavMeshPathNode> nr;
        int index = node.getIndex();

        if(index == START_NODE_INDEX) {
            nr = currReq.getStartNodeRec();

            if(nr.searchId != searchId) {
                nr.searchId = searchId;
            }

        } else if(index == END_NODE_INDEX) {
            nr = currReq.getEndNodeRec();

            if(nr.searchId != searchId) {
                nr.searchId = searchId;
            }

        } else {

            nr = nodeRecords[index]; // TODO: EMPTY NODE RECORDS CAUSES CRASH HERE, NO GEOMETRY IN WORLD

            if(nr == null) {
                nr = nodeRecords[index] = new NodeRecord<>();
                nr.node = node;
                nr.searchId = searchId;

            } else if(nr.searchId != searchId) {
                nr.category = UNVISITED;
                nr.searchId = searchId;
            }
        }

        return nr;
    }

    /** Status for {@link NodeRecord} */
    public static final int UNVISITED = 0;

    /** Status for {@link NodeRecord} */
    public static final int OPEN = 1;

    /** Status for {@link NodeRecord} */
    public static final int CLOSED = 2;

    /**
     * @param <NavMeshPathNode>
     * @author davebaol
     */
    public static class NodeRecord<NavMeshPathNode> extends BinaryHeap.Node {

        /** The reference to the node. */
        NavMeshPathNode node;

        /** The incoming connection to the node */
        Connection<NavMeshPathNode> connection;

        /** The actual cost from the start node. */
        float costSoFar;

        /** The node category: {@link #UNVISITED}, {@link #OPEN} or {@link #CLOSED}. */
        int category;

        /** ID of the current search. */
        int searchId;

        /** Creates a {@code NodeRecord}. */
        public NodeRecord () {
            super(0);
        }

        /** Returns the estimated total cost. */
        public float getEstimatedTotalCost () {
            return getValue();
        }
    }

    /**
     * A class used by {@link IndexedAStarPathFinder} to collect search metrics.
     *
     * @author davebaol
     */
    public static class Metrics {
        public int visitedNodes;
        public int openListAdditions;
        public int openListPeak;

        public Metrics () {}

        public void reset () {
            visitedNodes = 0;
            openListAdditions = 0;
            openListPeak = 0;
        }
    }

    /**
     * Provide dynamic connections based on starting and ending graph positions which are different
     * between searches and do not require constructing an entirely new search graph.
     */
    private static class DynamicConnections {
        private static final int MAX_CONN = 2 + 4; // start, end, up to 4 normal connections

        public NavMeshPathConn start, end;
        public NavMeshPortal startPortal, endPortal;
        public final Array<Connection<NavMeshPathNode>> connections;

        public DynamicConnections() {
            connections = new Array<>(MAX_CONN);
            start = new NavMeshPathConn();
            end = new NavMeshPathConn();
            startPortal = new NavMeshPortal();
            endPortal = new NavMeshPortal();
        }

        public DynamicConnections init(NodeRecord<NavMeshPathNode> current, IndexedGraph<NavMeshPathNode> graph, NavMeshPathRequest currReq) {
            connections.clear();

            // Get current node's outgoing connections
            int currNodeIndex = current.node.getIndex();

            if(currNodeIndex == NavMeshPathRequest.END_NODE_INDEX) {
                connections.addAll(currReq.endNode.connections);

            } else if(currNodeIndex == NavMeshPathRequest.START_NODE_INDEX) {
                connections.addAll(currReq.startNode.connections);

            } else {
                connections.addAll(graph.getConnections(current.node));

                final Array<Connection<NavMeshPathNode>> endNodeConns = currReq.endNode.getConnections();
                final Array<Connection<NavMeshPathNode>> startNodeConns = currReq.startNode.getConnections();
                boolean connectEndNode = false;
                boolean connectStartNode = false;
                for(Connection<NavMeshPathNode> nextConn : endNodeConns) {
                    if(nextConn.getToNode().getIndex() == currNodeIndex) {
                        connectEndNode = true;
                        break;
                    }
                }
                for(Connection<NavMeshPathNode> nextConn : startNodeConns) {
                    if(nextConn.getToNode().getIndex() == currNodeIndex) {
                        connectStartNode = true;
                        break;
                    }
                }
                if(connectEndNode) {
                    currReq.endNode.getPortal().setIgnorePortalLength(true);
                    end.init(current.node, currReq.endNode);
                    connections.add(end);
                }
                if(connectStartNode) {
                    currReq.startNode.getPortal().setIgnorePortalLength(true);
                    start.init(current.node, currReq.startNode);
                    connections.add(start);
                }
            }

            return this;
        }
    }
}
