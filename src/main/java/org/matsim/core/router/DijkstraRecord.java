/* *********************************************************************** *
 * project: org.matsim.*
 * AStarEuclidean.java
 *                                                                         *
 * *********************************************************************** *
 *                                                                         *
 * copyright       : (C) 2007 by the members listed in the COPYING,        *
 *                   LICENSE and WARRANTY file.                            *
 * email           : info at matsim dot org                                *
 *                                                                         *
 * *********************************************************************** *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *   See also COPYING, LICENSE and WARRANTY file                           *
 *                                                                         *
 * *********************************************************************** */

package org.matsim.core.router;

import org.apache.log4j.Logger;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.api.core.v01.network.Node;
import org.matsim.core.router.util.DijkstraNodeData;
import org.matsim.core.router.util.PreProcessDijkstra;
import org.matsim.core.router.util.TravelDisutility;
import org.matsim.core.router.util.TravelTime;
import org.matsim.core.router.util.*;
import org.matsim.core.utils.collections.RouterPriorityQueue;
import org.matsim.core.utils.geometry.CoordUtils;

import org.matsim.api.core.v01.population.Person;
import org.matsim.vehicles.Vehicle;

/**
 * Implements the <a href="http://en.wikipedia.org/wiki/A%2A">A* router algorithm</a>
 * for a given NetworkLayer by using the euclidean distance divided by the
 * maximal free speed per length unit as the heuristic estimate during routing.
 *
 * AStarEuclidean about 3 times faster than Dijkstra.<br />
 *
 * <p>For every router, there exists a class which computes some
 * preprocessing data and is passed to the router class
 * constructor in order to accelerate the routing procedure.
 * The one used for AStarEuclidean is org.matsim.demandmodeling.router.util.PreProcessEuclidean.<br />
 *
 * Network conditions:<br />
 * <ul>
 * <li>The same as for Dijkstra: The link cost must be non-negative,
 * otherwise Dijkstra does not work.</li>
 * <li>The length stored in the links must be greater or equal to the euclidean distance of the
 * link's start and end node, otherwise the algorithm is not guaranteed to deliver least-cost paths.
 * In this case PreProcessEuclidean gives out a warning message.</li>
 * <li>The CostCalculator which calculates the cost for each link must implement the TravelMinCost
 * interface, i.e. it must implement the function getLinkMinimumTravelCost(Link link).
 * The TravelTimeCalculator class does implement it.</li></p>
 * <p><code> PreProcessEuclidean.run() </code>is very fast and needs (almost) no additional
 * memory.<code><br />
 * </code> Code Example:<code><br />
 * TravelMinCost costFunction = ...<br />
 * PreProcessEuclidean preProcessData = new PreProcessEuclidean(costFunction);<br />
 * preProcessData.run(network);<br />...<br />
 * LeastCostPathCalculator routingAlgo = new AStarEuclidean(network, preProcessData);<br />
 * ...</code></p>
 * <p>A note about the so-called overdo factor: You can drastically accelerate the routing of
 * AStarEuclidean by providing an overdo factor &gt; 1 (e.g. 1.5, 2 or 3). In this case,
 * AStarEuclidean does not calculate least-cost paths anymore but tends to deliver distance-minimal
 * paths. The greater the overdo factor, the faster the algorithm but the more the calculated routes
 * diverge from the least-cost ones.<br />
 * A typical invocation then looks like this:<br>
 * <code>LeastCostPathCalculator routingAlgo = new AStarEuclidean(network, preProcessData, 2);</code>
 * <br />
 * @see org.matsim.core.router.util.PreProcessEuclidean
 * @see org.matsim.core.router.Dijkstra
 * @see org.matsim.core.router.AStarLandmarks
 * @author lnicolas
 */
public class DijkstraRecord extends Dijkstra {
	private static final Logger log = Logger.getLogger( DijkstraRecord.class ) ;

    private int visitedNodes = 0;

	DijkstraRecord(final Network network, final TravelDisutility costFunction, final TravelTime timeFunction) {
		super(network, costFunction, timeFunction);
    }

	DijkstraRecord(final Network network, final TravelDisutility costFunction, final TravelTime timeFunction,
			final PreProcessDijkstra preProcessData) {
		super(network, costFunction, timeFunction, preProcessData);
    }


	/**
	 * Inserts the given Node n into the pendingNodes queue and updates its time and cost information.
	 * 
	 * @param n The Node that is revisited.
	 * @param data The data for node.
	 * @param pendingNodes The nodes visited and not processed yet.
	 * @param time The time of the visit of n.
	 * @param cost The accumulated cost at the time of the visit of n.
	 * @param outLink The link from which we came visiting n.
	 */
    @Override
	protected void visitNode(final Node n, final DijkstraNodeData data, final RouterPriorityQueue<Node> pendingNodes,
			final double time, final double cost, final Link outLink) {
        super.visitNode(n, data, pendingNodes, time, cost, outLink);
        visitedNodes += 1;
	}
	
    @Override
    public Path calcLeastCostPath(final Node fromNode, final Node toNode, final double startTime, final Person person2, final Vehicle vehicle2) {
        Path finalPath = super.calcLeastCostPath(fromNode, toNode, startTime, person2, vehicle2);
        log.info(String.format("AAA ROUTE LENGTH LINKS: %d", finalPath.links.size()));
        log.info(String.format("AAA ROUTE LENGTH NODES: %d", finalPath.nodes.size()));
        log.info(String.format("AAA ROUTE TRAVEL TIME: %f", finalPath.travelTime));
        log.info(String.format("AAA ROUTE TRAVEL COST: %f", finalPath.travelCost));
        log.info(String.format("AAA VISITED NODES: %d", visitedNodes));
        return finalPath;
    }

}
