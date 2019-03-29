package org.matsim.run;

import java.util.Arrays;
import java.util.Collections;

import org.junit.Assert;
import org.junit.Test;
import org.matsim.api.core.v01.Coord;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.Scenario;
import org.matsim.api.core.v01.events.PersonArrivalEvent;
import org.matsim.api.core.v01.events.PersonDepartureEvent;
import org.matsim.api.core.v01.events.handler.PersonArrivalEventHandler;
import org.matsim.api.core.v01.events.handler.PersonDepartureEventHandler;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.api.core.v01.network.Node;
import org.matsim.api.core.v01.population.Activity;
import org.matsim.api.core.v01.population.Leg;
import org.matsim.api.core.v01.population.Person;
import org.matsim.api.core.v01.population.Plan;
import org.matsim.api.core.v01.population.Population;
import org.matsim.core.api.experimental.events.EventsManager;
import org.matsim.core.config.Config;
import org.matsim.core.config.ConfigUtils;
import org.matsim.core.config.groups.PlanCalcScoreConfigGroup.ActivityParams;
import org.matsim.core.controler.AbstractModule;
import org.matsim.core.controler.Controler;
import org.matsim.core.controler.OutputDirectoryHierarchy.OverwriteFileSetting;
import org.matsim.core.events.EventsUtils;
import org.matsim.core.router.LinkWrapperFacility;
import org.matsim.core.router.NetworkRoutingModule;
import org.matsim.core.router.costcalculators.OnlyTimeDependentTravelDisutility;
import org.matsim.core.router.util.LeastCostPathCalculator;
import org.matsim.core.router.DijkstraRecordFactory;
import org.matsim.core.router.AStarEuclideanRecordFactory;
import org.matsim.core.router.AStarLandmarksRecordFactory;
import org.matsim.core.router.util.TravelDisutility;
import org.matsim.core.router.util.TravelTime;
import org.matsim.core.scenario.ScenarioUtils;
import org.matsim.core.trafficmonitoring.FreeSpeedTravelTime;
import org.matsim.vehicles.Vehicle;
import org.matsim.vehicles.VehicleUtils;

public class RouterSingleTest{
		@Test
		/*
         * Run a single agent with a single plan on Dijkstra
		 */
		public void testDijkstraOnRuntimeCreatedNetwork() {
			Scenario scenario = ScenarioUtils.createScenario(ConfigUtils.createConfig());

			Network network = scenario.getNetwork();

			Node node1 = network.getFactory().createNode(Id.createNodeId("N1"), new Coord(0.0, 0.0));
			Node node2 = network.getFactory().createNode(Id.createNodeId("N2"), new Coord(0.0, 0.0));
			Node node3 = network.getFactory().createNode(Id.createNodeId("N3"), new Coord(0.0, 0.0));
			Node node4 = network.getFactory().createNode(Id.createNodeId("N4"), new Coord(0.0, 0.0));
			Node node5 = network.getFactory().createNode(Id.createNodeId("N5"), new Coord(0.0, 0.0));
			Node node6 = network.getFactory().createNode(Id.createNodeId("N6"), new Coord(0.0, 0.0));

			Link link12 = network.getFactory().createLink(Id.createLinkId("L12"), node1, node2);
			Link link23 = network.getFactory().createLink(Id.createLinkId("L23"), node2, node3);
			Link link34 = network.getFactory().createLink(Id.createLinkId("L34"), node3, node4);
			Link link45 = network.getFactory().createLink(Id.createLinkId("L45"), node4, node5);
			Link link56 = network.getFactory().createLink(Id.createLinkId("L56"), node5, node6);

			network.addNode(node1);
			network.addNode(node2);
			network.addNode(node3);
			network.addNode(node4);
			network.addNode(node5);
			network.addNode(node6);

			network.addLink(link12);
			network.addLink(link23);
			network.addLink(link34);
			network.addLink(link45);
			network.addLink(link56);

			Arrays.asList(link12, link23, link34, link45, link56).forEach(l -> l.setAllowedModes(Collections.singleton("car")));
			Arrays.asList(link12, link23, link34, link45, link56).forEach(l -> l.setLength(1000.0));
			Arrays.asList(link12, link23, link34, link45, link56).forEach(l -> l.setFreespeed(10.0));

            Person person = addPersonWithPlanToPopulation(scenario);

			TravelTime travelTime = new FreeSpeedTravelTime();
			TravelDisutility travelDisutility = new OnlyTimeDependentTravelDisutility(travelTime);

			//LeastCostPathCalculator router = new DijkstraRecordFactory().createPathCalculator(network, travelDisutility,
			//LeastCostPathCalculator router = new AStarEuclideanRecordFactory().createPathCalculator(network, travelDisutility,
			LeastCostPathCalculator router = new AStarLandmarksRecordFactory().createPathCalculator(network, travelDisutility,
					travelTime);
			NetworkRoutingModule routingModule = new NetworkRoutingModule("car", scenario.getPopulation().getFactory(), network,
					router);

            System.out.println("AAA");
            System.out.println("AAA--------------------------");
            System.out.println("AAA Constructed Network AStarLandmarks");
            System.out.println("AAA--------------------------");

			Leg leg = (Leg) routingModule
					.calcRoute(new LinkWrapperFacility(link12), new LinkWrapperFacility(link45), 0.0, person).get(0);

        }
		@Test
		/*
         * Run a single agent with a single plan on Dijkstra
		 */
		public void testOnSimpleNetwork() {
            String configFile = "~/rmitcs/ees/matsim-example-project/scenarios/equil/config.xml";
			Scenario scenario = ScenarioUtils.loadScenario(ConfigUtils.loadConfig(configFile));
            
			Network network = scenario.getNetwork();

            // matsim-example-network
			Node node2 = network.getFactory().createNode(Id.createNodeId("2"), new Coord(0.0, 0.0));
			Node node3 = network.getFactory().createNode(Id.createNodeId("3"), new Coord(0.0, 0.0));
			Node node9 = network.getFactory().createNode(Id.createNodeId("9"), new Coord(0.0, 0.0));
			Node node12 = network.getFactory().createNode(Id.createNodeId("12"), new Coord(0.0, 0.0));
			Link linkfrom = network.getFactory().createLink(Id.createLinkId("2"), node2, node3);
			Link linkto = network.getFactory().createLink(Id.createLinkId("17"), node9, node12);

            Person person = addPersonWithPlanToPopulation(scenario);

			TravelTime travelTime = new FreeSpeedTravelTime();
			TravelDisutility travelDisutility = new OnlyTimeDependentTravelDisutility(travelTime);

			//LeastCostPathCalculator router = new DijkstraTestFactory().createPathCalculator(network, travelDisutility,
			//LeastCostPathCalculator router = new AStarEuclideanRecordFactory().createPathCalculator(network, travelDisutility,
			LeastCostPathCalculator router = new AStarLandmarksRecordFactory().createPathCalculator(network, travelDisutility,
					travelTime);
			NetworkRoutingModule routingModule = new NetworkRoutingModule("car", scenario.getPopulation().getFactory(), network,
					router);

            System.out.println("AAA");
            System.out.println("AAA--------------------------");
            System.out.println("AAA Simple Network AStarLandmarks");
            System.out.println("AAA--------------------------");

			Leg leg = (Leg) routingModule
					.calcRoute(new LinkWrapperFacility(linkfrom), new LinkWrapperFacility(linkto), 0.0, person).get(0);

			//Assert.assertEquals(adjustedRoutingTravelTime, 202.0, 1e-3);
		}
		@Test
		/*
         * Run a single agent with a single plan on Dijkstra
		 */
		public void testDijkstraOnFreeSpeedAnglesea() {
            String configFile = "~/rmitcs/ees/ees/scenarios/surf-coast-shire/anglesea-12k/scenario_matsim_main.xml";
			Scenario scenario = ScenarioUtils.loadScenario(ConfigUtils.loadConfig(configFile));
            
			Network network = scenario.getNetwork();

			Node node2 = network.getFactory().createNode(Id.createNodeId("1763048114"), new Coord(0.0, 0.0));
			Node node3 = network.getFactory().createNode(Id.createNodeId("288435917"), new Coord(0.0, 0.0));
			Node node9 = network.getFactory().createNode(Id.createNodeId("2315437915"), new Coord(0.0, 0.0));
			Node node12 = network.getFactory().createNode(Id.createNodeId("3545568272"), new Coord(0.0, 0.0));

            String linkFromId = "10040-10041-9998-9999-10000-10001-10002";
            String linkToId = "10121";

			Link linkfrom = network.getFactory().createLink(Id.createLinkId("10040-10041-9998-9999-10000-10001-10002"), node2, node3);
			Link linkto = network.getFactory().createLink(Id.createLinkId("10121"), node9, node12);

            Person person = addPersonWithPlanToPopulation(scenario);

			TravelTime travelTime = new FreeSpeedTravelTime();
			TravelDisutility travelDisutility = new OnlyTimeDependentTravelDisutility(travelTime);

			LeastCostPathCalculator router = new DijkstraRecordFactory().createPathCalculator(network, travelDisutility,
					travelTime);
			NetworkRoutingModule routingModule = new NetworkRoutingModule("car", scenario.getPopulation().getFactory(), network,
					router);

            System.out.println("AAA");
            System.out.println("AAA--------------------------");
            System.out.println("AAA Anglesea Freespeed Dijkstra");
            System.out.println("AAA--------------------------");

			Leg leg = (Leg) routingModule
					.calcRoute(new LinkWrapperFacility(linkfrom), new LinkWrapperFacility(linkto), 0.0, person).get(0);

		}

		@Test
		/*
         * Run a single agent with a single plan on AStarEuclidean
		 */
		public void testAStarEuclideanOnFreeSpeedAnglesea() {
            String configFile = "~/rmitcs/ees/ees/scenarios/surf-coast-shire/anglesea-12k/scenario_matsim_main.xml";
			Scenario scenario = ScenarioUtils.loadScenario(ConfigUtils.loadConfig(configFile));
            
			Network network = scenario.getNetwork();

			Node node2 = network.getFactory().createNode(Id.createNodeId("1763048114"), new Coord(0.0, 0.0));
			Node node3 = network.getFactory().createNode(Id.createNodeId("288435917"), new Coord(0.0, 0.0));
			Node node9 = network.getFactory().createNode(Id.createNodeId("2315437915"), new Coord(0.0, 0.0));
			Node node12 = network.getFactory().createNode(Id.createNodeId("3545568272"), new Coord(0.0, 0.0));

            String linkFromId = "10040-10041-9998-9999-10000-10001-10002";
            String linkToId = "10121";

			Link linkfrom = network.getFactory().createLink(Id.createLinkId("10040-10041-9998-9999-10000-10001-10002"), node2, node3);
			Link linkto = network.getFactory().createLink(Id.createLinkId("10121"), node9, node12);

            Person person = addPersonWithPlanToPopulation(scenario);

			TravelTime travelTime = new FreeSpeedTravelTime();
			TravelDisutility travelDisutility = new OnlyTimeDependentTravelDisutility(travelTime);

			LeastCostPathCalculator router = new AStarEuclideanRecordFactory().createPathCalculator(network, travelDisutility,
					travelTime);
			NetworkRoutingModule routingModule = new NetworkRoutingModule("car", scenario.getPopulation().getFactory(), network,
					router);

            System.out.println("AAA");
            System.out.println("AAA--------------------------");
            System.out.println("AAA Anglesea Freespeed AStarEuclidean");
            System.out.println("AAA--------------------------");
			Leg leg = (Leg) routingModule
					.calcRoute(new LinkWrapperFacility(linkfrom), new LinkWrapperFacility(linkto), 0.0, person).get(0);

		}

		@Test
		/*
         * Run a single agent with a single plan on Dijkstra
		 */
		public void testAStarLandmarksOnFreeSpeedAnglesea() {
            String configFile = "~/rmitcs/ees/ees/scenarios/surf-coast-shire/anglesea-12k/scenario_matsim_main.xml";
			Scenario scenario = ScenarioUtils.loadScenario(ConfigUtils.loadConfig(configFile));
            
			Network network = scenario.getNetwork();

			Node node2 = network.getFactory().createNode(Id.createNodeId("1763048114"), new Coord(0.0, 0.0));
			Node node3 = network.getFactory().createNode(Id.createNodeId("288435917"), new Coord(0.0, 0.0));
			Node node9 = network.getFactory().createNode(Id.createNodeId("2315437915"), new Coord(0.0, 0.0));
			Node node12 = network.getFactory().createNode(Id.createNodeId("3545568272"), new Coord(0.0, 0.0));

            String linkFromId = "10040-10041-9998-9999-10000-10001-10002";
            String linkToId = "10121";

			Link linkfrom = network.getFactory().createLink(Id.createLinkId("10040-10041-9998-9999-10000-10001-10002"), node2, node3);
			Link linkto = network.getFactory().createLink(Id.createLinkId("10121"), node9, node12);

            Person person = addPersonWithPlanToPopulation(scenario);

			TravelTime travelTime = new FreeSpeedTravelTime();
			TravelDisutility travelDisutility = new OnlyTimeDependentTravelDisutility(travelTime);

			LeastCostPathCalculator router = new AStarLandmarksRecordFactory().createPathCalculator(network, travelDisutility,
					travelTime);
			NetworkRoutingModule routingModule = new NetworkRoutingModule("car", scenario.getPopulation().getFactory(), network,
					router);

            System.out.println("AAA");
            System.out.println("AAA--------------------------");
            System.out.println("AAA Anglesea Freespeed AStarLandmarks");
            System.out.println("AAA--------------------------");
			Leg leg = (Leg) routingModule
					.calcRoute(new LinkWrapperFacility(linkfrom), new LinkWrapperFacility(linkto), 0.0, person).get(0);

		}

        public Person addPersonWithPlanToPopulation(Scenario scenario) {
			Vehicle vehicle = scenario.getVehicles().getFactory().createVehicle(Id.createVehicleId("P"),
					VehicleUtils.getDefaultVehicleType());
			scenario.getVehicles().addVehicleType(VehicleUtils.getDefaultVehicleType());
			scenario.getVehicles().addVehicle(vehicle);

			Population population = scenario.getPopulation();

			Person person = population.getFactory().createPerson(Id.createPersonId("P"));
			population.addPerson(person);

			Plan plan = population.getFactory().createPlan();
			person.addPlan(plan);

            return person;
        }

    }
