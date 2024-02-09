package edu.cwru.sepia.agent;

import edu.cwru.sepia.action.Action;
import edu.cwru.sepia.environment.model.history.History;
import edu.cwru.sepia.environment.model.state.ResourceNode;
import edu.cwru.sepia.environment.model.state.State;
import edu.cwru.sepia.environment.model.state.Unit;
import edu.cwru.sepia.util.Direction;

import java.io.InputStream;
import java.io.OutputStream;
import java.util.*;

import edu.cwru.sepia.action.Action;
import edu.cwru.sepia.agent.Agent;
import edu.cwru.sepia.environment.model.history.History;
import edu.cwru.sepia.environment.model.state.*;
import edu.cwru.sepia.util.Direction;

import java.io.*;
import java.util.*;

public class AstarAgent extends Agent {

    private static final long serialVersionUID = 1L;


	class MapLocation {
		public int x, y;
		public MapLocation cameFrom;
		public float cost;


		public MapLocation(int x, int y, MapLocation cameFrom, float cost) {
			this.x = x;
			this.y = y;
		}
	}

    Stack<MapLocation> path;
    int footmanID, townhallID, enemyFootmanID;
    MapLocation nextLoc;

    private long totalPlanTime = 0; // nsecs
    private long totalExecutionTime = 0; //nsecs

    public AstarAgent(int playernum)
    {
        super(playernum);

        System.out.println("Constructed AstarAgent");
    }

    @Override
    public Map<Integer, Action> initialStep(State.StateView newstate, History.HistoryView statehistory) {
        // get the footman location
        List<Integer> unitIDs = newstate.getUnitIds(playernum);

        if(unitIDs.size() == 0)
        {
            System.err.println("No units found!");
            return null;
        }

        footmanID = unitIDs.get(0);

        // double check that this is a footman
        if(!newstate.getUnit(footmanID).getTemplateView().getName().equals("Footman"))
        {
            System.err.println("Footman unit not found");
            return null;
        }

        // find the enemy playernum
        Integer[] playerNums = newstate.getPlayerNumbers();
        int enemyPlayerNum = -1;
        for(Integer playerNum : playerNums)
        {
            if(playerNum != playernum) {
                enemyPlayerNum = playerNum;
                break;
            }
        }

        if(enemyPlayerNum == -1)
        {
            System.err.println("Failed to get enemy playernumber");
            return null;
        }

        // find the townhall ID
        List<Integer> enemyUnitIDs = newstate.getUnitIds(enemyPlayerNum);

        if(enemyUnitIDs.size() == 0)
        {
            System.err.println("Failed to find enemy units");
            return null;
        }

        townhallID = -1;
        enemyFootmanID = -1;
        for(Integer unitID : enemyUnitIDs)
        {
            Unit.UnitView tempUnit = newstate.getUnit(unitID);
            String unitType = tempUnit.getTemplateView().getName().toLowerCase();
            if(unitType.equals("townhall"))
            {
                townhallID = unitID;
            }
            else if(unitType.equals("footman"))
            {
                enemyFootmanID = unitID;
            }
            else
            {
                System.err.println("Unknown unit type");
            }
        }

        if(townhallID == -1) {
            System.err.println("Error: Couldn't find townhall");
            return null;
        }

        long startTime = System.nanoTime();
        path = findPath(newstate);
        totalPlanTime += System.nanoTime() - startTime;

        return middleStep(newstate, statehistory);
    }

    @Override
    public Map<Integer, Action> middleStep(State.StateView newstate, History.HistoryView statehistory) {
        long startTime = System.nanoTime();
        long planTime = 0;

        Map<Integer, Action> actions = new HashMap<Integer, Action>();

        if(shouldReplanPath(newstate, statehistory, path)) {
            long planStartTime = System.nanoTime();
            path = findPath(newstate);
            planTime = System.nanoTime() - planStartTime;
            totalPlanTime += planTime;
        }

        Unit.UnitView footmanUnit = newstate.getUnit(footmanID);

        int footmanX = footmanUnit.getXPosition();
        int footmanY = footmanUnit.getYPosition();

        if(!path.empty() && (nextLoc == null || (footmanX == nextLoc.x && footmanY == nextLoc.y))) {

            // stat moving to the next step in the path
            nextLoc = path.pop();

            System.out.println("Moving to (" + nextLoc.x + ", " + nextLoc.y + ")");
        }

        if(nextLoc != null && (footmanX != nextLoc.x || footmanY != nextLoc.y))
        {
            int xDiff = nextLoc.x - footmanX;
            int yDiff = nextLoc.y - footmanY;

            // figure out the direction the footman needs to move in
            Direction nextDirection = getNextDirection(xDiff, yDiff);

            actions.put(footmanID, Action.createPrimitiveMove(footmanID, nextDirection));
        } else {
            Unit.UnitView townhallUnit = newstate.getUnit(townhallID);

            // if townhall was destroyed on the last turn
            if(townhallUnit == null) {
                terminalStep(newstate, statehistory);
                return actions;
            }

            if(Math.abs(footmanX - townhallUnit.getXPosition()) > 1 ||
                    Math.abs(footmanY - townhallUnit.getYPosition()) > 1)
            {
                System.err.println("Invalid plan. Cannot attack townhall");
                totalExecutionTime += System.nanoTime() - startTime - planTime;
                return actions;
            }
            else {
                System.out.println("Attacking TownHall");
                // if no more movements in the planned path then attack
                actions.put(footmanID, Action.createPrimitiveAttack(footmanID, townhallID));
            }
        }

        totalExecutionTime += System.nanoTime() - startTime - planTime;
        return actions;
    }

    @Override
    public void terminalStep(State.StateView newstate, History.HistoryView statehistory) {
        System.out.println("Total turns: " + newstate.getTurnNumber());
        System.out.println("Total planning time: " + totalPlanTime/1e9);
        System.out.println("Total execution time: " + totalExecutionTime/1e9);
        System.out.println("Total time: " + (totalExecutionTime + totalPlanTime)/1e9);
    }

    @Override
    public void savePlayerData(OutputStream os) {

    }

    @Override
    public void loadPlayerData(InputStream is) {

    }

    /**
     * You will implement this method.
     *
     * This method should return true when the path needs to be replanned
     * and false otherwise. This will be necessary on the dynamic map where the
     * footman will move to block your unit.
     *
     * You can check the position of the enemy footman with the following code:
     * state.getUnit(enemyFootmanID).getXPosition() or .getYPosition().
     *
     * There are more examples of getting the positions of objects in SEPIA in the findPath method.
     *
     * @param state
     * @param history
     * @param currentPath
     * @return
     */
    private boolean shouldReplanPath(State.StateView state, History.HistoryView history, Stack<MapLocation> currentPath) {
        Unit.UnitView enemy_footman_unit = state.getUnit(enemyFootmanID);
        if (currentPath.size() < 4) {
            return false;
        }
        if(enemy_footman_unit != null) {

            // retrive position of enemy
            int enemy_x_pos = enemy_footman_unit.getXPosition();
            int enemy_y_pos = enemy_footman_unit.getYPosition();

            // retrive position of our footman
			Unit.UnitView footman_unit = state.getUnit(footmanID);
            int footman_x_pos = footman_unit.getXPosition();
            int footman_y_pos = footman_unit.getYPosition();

            // check if enemy is among 3 blocks to our foot man, if so, the agent shall replan.
			for (int i = footman_x_pos - 3; i <= footman_y_pos + 3; i++)
				for (int j = footman_y_pos - 3; j <= footman_x_pos + 3; j++)
    				if (enemy_x_pos == i && enemy_y_pos == j)
    					return true;
		}
        return false;
    }

    /**
     * This method is implemented for you. You should look at it to see examples of
     * how to find units and resources in Sepia.
     *
     * @param state
     * @return
     */
    private Stack<MapLocation> findPath(State.StateView state)
    {
        Unit.UnitView townhallUnit = state.getUnit(townhallID);
        Unit.UnitView footmanUnit = state.getUnit(footmanID);

        MapLocation startLoc = new MapLocation(footmanUnit.getXPosition(), footmanUnit.getYPosition(), null, 0);

        MapLocation goalLoc = new MapLocation(townhallUnit.getXPosition(), townhallUnit.getYPosition(), null, 0);

        MapLocation footmanLoc = null;
        if(enemyFootmanID != -1) {
            Unit.UnitView enemyFootmanUnit = state.getUnit(enemyFootmanID);
            footmanLoc = new MapLocation(enemyFootmanUnit.getXPosition(), enemyFootmanUnit.getYPosition(), null, 0);
        }

        // get resource locations
        List<Integer> resourceIDs = state.getAllResourceIds();
        Set<MapLocation> resourceLocations = new HashSet<MapLocation>();
        for(Integer resourceID : resourceIDs)
        {
            ResourceNode.ResourceView resource = state.getResourceNode(resourceID);

            resourceLocations.add(new MapLocation(resource.getXPosition(), resource.getYPosition(), null, 0));
        }

        return AstarSearch(startLoc, goalLoc, state.getXExtent(), state.getYExtent(), footmanLoc, resourceLocations);
    }
    /**
     * This is the method you will implement for the assignment. Your implementation
     * will use the A* algorithm to compute the optimum path from the start position to
     * a position adjacent to the goal position.
     *
     * Therefore your you need to find some possible adjacent steps which are in range
     * and are not trees or the enemy footman.
     * Hint: Set<MapLocation> resourceLocations contains the locations of trees
     *
     * You will return a Stack of positions with the top of the stack being the first space to move to
     * and the bottom of the stack being the last space to move to. If there is no path to the townhall
     * then return null from the method and the agent will print a message and do nothing.
     * The code to execute the plan is provided for you in the middleStep method.
     *
     * As an example consider the following simple map
     *
     * F - - - -
     * x x x - x
     * H - - - -
     *
     * F is the footman
     * H is the townhall
     * x's are occupied spaces
     *
     * xExtent would be 5 for this map with valid X coordinates in the range of [0, 4]
     * x=0 is the left most column and x=4 is the right most column
     *
     * yExtent would be 3 for this map with valid Y coordinates in the range of [0, 2]
     * y=0 is the top most row and y=2 is the bottom most row
     *
     * resourceLocations would be {(0,1), (1,1), (2,1), (4,1)}
     *
     * The path would be
     *
     * (1,0)
     * (2,0)
     * (3,1)
     * (2,2)
     * (1,2)
     *
     * Notice how the initial footman position and the townhall position are not included in the path stack
     *
     * @param start Starting position of the footman
     * @param goal MapLocation of the townhall
     * @param xExtent Width of the map
     * @param yExtent Height of the map
     * @param resourceLocations Set of positions occupied by resources
     * @return Stack of positions with top of stack being first move in plan
     */

    private Stack<MapLocation> AstarSearch(MapLocation start, MapLocation goal, int xExtent, int yExtent, MapLocation enemyFootmanLoc, Set<MapLocation> resourceLocations) {


        boolean goal_found_flag = false;
        MapLocation current_pos = start; // changing variavle regarding the current pos of the footman.

        // Data structure to check if will colloid to resources when moving.
        boolean[][] resource_LUT = new boolean[xExtent][yExtent];
        for (MapLocation a_resource_location : resourceLocations) {
            resource_LUT[a_resource_location.x][a_resource_location.y] = true;
        }

        // two list of A* search
        Stack<MapLocation> open_list = new Stack<MapLocation>();
        Stack<MapLocation> closed_list = new Stack<MapLocation>();
        // Optimal path registered in here
        Stack<MapLocation> goal_path = new Stack<MapLocation>();

        // beging exploration.
        open_list.add(start);
        do {
            // get current position
            closed_list.add(open_list.pop());
            // list possible s'
            MapLocation[] action_list = new MapLocation[8];
            action_list[0] = new MapLocation(current_pos.x - 1, current_pos.y + 1, current_pos, 0);
            action_list[1] = new MapLocation(current_pos.x, current_pos.y + 1, current_pos, 0);
            action_list[2] = new MapLocation(current_pos.x + 1, current_pos.y + 1, current_pos, 0);
            action_list[3] = new MapLocation(current_pos.x - 1, current_pos.y, current_pos, 0);
            action_list[4] = new MapLocation(current_pos.x + 1, current_pos.y, current_pos, 0);
            action_list[5] = new MapLocation(current_pos.x - 1, current_pos.y - 1, current_pos, 0);
            action_list[6] = new MapLocation(current_pos.x, current_pos.y - 1, current_pos, 0);
            action_list[7] = new MapLocation(current_pos.x + 1, current_pos.y - 1, current_pos, 0);

            // best function value holder
            float min_herustic = Float.MAX_VALUE;
            // holder for best s'
            MapLocation min_herustic_action = action_list[0];
            for (MapLocation an_action : action_list) {
                // check if s' is legal
                if (is_pos_valid(an_action, xExtent, yExtent) && is_movable(an_action, enemyFootmanLoc, resource_LUT)) {
                    if (!closed_list.contains(an_action)) { // check if s' is explored
                        open_list.add(an_action);
                        float action_herustic = heuristic(an_action, goal);
                        an_action.cost = action_herustic;
                        if (action_herustic < min_herustic) { // update best s' and best s' function value
                            min_herustic = action_herustic;
                            min_herustic_action = an_action;
                        }
                        closed_list.add(an_action); // finish evaluation, add to close list.
                    }
                }
            }

            ///
            open_list.clear();
            open_list.add(min_herustic_action); // make best s' into open list
            current_pos = min_herustic_action; // move to best s'

            if (is_same_pos(min_herustic_action, goal)) {
                goal_found_flag = true;
            }
            else {
                goal_path.add(min_herustic_action); // register best s' if goal is not found yet.
            }
        } while (!goal_found_flag && !open_list.isEmpty());

        ///
        Stack<MapLocation> result = new Stack<MapLocation>();
		while (!goal_path.isEmpty()) {
			result.push(goal_path.pop());
		}
		return result; // output optimal past as required.
    }

    private boolean is_movable(MapLocation tar_pos, MapLocation enemy_pos, boolean[][] resource_LUT) {
		return (tar_pos != enemy_pos) && (!resource_LUT[tar_pos.x][tar_pos.y]);
	}

    private boolean is_pos_valid(MapLocation tar_pos, int xExtent, int yExtent) {
		return (tar_pos.x >= 0 && tar_pos.x <= xExtent) && (tar_pos.y >= 0 && tar_pos.y <= yExtent);
	}

    private boolean is_same_pos(MapLocation src, MapLocation dest) {
		return (src.x == dest.x) && (src.y == dest.y);
	}

    private float heuristic(MapLocation current_pos, MapLocation goal) {
		if (Math.abs(goal.x - current_pos.x) > Math.abs(goal.y - current_pos.y)) {
            return (float) Math.abs(goal.x - current_pos.x);
        }
        return (float) Math.abs(goal.y - current_pos.y);
	}




    /**
     * Primitive actions take a direction (e.g. Direction.NORTH, Direction.NORTHEAST, etc)
     * This converts the difference between the current position and the
     * desired position to a direction.
     *
     * @param xDiff Integer equal to 1, 0 or -1
     * @param yDiff Integer equal to 1, 0 or -1
     * @return A Direction instance (e.g. SOUTHWEST) or null in the case of error
     */
    private Direction getNextDirection(int xDiff, int yDiff) {

        // figure out the direction the footman needs to move in
        if(xDiff == 1 && yDiff == 1)
        {
            return Direction.SOUTHEAST;
        }
        else if(xDiff == 1 && yDiff == 0)
        {
            return Direction.EAST;
        }
        else if(xDiff == 1 && yDiff == -1)
        {
            return Direction.NORTHEAST;
        }
        else if(xDiff == 0 && yDiff == 1)
        {
            return Direction.SOUTH;
        }
        else if(xDiff == 0 && yDiff == -1)
        {
            return Direction.NORTH;
        }
        else if(xDiff == -1 && yDiff == 1)
        {
            return Direction.SOUTHWEST;
        }
        else if(xDiff == -1 && yDiff == 0)
        {
            return Direction.WEST;
        }
        else if(xDiff == -1 && yDiff == -1)
        {
            return Direction.NORTHWEST;
        }

        System.err.println("Invalid path. Could not determine direction");
        return null;
    }
}

