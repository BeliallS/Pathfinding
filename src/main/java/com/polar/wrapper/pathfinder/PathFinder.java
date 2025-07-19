package com.polar.wrapper.pathfinder; // Make sure this matches your package path

import com.polar.wrapper.PolarClient;
import net.minecraft.block.*;
import net.minecraft.init.Blocks;
import net.minecraft.util.BlockPos;
import net.minecraft.util.EnumFacing;
import net.minecraft.util.Vec3;

import java.util.*;

/**
 * A high-performance, walking-focused pathfinder for Minecraft 1.8.9.
        * This version is tuned to prioritize walking and stepping down, ignoring jumping.
 */
public class PathFinder {
    private final Node startNode;
    private final Node goalNode;
    private final PriorityQueue<Node> openSet = new PriorityQueue<>();
    private final Map<BlockPos, Node> closedSet = new HashMap<>();
    private final Map<BlockPos, Double> heuristicCache = new HashMap<>();

    private static final BlockPos[] DIRECTIONS = {
            new BlockPos(1, 0, 0), new BlockPos(-1, 0, 0), new BlockPos(0, 0, 1), new BlockPos(0, 0, -1),
            new BlockPos(1, 0, 1), new BlockPos(1, 0, -1), new BlockPos(-1, 0, 1), new BlockPos(-1, 0, -1)
    };

    private static final double STRAIGHT_COST = 1.0;
    private static final double DIAGONAL_COST = Math.sqrt(2);

    public PathFinder(BlockPos start, BlockPos goal) {
        this.startNode = new Node(start, null, 0.0, getHeuristic(start, goal));
        this.goalNode = new Node(goal, null, Double.MAX_VALUE, 0.0);
    }

    public List<Node> calculatePath() {
        openSet.add(startNode);
        closedSet.put(startNode.pos, startNode);

        while (!openSet.isEmpty()) {
            Node current = openSet.poll();

            if (current.pos.equals(goalNode.pos)) {
                return smoothPath(reconstructPath(current));
            }

            for (Node neighbor : getNeighbours(current)) {
                if (closedSet.containsKey(neighbor.pos)) continue;

                double directionChangePenalty = 0.0;
                if (current.parent != null) {
                    BlockPos parentToCurrent = current.pos.subtract(current.parent.pos);
                    BlockPos currentToNeighbor = neighbor.pos.subtract(current.pos);
                    if (parentToCurrent.getX() != currentToNeighbor.getX() || parentToCurrent.getZ() != currentToNeighbor.getZ()) {
                        directionChangePenalty = 0.01;
                    }
                }

                double newGCost = current.gCost + neighbor.movementCost + getEnvironmentPenalty(neighbor.pos) + directionChangePenalty;

                Node newNode = new Node(neighbor.pos, current, newGCost, getHeuristic(neighbor.pos, goalNode.pos));
                newNode.movementCost = neighbor.movementCost;

                openSet.add(newNode);
                closedSet.put(neighbor.pos, newNode);
            }
        }
        return null; // No path found
    }

    private List<Node> getNeighbours(Node current) {
        List<Node> neighbours = new ArrayList<>();
        for (BlockPos dir : DIRECTIONS) {
            BlockPos horizontalNeighborPos = current.pos.add(dir.getX(), 0, dir.getZ());
            boolean isDiagonal = (dir.getX() != 0 && dir.getZ() != 0);
            double moveCost = isDiagonal ? DIAGONAL_COST : STRAIGHT_COST;

            if (isWalkable(horizontalNeighborPos)) {
                Node neighbor = new Node(horizontalNeighborPos, null, 0, 0);
                neighbor.movementCost = moveCost;
                neighbours.add(neighbor);
                continue;
            }

            BlockPos downPos = horizontalNeighborPos.down();
            if (isWalkable(downPos)) {
                Node neighbor = new Node(downPos, null, 0, 0);
                neighbor.movementCost = moveCost + 1.2; // Extra cost for falling
                neighbours.add(neighbor);
            }
        }
        return neighbours;
    }

    private boolean isWalkable(BlockPos pos) {
        Block groundBlock = PolarClient.mc.theWorld.getBlockState(pos.down()).getBlock();
        boolean isGroundSolid = groundBlock.isFullCube() || groundBlock instanceof BlockSlab || groundBlock instanceof BlockStairs;

        // FIXED: Check against Blocks.lava and Blocks.flowing_lava instead of BlockLava class
        if (!isGroundSolid || groundBlock == Blocks.lava || groundBlock == Blocks.flowing_lava || groundBlock instanceof BlockCactus) {
            return false;
        }

        Block bodyBlock = PolarClient.mc.theWorld.getBlockState(pos).getBlock();
        Block headBlock = PolarClient.mc.theWorld.getBlockState(pos.up()).getBlock();
        return !bodyBlock.getMaterial().isSolid() && !headBlock.getMaterial().isSolid();
    }

    private double getEnvironmentPenalty(BlockPos pos) {
        double penalty = 0.0;
        for (int x = -1; x <= 1; x++) {
            for (int z = -1; z <= 1; z++) {
                if (x == 0 && z == 0) continue;
                Block block = PolarClient.mc.theWorld.getBlockState(pos.add(x, 0, z)).getBlock();
                // FIXED: Check against Blocks.lava and Blocks.flowing_lava instead of BlockLava class
                if (block == Blocks.lava || block == Blocks.flowing_lava || block instanceof BlockCactus || block == Blocks.fire) {
                    penalty += 25.0;
                }
            }
        }
        if (isNearLedge(pos)) penalty += 1.5;
        return penalty;
    }

    private boolean isNearLedge(BlockPos pos) {
        for (EnumFacing facing : EnumFacing.Plane.HORIZONTAL) {
            if (PolarClient.mc.theWorld.isAirBlock(pos.offset(facing).down())) {
                return true;
            }
        }
        return false;
    }

    private double getHeuristic(BlockPos from, BlockPos to) {
        return heuristicCache.computeIfAbsent(from, k -> {
            int dx = Math.abs(from.getX() - to.getX());
            int dz = Math.abs(from.getZ() - to.getZ());
            double dy = Math.abs(from.getY() - to.getY());
            return (STRAIGHT_COST * (dx + dz) + (DIAGONAL_COST - 2 * STRAIGHT_COST) * Math.min(dx, dz)) + dy;
        });
    }

    private List<Node> reconstructPath(Node goalNode) {
        List<Node> path = new ArrayList<>();
        Node current = goalNode;
        while (current != null) {
            path.add(current);
            current = current.parent;
        }
        Collections.reverse(path);
        return path;
    }

    private List<Node> smoothPath(List<Node> path) {
        if (path == null || path.size() < 3) return path;
        List<Node> smoothedPath = new ArrayList<>();
        smoothedPath.add(path.get(0));
        int currentIdx = 0;
        while (currentIdx < path.size() - 1) {
            int lastVisibleIdx = currentIdx + 1;
            for (int nextIdx = currentIdx + 2; nextIdx < path.size(); nextIdx++) {
                if (hasLineOfSight(path.get(currentIdx).pos, path.get(nextIdx).pos)) {
                    lastVisibleIdx = nextIdx;
                } else {
                    break;
                }
            }
            smoothedPath.add(path.get(lastVisibleIdx));
            currentIdx = lastVisibleIdx;
        }
        return smoothedPath;
    }

    private boolean hasLineOfSight(BlockPos start, BlockPos end) {
        Vec3 startVec = new Vec3(start.getX() + 0.5, start.getY() + 0.5, start.getZ() + 0.5);
        Vec3 endVec = new Vec3(end.getX() + 0.5, end.getY() + 0.5, end.getZ() + 0.5);
        int steps = (int) Math.ceil(startVec.distanceTo(endVec) * 2);
        if (steps == 0) return true;

        Vec3 direction = endVec.subtract(startVec);

        for (int i = 1; i < steps; i++) {
            // FIXED: Manually performing the vector math to avoid any naming issues.
            double scale = (double) i / steps;
            Vec3 scaledDirection = new Vec3(direction.xCoord * scale, direction.yCoord * scale, direction.zCoord * scale);
            Vec3 intermediate = startVec.add(scaledDirection);

            if (!isWalkable(new BlockPos(intermediate))) {
                return false;
            }
        }
        return true;
    }
}
