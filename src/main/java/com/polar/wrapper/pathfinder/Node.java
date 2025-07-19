package com.polar.wrapper.pathfinder; // Make sure this matches your package path

import net.minecraft.util.BlockPos;
import java.util.Objects;

/**
 * Represents a node in the pathfinding grid.
 * Implements Comparable to be used in a PriorityQueue.
 */
public class Node implements Comparable<Node> {
    public final BlockPos pos;
    public final Node parent;
    public double gCost; // Cost from start to this node
    public double hCost; // Heuristic cost from this node to goal
    public double fCost; // Total cost (gCost + hCost)
    public double movementCost; // The specific cost of the single movement that led to this node

    public Node(BlockPos pos, Node parent, double gCost, double hCost) {
        this.pos = pos;
        this.parent = parent;
        this.gCost = gCost;
        this.hCost = hCost;
        this.fCost = gCost + hCost;
    }

    @Override
    public int compareTo(Node other) {
        int fCostComparison = Double.compare(this.fCost, other.fCost);
        if (fCostComparison == 0) {
            return Double.compare(this.hCost, other.hCost);
        }
        return fCostComparison;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Node node = (Node) o;
        return Objects.equals(pos, node.pos);
    }

    @Override
    public int hashCode() {
        return Objects.hash(pos);
    }
}
