package frc.robot.util.state.graph;

import java.util.*;

import frc.robot.util.state.transitions.TransitionBase;

public class DirectionalEnumGraph<V extends Enum<V>, T extends TransitionBase<? extends Enum<V>>> {
  // Array where the index [1][2] = TransitionBase from state at ordinal 1 to state at ordinal 2
  private final Object[][] adjacencyMap;
  @SuppressWarnings("unused")
  private final Class<V> enumType;

  public DirectionalEnumGraph(Class<V> enumType) {
    int c = enumType.getEnumConstants().length;
    this.enumType = enumType;

    adjacencyMap = new Object[c][c];
  }

  /**
   * Update the adjacency map to include the data for the given transition Note: This WILL NOT
   * overwrite an existing edge
   *
   * @param transition The transition to add to the graph
   */
  public void addEdge(T transition) {
    setEdge(transition);
  }

  public void addEdges(@SuppressWarnings("unchecked") T... transitions) {
    for (T transition : transitions) {
      addEdge(transition);
    }
  }

  @SuppressWarnings("unchecked")
  private T getAsEdge(int x, int y) {
    return (T) adjacencyMap[x][y];
  }

  /**
   * Update the adjacency map to include the data for the given transition. Note: This WILL override
   * an existing transition, if one is present
   *
   * @param transition The transition to set to the graph
   */
  public void setEdge(T transition) {
    adjacencyMap[transition.getStartState().ordinal()][transition.getEndState().ordinal()] =
        transition;
  }

  public void removeEdge(V start, V end) {
    adjacencyMap[start.ordinal()][end.ordinal()] = null;
  }

  /**
   * Returns the edge of the graph
   *
   * @param start starting vertex of the edge
   * @param end ending vertex of the edge
   * @return the edge, if one is found. Otherwise, the method will return null
   */
  public T getEdge(V start, V end) {
    return getAsEdge(start.ordinal(), end.ordinal());
  }

  /**
   * Returns the edges of the graph either going to, from, or both to and from a vertex of the graph
   *
   * @param vertex the vertex to search
   * @param t the type of resultant edges to return
   * @return the edges of the graph that match the conditions
   */
  public List<T> getEdges(V vertex, EdgeType t) {
    List<T> outgoing = new ArrayList<>();
    List<T> incoming = new ArrayList<>();

    for (int i = 0; i < adjacencyMap.length; i++) {
      T in = getAsEdge(i, vertex.ordinal());
      T out = getAsEdge(vertex.ordinal(), i);
      if (out != null) outgoing.add(out);
      if (in != null) incoming.add(in);
    }

    switch (t) {
      case Incoming:
        return incoming;
      case Outgoing:
        return outgoing;
      default: // Return both
        outgoing.addAll(incoming);
        return outgoing;
    }
  }
}