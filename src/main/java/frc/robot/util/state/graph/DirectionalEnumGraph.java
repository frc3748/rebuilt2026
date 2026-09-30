package frc.robot.util.state.graph;

import java.util.*;

import frc.robot.util.state.transitions.TransitionBase;

public class DirectionalEnumGraph<V extends Enum<V>, T extends TransitionBase<? extends Enum<V>>> {
  private final Object[][] adjacencyMap;
  @SuppressWarnings("unused")
  private final Class<V> enumType;

  public DirectionalEnumGraph(Class<V> enumType) {
    int c = enumType.getEnumConstants().length;
    this.enumType = enumType;

    adjacencyMap = new Object[c][c];
  }

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

  public void setEdge(T transition) {
    adjacencyMap[transition.getStartState().ordinal()][transition.getEndState().ordinal()] =
        transition;
  }

  public void removeEdge(V start, V end) {
    adjacencyMap[start.ordinal()][end.ordinal()] = null;
  }

  public T getEdge(V start, V end) {
    return getAsEdge(start.ordinal(), end.ordinal());
  }

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
      default:
        outgoing.addAll(incoming);
        return outgoing;
    }
  }
}