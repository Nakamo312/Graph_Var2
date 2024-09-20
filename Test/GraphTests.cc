#include "gtest/gtest.h"
#include "Graph.h"
using namespace std;
using namespace Graphs;

TEST(GraphTest, AddAndRemoveVertex) {
    Graph<int> graph;

    graph.add_vertex(1);
    graph.add_vertex(2);
    graph.add_vertex(3);
    graph.remove_vertex(3);
    EXPECT_TRUE(graph.has_vertex(1));
    EXPECT_TRUE(graph.has_vertex(2));
    EXPECT_FALSE(graph.has_vertex(3));
}
Graph<char> graph;

TEST(GraphTest, AddAndRemoveEdge) {
    Graph<int> graph;

    graph.add_vertex(1);
    graph.add_vertex(2);

    graph.add_edge(1, 2, 5.0);

    EXPECT_TRUE(graph.has_edge(1, 2));
    EXPECT_FALSE(graph.has_edge(2, 1));

    graph.remove_edge(1, 2);

    EXPECT_FALSE(graph.has_edge(1, 2));

}
TEST(GraphTest, ShortestPath) {
    Graph<char> graph;
    graph.add_vertex('s');
    graph.add_vertex('t');
    graph.add_vertex('y');
    graph.add_vertex('x');
    graph.add_vertex('z');

    graph.add_edge('s', 't', 10);
    graph.add_edge('s', 'y', 5);
    graph.add_edge('t', 'y', 2);
    graph.add_edge('y', 't', 3);
    graph.add_edge('t', 'x', 1);
    graph.add_edge('y', 'x', 9);
    graph.add_edge('x', 'z', 4);
    graph.add_edge('z', 'x', 6);
    graph.add_edge('z', 's', 7);
    graph.add_edge('y', 'z', 2);
    std::vector<char> result = graph.shortest_path('s', 'z');
    std::vector<char> answer = { 's','y','z' };
    EXPECT_EQ(result, answer);

}
TEST(GraphTest, walk) {
    Graph<char> graph;
    graph.add_vertex('s');
    graph.add_vertex('t');
    graph.add_vertex('y');
    graph.add_vertex('x');
    graph.add_vertex('z');

    graph.add_edge('s', 't', 10);
    graph.add_edge('s', 'y', 5);
    graph.add_edge('t', 'y', 2);
    graph.add_edge('y', 't', 3);
    graph.add_edge('t', 'x', 1);
    graph.add_edge('y', 'x', 9);
    graph.add_edge('x', 'z', 4);
    graph.add_edge('z', 'x', 6);
    graph.add_edge('z', 's', 7);
    graph.add_edge('y', 'z', 2);
    std::vector<char> result;
    graph.BFS('s', result);
    std::vector<char> answer = { 's','t','y','x','z'};
    EXPECT_EQ(result, answer);

}
TEST(GraphTest, min_distance) {
    Graph<char> graph;
    graph.add_vertex('s');
    graph.add_vertex('t');
    graph.add_vertex('y');
    graph.add_vertex('x');
    graph.add_vertex('z');

    graph.add_edge('s', 't', 10);
    graph.add_edge('s', 'y', 5);
    graph.add_edge('t', 'y', 2);
    graph.add_edge('y', 't', 3);
    graph.add_edge('t', 'x', 1);
    graph.add_edge('y', 'x', 9);
    graph.add_edge('x', 'z', 4);
    graph.add_edge('z', 'x', 6);
    graph.add_edge('z', 's', 7);
    graph.add_edge('y', 'z', 2);
    EXPECT_EQ(graph.calc_min_distance('s', 'x'),9);
}