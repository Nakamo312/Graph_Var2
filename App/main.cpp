#include "Graph.h"
using namespace Graphs;

template<typename Vertex, typename Distance>
Vertex find_point( Graph<Vertex>& graph)
{
    Vertex point;
    std::vector<Vertex> points = graph.vertices();
    Distance min_d = 1e9;
    for (auto it = points.begin(); it != (points.end() - 1); ++it)
    {
        Distance max_d = 0;
        for (auto jt = points.begin(); jt != (points.end() - 1); ++jt)
        {
            if (*it != *jt)
            {
                Distance d = graph.calc_min_distance(*it, *jt);
                if (d > max_d)
                {
                    max_d = d;
                }
            }
        }
        if (max_d < min_d)
        {
            min_d = max_d;
            point = *it;
        }
    }
    return point;
}

int main()
{
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
    std::cout << find_point<char,int>(graph);
	return 0;
}