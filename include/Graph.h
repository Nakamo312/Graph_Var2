#pragma once
#include <unordered_map>
#include <list>
#include <queue>
#include <vector>
#include <set>
#include <iostream>
namespace Graphs
{
    template<typename Vertex, typename Distance = double>
    class Graph
    {

        struct CompareVertex
        {
            bool operator()(const std::pair<Vertex, Distance>& v1, const std::pair<Vertex, Distance>& v2) const
            {
                return v1.second > v2.second;
            }
        };
        struct Edge
        {
            Vertex id;
            Distance d;
            Edge(const Vertex& id, const Distance& d) : id(id), d(d) {};
        };
    private:

        std::unordered_map<Vertex, std::list<Edge>> _vertices;
        Distance get_weight(const Vertex& from, const Vertex& to)
        {
            if (has_vertex(from) && has_vertex(to))
            {
                for (auto it = _vertices[from].begin(); it != _vertices[from].end(); ++it)
                {
                    if (it->id == to)
                    {
                        return it->d;
                    }
                }
                throw("This vertices do not adjacent ");
            }
            throw("Any vertices do not exist");
        }
        void calc_dist_for_each(const Vertex from, std::unordered_map<Vertex, Distance>& result)
        {

            if (result.contains(from))
            {
                result[from] = 0;
                std::priority_queue<std::pair<Vertex, Distance>, std::vector<std::pair<Vertex, Distance>>, CompareVertex> pq;
                pq.push({ from, result[from] });

                while (!pq.empty())
                {
                    Vertex u = pq.top().first;
                    Distance u_d = pq.top().second;
                    pq.pop();
                    for (auto it = _vertices[u].begin(); it != _vertices[u].end(); ++it)
                    {
                        Vertex v = it->id;
                        Distance d = get_weight(u, v);
                        if (result[v] > (u_d + d))
                        {
                            result[v] = u_d + d;
                            pq.push({ v, result[v] });
                        }
                    }
                }
            }
            else
            {
                throw("Out data does not contains this vertex");
            }
        }
        void _BFS(Vertex& start, std::vector<Vertex>& out)
        {
            std::vector<Vertex> visited(_vertices.size());

            std::queue<Vertex> Q;

            visited.push_back(start);
            Q.push(start);

            while (!Q.empty())
            {
                Vertex u = Q.front();
                Q.pop();

                out.push_back(u);
                for (const auto& [neighbor, distance] : _vertices[u])
                {
                    bool is_visited = false;
                    for (auto it_visited = visited.begin(); it_visited != visited.end(); ++it_visited)
                    {
                        if (*it_visited == neighbor)
                        {
                            is_visited = true;
                            break;
                        }
                    }
                    if (!is_visited)
                    {
                        visited.push_back(neighbor);
                        Q.push(neighbor);
                    }

                }
            }
        }


        Edge& get_edge(const Vertex& from, const Vertex& to)
        {
            if (has_vertex(from) && has_vertex(to))
            {
                for (auto it = _vertices[from].begin(); it != _vertices[from].end(); ++it)
                {
                    if (it->id == to)
                    {
                        return *it;
                    }
                }
                throw std::runtime_error("This vertices do not adjacent ");
            }
            throw std::runtime_error("Any vertices do not exist");
        }
    public:

        bool has_vertex(const Vertex& v) const
        {
            return (_vertices.find(v) != _vertices.end());
        }
        void add_vertex(const Vertex& v)
        {
            if (!_vertices.contains(v))
            {
                std::list<Edge> _list;
                _vertices.insert(std::make_pair(v, _list));
                return;
            }
            throw std::runtime_error("Vertex with this id is already exist");
        }
        bool remove_vertex(const Vertex& v)
        {
            if (has_vertex(v))
            {
                _vertices.erase(v);
                for (auto& [key, edges_list] : _vertices)
                {
                    for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
                    {
                        if (it->id == v)
                        {
                            edges_list.erase(it);
                        }
                    }
                }
                return true;
            }
            return false;
        }
        void add_edge(const Vertex& from, const Vertex& to, const Distance& d)
        {
            if (has_vertex(from) && has_vertex(to))
            {
                _vertices[from].push_back(Edge(to, d));
                return;
            }
            throw std::runtime_error("Any vertices do not exist");
        }
        bool remove_edge(const Vertex& from, const Vertex& to)
        {
            if (has_vertex(from) && has_vertex(to)) {
                auto it = _vertices[from].begin();
                while (it != _vertices[from].end())
                {
                    if (it->id == to)
                    {
                        it = _vertices[from].erase(it);
                    }
                    else
                    {
                        ++it;
                    }
                }
                return true;
            }
            return false;
        }
            bool remove_edge(const Edge & e)
            {
                for (auto& [key, edges_list] : _vertices)
                {
                    for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
                    {
                        if (it == e)
                        {
                            edges_list.erase(it);
                            return true;
                        }
                    }
                }
                return false;
            }
            bool has_edge(const Vertex & from, const Vertex & to)
            {
                if (has_vertex(from) && has_vertex(to))
                {
                    for (auto it = _vertices[from].begin(); it != _vertices[from].end(); ++it)
                    {
                        if (it->id == to)
                        {
                            return true;
                        }
                    }
                    return false;
                }
                return false;
            }
            bool has_edge(const Edge & e) const
            {
                for (auto& [key, edges_list] : _vertices)
                {
                    for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
                    {
                        if (it == e)
                        {
                            return true;
                        }
                    }
                }
                return false;
            }
            std::vector<Vertex> vertices() const
            {
                std::vector<Vertex> v;
                for (auto& [key, edges_list] : _vertices)
                {
                    v.push_back(key);
                }
                return v;
            }
            std::vector<Edge> edges(const Vertex & vertex)
            {
                std::vector<Edge> edges;
                for (auto it = _vertices[vertex].begin(); it != _vertices[vertex].end(); ++it)
                {
                    edges.push_back(it);
                }
            }
            size_t order() const
            {
                return _vertices.size();
            }
            size_t degree(const Vertex & v) const
            {
                _vertices[v].size();
            }
            Distance calc_min_distance(const Vertex & from, const Vertex & to)
            {
                std::unordered_map<Vertex, Distance> distances;
                for (auto& [key, edges_list] : _vertices)
                {
                    distances.insert({ key, std::numeric_limits<double>::infinity() });
                }
                try
                {
                    calc_dist_for_each(from, distances);
                }
                catch (std::runtime_error& e)
                {
                    return std::numeric_limits<double>::infinity();
                }
                return distances[to];
            }
            std::vector<Vertex> shortest_path(const Vertex & from, const Vertex & to)
            {
                std::vector<Vertex> path = std::vector<Vertex>();
                std::unordered_map<Vertex, Distance> distances;
                for (auto& [key, edges_list] : _vertices)
                {
                    distances.insert({ key, std::numeric_limits<double>::infinity() });
                }
                try
                {
                    calc_dist_for_each(from, distances);
                }
                catch (std::runtime_error& e)
                {
                    return path;
                }

                Vertex current = to;
                path.push_back(current);
                while (current != from)
                {
                    for (auto& [key, edges_list] : _vertices)
                    {
                        for (const auto& [neighbor, distance] : _vertices[key])
                        {
                            if (neighbor == current)
                            {
                                if (distances[key] + distance == distances[neighbor])
                                {
                                    current = key;
                                    path.insert(path.begin(), current);
                                    break;
                                }
                            }
                        }

                    }

                }
                return path;
            }

            void BFS(const Vertex & start_vertex, std::vector<Vertex>&out)
            {
                for (auto& [key, edges_list] : _vertices)
                {
                    Vertex current = start_vertex;
                    bool is_consist = false;
                    for (auto it = out.begin(); it != out.end(); ++it)
                    {
                        current = *it;
                        if (key == *it)
                        {
                            is_consist = true;
                            break;
                        }
                    }
                    if (!is_consist)
                    {
                        _BFS(current, out);

                    }
                }

            }

        };

    }
