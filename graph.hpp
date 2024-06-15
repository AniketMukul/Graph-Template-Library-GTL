#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <iostream>
#include <map>
#include <set>
#include <vector>
#include <queue>
#include <stack>
#include <limits>
#include <algorithm>

template <typename T>
class Graph
{
public:
    void addNode(T node);
    void addEdge(T src, T dest, int weight = 1);
    void addWeightedEdge(T src, T dest, int weight);
    void removeEdge(T src, T dest);
    std::vector<T> bfs(T start);
    std::vector<T> dfs(T start);
    std::vector<T> shortestPath(T start, T end);
    std::vector<T> topologicalSort();
    std::vector<std::pair<int, std::pair<T, T>>> kruskal();
    std::vector<std::pair<T, T>> prim();

private:
    std::map<T, std::map<T, int>> adjList;
    std::vector<std::pair<int, std::pair<T, T>>> edges;

    void dfsUtil(T node, std::set<T> &visited, std::vector<T> &result);
    void topologicalSortUtil(T node, std::set<T> &visited, std::stack<T> &Stack);
    int findParent(T node, std::map<T, T> &parent);
    void unionNodes(T u, T v, std::map<T, T> &parent, std::map<T, int> &rank);
};

template <typename T>
void Graph<T>::addNode(T node)
{
    adjList[node] = std::map<T, int>();
}

template <typename T>
void Graph<T>::addEdge(T src, T dest, int weight)
{
    adjList[src][dest] = weight;
    adjList[dest][src] = weight; // For undirected graph
    edges.push_back({weight, {src, dest}});
}

template <typename T>
void Graph<T>::addWeightedEdge(T src, T dest, int weight)
{
    addEdge(src, dest, weight);
}

template <typename T>
void Graph<T>::removeEdge(T src, T dest)
{
    adjList[src].erase(dest);
    adjList[dest].erase(src);
    edges.erase(std::remove_if(edges.begin(), edges.end(),
                               [src, dest](const std::pair<int, std::pair<T, T>> &edge)
                               {
                                   return (edge.second.first == src && edge.second.second == dest) ||
                                          (edge.second.first == dest && edge.second.second == src);
                               }),
                edges.end());
}

template <typename T>
std::vector<T> Graph<T>::bfs(T start)
{
    std::vector<T> result;
    std::set<T> visited;
    std::queue<T> q;

    visited.insert(start);
    q.push(start);

    while (!q.empty())
    {
        T node = q.front();
        q.pop();
        result.push_back(node);

        for (const auto &neighbor : adjList[node])
        {
            if (visited.find(neighbor.first) == visited.end())
            {
                visited.insert(neighbor.first);
                q.push(neighbor.first);
            }
        }
    }
    return result;
}

template <typename T>
void Graph<T>::dfsUtil(T node, std::set<T> &visited, std::vector<T> &result)
{
    visited.insert(node);
    result.push_back(node);

    for (const auto &neighbor : adjList[node])
    {
        if (visited.find(neighbor.first) == visited.end())
        {
            dfsUtil(neighbor.first, visited, result);
        }
    }
}

template <typename T>
std::vector<T> Graph<T>::dfs(T start)
{
    std::vector<T> result;
    std::set<T> visited;
    dfsUtil(start, visited, result);
    return result;
}

template <typename T>
std::vector<T> Graph<T>::shortestPath(T start, T end)
{
    std::map<T, int> distances;
    std::map<T, T> previous;
    std::set<T> visited;
    auto compare = [&distances](T left, T right)
    { return distances[left] > distances[right]; };
    std::priority_queue<T, std::vector<T>, decltype(compare)> queue(compare);

    for (const auto &pair : adjList)
    {
        distances[pair.first] = std::numeric_limits<int>::max();
    }
    distances[start] = 0;
    queue.push(start);

    while (!queue.empty())
    {
        T current = queue.top();
        queue.pop();

        if (current == end)
            break;

        visited.insert(current);

        for (const auto &neighbor : adjList[current])
        {
            if (visited.find(neighbor.first) != visited.end())
                continue;

            int newDist = distances[current] + neighbor.second;
            if (newDist < distances[neighbor.first])
            {
                distances[neighbor.first] = newDist;
                previous[neighbor.first] = current;
                queue.push(neighbor.first);
            }
        }
    }

    std::vector<T> path;
    for (T at = end; at != start; at = previous[at])
    {
        path.push_back(at);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

template <typename T>
void Graph<T>::topologicalSortUtil(T node, std::set<T> &visited, std::stack<T> &Stack)
{
    visited.insert(node);
    for (const auto &neighbor : adjList[node])
    {
        if (visited.find(neighbor.first) == visited.end())
        {
            topologicalSortUtil(neighbor.first, visited, Stack);
        }
    }
    Stack.push(node);
}

template <typename T>
std::vector<T> Graph<T>::topologicalSort()
{
    std::stack<T> Stack;
    std::set<T> visited;
    std::vector<T> result;

    for (const auto &pair : adjList)
    {
        if (visited.find(pair.first) == visited.end())
        {
            topologicalSortUtil(pair.first, visited, Stack);
        }
    }

    while (!Stack.empty())
    {
        result.push_back(Stack.top());
        Stack.pop();
    }

    return result;
}

template <typename T>
int Graph<T>::findParent(T node, std::map<T, T> &parent)
{
    if (parent[node] == node)
    {
        return node;
    }
    return parent[node] = findParent(parent[node], parent);
}

template <typename T>
void Graph<T>::unionNodes(T u, T v, std::map<T, T> &parent, std::map<T, int> &rank)
{
    T parentU = findParent(u, parent);
    T parentV = findParent(v, parent);

    if (rank[parentU] < rank[parentV])
    {
        parent[parentU] = parentV;
    }
    else if (rank[parentU] > rank[parentV])
    {
        parent[parentV] = parentU;
    }
    else
    {
        parent[parentV] = parentU;
        rank[parentU]++;
    }
}

template <typename T>
std::vector<std::pair<int, std::pair<T, T>>> Graph<T>::kruskal()
{
    std::vector<std::pair<int, std::pair<T, T>>> result;
    std::map<T, T> parent;
    std::map<T, int> rank;

    for (const auto &pair : adjList)
    {
        parent[pair.first] = pair.first;
        rank[pair.first] = 0;
    }

    std::sort(edges.begin(), edges.end());

    for (const auto &edge : edges)
    {
        T u = edge.second.first;
        T v = edge.second.second;
        int weight = edge.first;

        T parentU = findParent(u, parent);
        T parentV = findParent(v, parent);

        if (parentU != parentV)
        {
            result.push_back(edge);
            unionNodes(parentU, parentV, parent, rank);
        }
    }

    return result;
}

template <typename T>
std::vector<std::pair<T, T>> Graph<T>::prim()
{
    std::vector<std::pair<T, T>> result;
    std::set<T> visited;
    std::priority_queue<std::pair<int, std::pair<T, T>>, std::vector<std::pair<int, std::pair<T, T>>>, std::greater<>> minHeap;

    T start = adjList.begin()->first;
    visited.insert(start);

    for (const auto &neighbor : adjList[start])
    {
        minHeap.push({neighbor.second, {start, neighbor.first}});
    }

    while (!minHeap.empty())
    {
        auto edge = minHeap.top();
        minHeap.pop();

        T u = edge.second.first;
        T v = edge.second.second;

        if (visited.find(v) == visited.end())
        {
            visited.insert(v);
            result.push_back({u, v});

            for (const auto &neighbor : adjList[v])
            {
                if (visited.find(neighbor.first) == visited.end())
                {
                    minHeap.push({neighbor.second, {v, neighbor.first}});
                }
            }
        }
    }

    return result;
}

#endif // GRAPH_HPP
