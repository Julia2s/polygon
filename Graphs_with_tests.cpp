#include <iostream>
#include <chrono>
#include <vector>
#include <map>
#include <queue>
#include <stack>
#include <set>
#include <limits>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <fstream>

using namespace std;
using namespace chrono;

// Структуры графа
struct Node {
    double lat, lon;
    vector<pair<Node*, double>> neighbors; // Соседи с весами
};

struct Graph {
    map<pair<double, double>, Node> nodes;

    // Загрузка данных из файла
    void load_from_file(const string& filename) {
        ifstream file(filename);
        string line;
        while (getline(file, line)) {
            size_t pos = line.find(':');
            if (pos == string::npos) continue;

            // Родительский узел
            string parentPart = line.substr(0, pos);
            double lon1, lat1;
            sscanf(parentPart.c_str(), "%lf,%lf", &lon1, &lat1);
            Node& parent = nodes[{lon1, lat1}];
            parent.lat = lat1;
            parent.lon = lon1;

            // Соседи
            string neighborsPart = line.substr(pos + 1);
            stringstream ss(neighborsPart);
            string neighborEntry;
            while (getline(ss, neighborEntry, ';')) {
                double lon2, lat2, weight;
                sscanf(neighborEntry.c_str(), "%lf,%lf,%lf", &lon2, &lat2, &weight);
                Node& neighbor = nodes[{lon2, lat2}];
                neighbor.lat = lat2;
                neighbor.lon = lon2;
                parent.neighbors.emplace_back(&neighbor, weight);
            }
        }
        file.close();
    }

    // Поиск ближайшего узла к заданным координатам
    Node* find_closest_node(double lat, double lon) {
        Node* closest = nullptr;
        double min_distance = numeric_limits<double>::infinity();
        for (auto& [coords, node] : nodes) {
            double distance = sqrt(pow(lat - node.lat, 2) + pow(lon - node.lon, 2));
            if (distance < min_distance) {
                min_distance = distance;
                closest = &node;
            }
        }
        return closest;
    }
};

// BFS
vector<Node*> bfs(Graph& graph, Node* start, Node* goal) {
    queue<Node*> q; // Память: O(V), где V — количество узлов (для очереди)
    map<Node*, Node*> parent; // Память: O(V) для хранения пути
    set<Node*> visited; // Память: O(V) для хранения посещённых узлов

    q.push(start);
    visited.insert(start);

    while (!q.empty()) { // Время: O(V + E), где V — количество узлов, E — количество рёбер
        Node* current = q.front();
        q.pop();

        if (current == goal) {
            vector<Node*> path; // Память: O(V) для хранения пути
            for (Node* at = goal; at != nullptr; at = parent[at])
                path.push_back(at);
            reverse(path.begin(), path.end()); // Время: O(V) для разворота пути
            return path;
        }

        for (auto& [neighbor, _] : current->neighbors) { // Время: O(E) для обхода соседей
            if (visited.find(neighbor) == visited.end()) {
                visited.insert(neighbor);
                parent[neighbor] = current;
                q.push(neighbor);
            }
        }
    }

    return {}; // Путь не найден
}

// DFS
vector<Node*> dfs(Graph& graph, Node* start, Node* goal) {
    stack<Node*> s; // Память: O(V) для стека
    map<Node*, Node*> parent; // Память: O(V) для хранения пути
    set<Node*> visited; // Память: O(V) для хранения посещённых узлов

    s.push(start);

    while (!s.empty()) { // Время: O(V + E), где V — количество узлов, E — количество рёбер
        Node* current = s.top();
        s.pop();

        if (visited.find(current) != visited.end()) continue;
        visited.insert(current);

        if (current == goal) {
            vector<Node*> path; // Память: O(V) для хранения пути
            for (Node* at = goal; at != nullptr; at = parent[at])
                path.push_back(at);
            reverse(path.begin(), path.end()); // Время: O(V) для разворота пути
            return path;
        }

        for (auto& [neighbor, _] : current->neighbors) { // Время: O(E) для обхода соседей
            if (visited.find(neighbor) == visited.end()) {
                parent[neighbor] = current;
                s.push(neighbor);
            }
        }
    }

    return {}; // Путь не найден
}

// Dijkstra
vector<Node*> dijkstra(Graph& graph, Node* start, Node* goal) {
    map<Node*, double> distances; // Память: O(V) для хранения расстояний
    map<Node*, Node*> parent; // Память: O(V) для хранения пути
    set<Node*> visited; // Память: O(V) для хранения посещённых узлов
    priority_queue<pair<double, Node*>, vector<pair<double, Node*>>, greater<>> pq; // Память: O(V) для очереди с приоритетом

    for (auto& [_, node] : graph.nodes) {
        distances[&node] = numeric_limits<double>::infinity();
    }
    distances[start] = 0;
    pq.emplace(0, start);

    while (!pq.empty()) { // Время: O((V + E) * log(V)), где V — количество узлов, E — количество рёбер
        auto [dist, current] = pq.top();
        pq.pop();

        if (visited.find(current) != visited.end()) continue;
        visited.insert(current);

        if (current == goal) {
            vector<Node*> path; // Память: O(V) для хранения пути
            for (Node* at = goal; at != nullptr; at = parent[at])
                path.push_back(at);
            reverse(path.begin(), path.end()); // Время: O(V) для разворота пути
            return path;
        }

        for (auto& [neighbor, weight] : current->neighbors) { // Время: O(E) для обхода соседей
            double new_dist = dist + weight;
            if (new_dist < distances[neighbor]) {
                distances[neighbor] = new_dist;
                parent[neighbor] = current;
                pq.emplace(new_dist, neighbor);
            }
        }
    }

    return {}; // Путь не найден
}

// A*
vector<Node*> a_star(Graph& graph, Node* start, Node* goal) {
    auto heuristic = [&](Node* a, Node* b) {
        return sqrt(pow(a->lat - b->lat, 2) + pow(a->lon - b->lon, 2)); // Время: O(1) для вычисления эвристики
    };

    map<Node*, double> g_score; // Память: O(V) для хранения g-оценок
    map<Node*, double> f_score; // Память: O(V) для хранения f-оценок
    map<Node*, Node*> parent; // Память: O(V) для хранения пути
    priority_queue<pair<double, Node*>, vector<pair<double, Node*>>, greater<>> pq; // Память: O(V) для очереди с приоритетом

    for (auto& [_, node] : graph.nodes) {
        g_score[&node] = numeric_limits<double>::infinity();
        f_score[&node] = numeric_limits<double>::infinity();
    }
    g_score[start] = 0;
    f_score[start] = heuristic(start, goal);
    pq.emplace(f_score[start], start);

    while (!pq.empty()) { // Время: O((V + E) * log(V)), где V — количество узлов, E — количество рёбер
        auto [_, current] = pq.top();
        pq.pop();

        if (current == goal) {
            vector<Node*> path; // Память: O(V) для хранения пути
            for (Node* at = goal; at != nullptr; at = parent[at])
                path.push_back(at);
            reverse(path.begin(), path.end()); // Время: O(V) для разворота пути
            return path;
        }

        for (auto& [neighbor, weight] : current->neighbors) { // Время: O(E) для обхода соседей
            double tentative_g_score = g_score[current] + weight;
            if (tentative_g_score < g_score[neighbor]) {
                parent[neighbor] = current;
                g_score[neighbor] = tentative_g_score;
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal);
                pq.emplace(f_score[neighbor], neighbor);
            }
        }
    }

    return {}; // Путь не найден
}

void run_tests() {
    Graph graph;
    graph.load_from_file("spb_graph.txt");

    // Тест на загрузку графа
    assert(graph.nodes.size() == 4); // Ожидаем 4 узла
    cout << "Graph loaded successfully" << endl;

    // Тест на поиск ближайшего узла
    Node* closest = graph.find_closest_node(50.0, 50.0);
    assert(closest != nullptr);
    assert(closest->lat == 50.0 && closest->lon == 50.0);
    cout << "Closest node test passed" << endl;

    // Тест на BFS
    Node* start = graph.find_closest_node(50.0, 50.0);
    Node* goal = graph.find_closest_node(53.0, 53.0);
    vector<Node*> bfs_path = bfs(graph, start, goal);
    assert(!bfs_path.empty());
    assert(bfs_path.front() == start && bfs_path.back() == goal);
    cout << "BFS path found" << endl;

    // Тест на DFS
    vector<Node*> dfs_path = dfs(graph, start, goal);
    assert(!dfs_path.empty());
    assert(dfs_path.front() == start && dfs_path.back() == goal);
    cout << "DFS path found" << endl;

    // Тест на Dijkstra
    vector<Node*> dijkstra_path = dijkstra(graph, start, goal);
    assert(!dijkstra_path.empty());
    assert(dijkstra_path.front() == start && dijkstra_path.back() == goal);
    cout << "Dijkstra path found" << endl;

    // Тест на A*
    vector<Node*> a_star_path = a_star(graph, start, goal);
    assert(!a_star_path.empty());
    assert(a_star_path.front() == start && a_star_path.back() == goal);
    cout << "A* path found" << endl;
}

int main() {
    Graph graph;
    graph.load_from_file("spb_graph.txt");

    Node* start_node = graph.find_closest_node(59.867173, 30.307543);
    Node* goal_node = graph.find_closest_node(30.309988, 30.308108);

    auto start_time = high_resolution_clock::now();
    auto bfs_path = bfs(graph, start_node, goal_node);
    auto end_time = high_resolution_clock::now();
    cout << "BFS Time: " << duration<double>(end_time - start_time).count() << " seconds\n";

    start_time = high_resolution_clock::now();
    auto dfs_path = dfs(graph, start_node, goal_node);
    end_time = high_resolution_clock::now();
    cout << "DFS Time: " << duration<double>(end_time - start_time).count() << " seconds\n";

    start_time = high_resolution_clock::now();
    auto dijkstra_path = dijkstra(graph, start_node, goal_node);
    end_time = high_resolution_clock::now();
    cout << "Dijkstra Time: " << duration<double>(end_time - start_time).count() << " seconds\n";

    start_time = high_resolution_clock::now();
    auto a_star_path = a_star(graph, start_node, goal_node);
    end_time = high_resolution_clock::now();
    cout << "A* Time: " << duration<double>(end_time - start_time).count() << " seconds\n";

    run_tests();

    return 0;
}
