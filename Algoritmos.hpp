#ifndef ALGORITMOS_HPP
#define ALGORITMOS_HPP

#include <queue>
#include <unordered_set>
#include <vector>
#include <iostream>
#include <string>
#include <limits>
#include "Board.hpp" 


// Definición de un nodo en la búsqueda
struct Node {
    U64 board[3];  
    int depth;     
    std::vector<std::pair<int, int>> moves;
    bool operator==(const Node& other) const;
};

struct NodeDijkstra
{
    U64 board[3];
    int cost; // Costo para llegar a este nodo desde el inicio
    std::vector<int> path; // Para almacenar el camino o movimientos hechos

    // Comparador para la cola de prioridad
    bool operator==(const NodeDijkstra& other) const;
    bool operator>(const NodeDijkstra& other) const;
    void print();
};


// Definición de un nodo para A* (con heurística)
struct NodeAStar {
    U64 board[3];  // Representación del tablero con bitboards
    int depth;      // Profundidad (número de movimientos realizados)
    int heuristic;  // Heurística para la función A*
    std::vector<std::pair<int, int>> moves; // Lista de movimientos realizados
    bool operator==(const NodeAStar& other) const;
    bool operator>(const NodeAStar& other) const;
    void print();  //Para Debuggear NodeAStar
};


//Declaracion de HASH
namespace std {
    template <>
    struct hash<Node> {
        size_t operator()(const Node& node) const;
    };
}
namespace std {
    template <>
    struct hash<NodeAStar> {
        size_t operator()(const NodeAStar& node) const;
    };
}

// Definir un hash para NodeDijkstra
namespace std {
    template <>
    struct hash<NodeDijkstra> {
        size_t operator()(const NodeDijkstra& node) const {
            size_t result = 0;
            // Calculamos el hash de cada campo (usando uint64_t en lugar de int)
            for (int i = 0; i < 3; ++i) {
                result ^= std::hash<U64>{}(node.board[i]) + 0x9e3779b9 + (result << 6) + (result >> 2); // Técnica de hash común
            }
            result ^= std::hash<U64>{}(node.cost) + 0x9e3779b9 + (result << 6) + (result >> 2);
            for (const auto& p : node.path) {
                result ^= std::hash<int>{}(p) + 0x9e3779b9 + (result << 6) + (result >> 2);
            }
            return result;
        }
    };
}

bool bfs(Board& board);  //Búsqueda por amplitud (Breadth-First Search)
int heuristic(const NodeAStar& node); // Heurística para A*
bool astar(Board& board); // A* Search
bool ida_star(Board& board); // IDA* Search
int search_ida(Board& board, NodeAStar startNode, int depth, int limit, int& nodesExplored);

bool dfs(Board& Board);
bool start_dfs(Board& board);
bool Dijkstra(Board& board);
bool WeightedAStar(Board& board, double weight);
bool JPS();

bool BeamSearch(Board& board, int beamWidth);
int RBFS(Board& board, NodeAStar node, int fLimit, int& nodesExplored); 
bool start_rbfs(Board& board);
bool SMAStar(Board& board, int maxMemory); 
//void print_results(int nodesExplored, std::chrono::high_resolution_clock::time_point start_time);
#endif // ALGORITMOS_HPP
