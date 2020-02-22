/**
 * Aluno: Daniel Martins Reis
 * Como compilar: g++ tp2_daniel_reis.cpp -o tp2
 * Como executar: ./tp2
 */

#include <math.h>
#include <dirent.h>
#include <iostream>
#include <list>
#include <algorithm> // função find
#include <fstream>   // leitura de arquivos
#include <tuple>
#include <time.h>
#include <cstdlib>  // função random

using namespace std;

class Solution {
    list<int> path;
    float cost;
    
    public:
        Solution(list<int> path, float cost); // construtor
        list<int> getPath();
        float getCost();
};

Solution::Solution(list<int> path, float cost) {
    this->path = path;
    this->cost = cost;
}

list<int> Solution::getPath() {
    return path;
}

float Solution::getCost() {
    return cost;
}

class Graph {
    string NAME, TYPE, EDGE_WEIGHT_TYPE;
    int V; // número de vértices
    list<pair<int, float>> *adj; // ponteiro para um array contendo as listas de adjacências
    list<pair<float,float>> vertex;

    public:
        Graph(int V, string NAME, string TYPE, string EDGE_WEIGHT_TYPE); // construtor
        void addEdge(int v1, int v2, float weight); // adiciona aresta no grafo
        void addVertex(float px, float py); // adiciona posição de um vértice
        int getV();
        list<pair<float,float>> getVertex();
        list<pair<int, float>> getAdj(int vertex);
};

Graph::Graph(int V, string NAME, string TYPE, string EDGE_WEIGHT_TYPE) {
    this->V = V; // atribui o número de vértices
    this->NAME = NAME;
    this->TYPE = TYPE;
    this->EDGE_WEIGHT_TYPE = EDGE_WEIGHT_TYPE;
    adj = new list<pair<int, float>>[V]; // cria as listas
}

void Graph::addEdge(int v1, int v2, float weight) {
    adj[v1].push_back(std::make_pair(v2, weight)); // adiciona vértice v2 à lista de vértices adjacentes de v1
}

void Graph::addVertex(float px, float py) {
    vertex.push_back(std::make_pair(px, py)); // adiciona vértice v2 à lista de vértices adjacentes de v1
}

int Graph::getV() {
    return V;
}

list<pair<float,float>> Graph::getVertex() {
    return vertex;
}

list<pair<int, float>> Graph::getAdj(int vertex) {
    return adj[vertex];
}

Graph readGraph(std::string filename) {
    // lê do arquivo de entrada
    std::string arquivoEntrada(filename);
    std::ifstream in(arquivoEntrada);

    if (!in.is_open()) {
        std::cout << "Nao foi possivel encontrar o arquivo de entrada informado!" << std::endl;
        exit(0);
    }
    
    std::streambuf *cinbuf = std::cin.rdbuf();
    std::cin.rdbuf(in.rdbuf());

    std::string line;
    std::string label;
    std::string valueString;
    std::string valueStringD;
    std::string NAME;
    std::string TYPE;
    std::string EDGE_WEIGHT_TYPE;
    int DIMENSION; // número de vértices

    //Leitura das 6 primeiras linhas
    std::getline(std::cin, label,':'); //label
    std::getline(std::cin, valueString,' '); // espaco em branco
    std::getline(std::cin, NAME); // armazena valor em uma string
//     std::cout << NAME << std::endl;
    
    std::getline(std::cin, label,':'); //label
    std::getline(std::cin, valueString,' '); // espaco em branco
    std::getline(std::cin, TYPE); // armazena valor em uma string
//     std::cout << TYPE << std::endl;
    
    std::getline(std::cin, label,':'); //label
    std::getline(std::cin, valueString,' '); // espaco em branco
    std::getline(std::cin, valueString); // armazena valor em uma string
//     std::cout << valueString << std::endl;

    std::getline(std::cin, label, ':'); //label
    std::getline(std::cin, valueString,' '); // espaco em branco
    std::getline(std::cin, valueStringD); // armazena valor em uma string
    DIMENSION = stoi(valueStringD); // armazena valor em um inteiro
//     std::cout << DIMENSION << std::endl;

    std::getline(std::cin, label,':'); //label
    std::getline(std::cin, valueString,' '); // espaco em branco
    std::getline(std::cin, EDGE_WEIGHT_TYPE); // armazena valor em uma string
//     std::cout << EDGE_WEIGHT_TYPE << std::endl;

    std::getline(std::cin, valueString); // ultima linha que contem somente uma string
  
    int vertex;
    float px, py;
    
    Graph graph(DIMENSION, NAME, TYPE, EDGE_WEIGHT_TYPE); // cria o grafo
    
    int contador = 0;
    while(std::getline(std::cin, line) && contador < DIMENSION) { // lê e insere o ponto de localização de cada vértice
        std::sscanf(line.c_str(),"%d %f %f", &vertex, &px, &py);
//         std::cout << vertex << "," << px << "," << py << std::endl;
        graph.addVertex(px, py);
        contador++;
    }
    
    int v1 = 0;
    for(auto p1 : graph.getVertex()) { // calcula a distância entre cada vértice
        int v2 = 0;
        for(auto p2 : graph.getVertex()) {
            if (v1 <= v2) {
                float p1x = p1.first;
                float p1y = p1.second;
                float p2x = p2.first;
                float p2y = p2.second;

                int distance = 0;
                if(!(p1x == p2x && p1y == p2y)) { // se são pontos diferentes, calcula a distância
                    float weight = 0;
                    if(EDGE_WEIGHT_TYPE.compare("EUC_2D") == 0) 
                        weight = round(sqrt(pow(p1x - p2x, 2) + pow(p1y - p2y, 2)));
                    else if(EDGE_WEIGHT_TYPE.compare("ATT") == 0) {
                        float rij = sqrt((pow(p1x - p2x, 2) + pow(p1y - p2y, 2)) / 10.0);
                        float tij = round(rij);
                        weight = tij < rij ? (tij + 1) : tij;
                    }
                    graph.addEdge(v1, v2, weight);
                    graph.addEdge(v2, v1, weight);
//                     std::cout << "        " << p1x << "," << p1y << " " << p2x << "," << p2y << " -> " << weight << std::endl;
                }
            }
            v2++;
        }
        v1++;
    }

    return graph;
}

Solution heuristicNearestNeighbor(Graph graph, bool showCost) {
    float cost = 0;
    list<int> path;

    int idxFirst = 0; // começa pelo primeiro vértice lido
    path.push_back(idxFirst);
    int idxNext = idxFirst;
    for(int i = 0; i < graph.getV() -1; i++) {
        pair<int,float> min = make_pair(-1, -1);
        for(auto adj : graph.getAdj(idxNext)) {
//             std::cout << adj.first << "," << adj.second << std::endl;
            // (se for o primeiro ou se o adj tem peso menor que o mínimo) e se o adj não está no caminho
            if ((min.first == -1 || adj.second < min.second) && adj.first != idxFirst && !(find(path.begin(), path.end(), adj.first) != path.end()))
                min = make_pair(adj.first, adj.second);
        }
        path.push_back(min.first);
        idxNext = min.first;
        cost += min.second;
    }
    
    for(auto adj : graph.getAdj(idxNext)) {
        if(adj.first == idxFirst) {
            path.push_back(adj.first);
            cost += adj.second;
            break;
        }
    }
    
//     for(auto e : path)
//         std::cout << e << ",";
//     std::cout << " " << std::endl;
//     std::cout << path.size() << std::endl;

    if (showCost)
        std::cout << cost << ",";
    
    Solution solution(path, cost); // cria a solução
    return solution;    // retorna a solução
}

Solution neighboor2OPTSWAP(Graph graph, Solution initialSolution) {
    int i, idxAnt;
    float cost1 = 0, cost2 = 0;
    list<int> path = initialSolution.getPath();
    list<int> path1, path2;
    
    // Gera dois indices
    int v1 = rand() % graph.getV()/2;               // Gera um numero 'v1' < metade
    int v2 = rand() % (graph.getV() - v1) + v1;     // Gera um numero 'v2' > 'v1' e 'v2' < total de vértices

    i = 0;
    path.pop_back();    // Remove o último elemento do caminho
    for(auto e : path) {
        if (i < v1 || i > v2)
            path1.push_back(e);   // Copia o que tem antes de v1 e/ou depois de v2 no novo caminho do vizinho 1
        else
            path2.push_back(e);   // Copia de v1 a v2 no novo caminho do vizinho 2
        i++;
    }
    i = 0;
    for(auto e : path) {
        if (i >= v1 && i <= v2)
            path1.push_back(e);   // Copia de v1 a v2 no novo caminho do vizinho 1
        else
            path2.push_back(e);   // Copia o que tem antes de v1 e/ou depois de v2 no novo caminho do vizinho 2
        i++;
    }
    path.push_back(*path.begin());  // Insere novamente o primeiro
    path1.push_back(*path1.begin());  // Insere novamente o primeiro
    path2.push_back(*path2.begin());  // Insere novamente o primeiro
    
//     for(auto e : path) { std::cout << e << ","; } std::cout << "---------------" << std::endl;
//     for(auto e : path1) { std::cout << e << ","; } std::cout << "---------------" << std::endl;
//     for(auto e : path2) { std::cout << e << ","; } std::cout << "---------------" << std::endl;
    
    // Computa o custo dos novos vizinhos
    i = 0;
    for(auto e : path1) {
        if (i > 0) {
            for(auto adj : graph.getAdj(idxAnt)) {
                if(adj.first == e) { 
                    cost1 += adj.second;
                    break;
                }
            }
        }
        idxAnt = e;
        i++;
    }
    i = 0;
    for(auto e : path2) {
        if (i > 0) {
            for(auto adj : graph.getAdj(idxAnt)) {
                if(adj.first == e) { 
                    cost2 += adj.second;
                    break;
                }
            }
        }
        idxAnt = e;
        i++;
    }
    
//     std::cout << initialSolution.getCost() << "," << cost1 << "," << cost2 << std::endl;
    
    // Retorna o melhor dos vizinhos gerados
    Solution neighboor1(path1, cost1);
    Solution neighboor2(path2, cost2);
    
    if (cost1 < cost2)
        return neighboor1;
    return neighboor2;
}

void VND(Graph graph, int lmax) {
    Solution bestSolution = heuristicNearestNeighbor(graph, false);             // Gera a solução inicial
    
//     for(auto e : bestSolution.getPath()) { std::cout << e << ","; } std::cout << " ---------- Cost: " << bestSolution.getCost() << std::endl;
        
    int l = 1;
    while (l < lmax) {
        Solution bestNeighboor = neighboor2OPTSWAP(graph, bestSolution);        // Gera dois vizinhos permutando e pega o melhor vizinho
        if (bestNeighboor.getCost() < bestSolution.getCost()) {                 // Se o melhor dos vizinhos tem custo menor do que a melhor solução
            bestSolution = bestNeighboor;                                       // Atualiza a melhor solução
            l = 1;
        }
        else
            l++;
    }
    
//     for(auto e : bestSolution.getPath()) { std::cout << e << ","; } std::cout << " ---------- Cost: " << bestSolution.getCost() << std::endl;
    std::cout << bestSolution.getCost() << ",";
}

bool has_suffix(const std::string &str, const std::string &suffix) {
    return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

int main() {
    std::string opcao = "";
    do {
        std::cout << "______________________________________________________________________" << std::endl;
        std::cout << "|                     HEURISTICAS E METAHEURISTICAS                   |" << std::endl;
        std::cout << "|_____________________________________________________________________|" << std::endl;
        std::cout << "|                                 TSP                                 |" << std::endl;
        std::cout << "|_____________________________________________________________________|" << std::endl;
        std::cout << "|                             DANIEL REIS                             |" << std::endl;
        std::cout << "|_____________________________________________________________________|" << std::endl;
        std::cout << "|                                 MENU                                |" << std::endl;
        std::cout << "|_____________________________________________________________________|" << std::endl;
        std::cout << "| DIGITE O NUMERO DA OPERAÇÃO QUE DESEJA                              |" << std::endl;
        std::cout << "| 1 - VND - EXECUTAR UMA INSTANCIA                                    |" << std::endl;
        std::cout << "| 2 - VND - EXECUTAR INSTANCIAS DE UM DIRETORIO                       |" << std::endl;
        std::cout << "| 3 - SAIR                                                            |" << std::endl;
        std::cout << "|_____________________________________________________________________|" << std::endl;
        std::cout << "Digite uma opcao: " << std::endl;
        std::cin >> opcao;
        
        if (opcao.compare("1") == 0) {
            std::string filename = "";
            std::cout << "Digite o nome do arquivo: (Ex: 'heu_e_met_tsp_instances/att48.tsp')" << std::endl;
            std::cin >> filename;
            Graph graph = readGraph(filename);
            int lmax = 500;
            VND(graph, lmax);
        }
        else if (opcao.compare("2") == 0) {
            std::string path = "";
            std::cout << "Digite o caminho do diretorio: (Ex: 'heu_e_met_tsp_instances/')" << std::endl;
            std::cin >> path;
            
            list<string> files;
            
            DIR *dir;
            struct dirent *lsdir;
            char cstr[path.size() + 1];
            path.copy(cstr, path.size() + 1);
            cstr[path.size()] = '\0';
            dir = opendir(cstr);
            while ((lsdir = readdir(dir)) != NULL) {
                if (has_suffix(lsdir->d_name, ".tsp"))
                    files.push_back(lsdir->d_name);
            }
            closedir(dir);
            
            files.sort();
            
            for (auto f : files) {
                std::cout << f << ",";
                clock_t tInicio = clock();
                Graph graph = readGraph(path + f);
                int lmax = 500;
                VND(graph, lmax);
                clock_t tFim = clock();
                float totalTime = (tFim - tInicio) / (CLOCKS_PER_SEC / 1000);
                totalTime /= 1000;
                std::cout << totalTime << std::endl;
            }
        }
        else
            opcao = "";
    } while (opcao == "");
    return 0;
}
