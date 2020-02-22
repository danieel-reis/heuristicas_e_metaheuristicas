/**
 * Aluno: Daniel Martins Reis
 * Como compilar: g++ tp1_daniel_reis.cpp -o tp1
 * Como executar: ./tp1
 */

#include <math.h>
#include <dirent.h>
#include <iostream>
#include <list>
#include <algorithm> // função find
#include <fstream>   // leitura de arquivos
#include <tuple>
#include <time.h>

using namespace std;

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

void heuristicNearestNeighbor(Graph graph) {
    float sizePath = 0;
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
        sizePath += min.second;
    }
    
    for(auto adj : graph.getAdj(idxNext)) {
        if(adj.first == idxFirst) {
            path.push_back(adj.first);
            sizePath += adj.second;
            break;
        }
    }
    
//     for(auto e : path)
//         std::cout << e << ",";
//     std::cout << " " << std::endl;
//     std::cout << path.size() << std::endl;
    std::cout << sizePath << ",";
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
        std::cout << "| 1 - HEURISTICA CONSTRUTIVA - EXECUTAR UMA INSTANCIA                 |" << std::endl;
        std::cout << "| 2 - HEURISTICA CONSTRUTIVA - EXECUTAR INSTANCIAS DE UM DIRETORIO    |" << std::endl;
        std::cout << "| 3 - SAIR                                                            |" << std::endl;
        std::cout << "|_____________________________________________________________________|" << std::endl;
        std::cout << "Digite uma opcao: " << std::endl;
        std::cin >> opcao;
        
        if (opcao.compare("1") == 0) {
//             std::string filename = "heu_e_met_tsp_instances/att48.tsp";
            std::string filename = "";
            std::cout << "Digite o nome do arquivo: (Ex: 'heu_e_met_tsp_instances/att48.tsp')" << std::endl;
            std::cin >> filename;
            Graph graph = readGraph(filename);
            heuristicNearestNeighbor(graph);
        }
        else if (opcao.compare("2") == 0) {
//             std::string path = "heu_e_met_tsp_instances/";
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
                heuristicNearestNeighbor(graph);
                clock_t tFim = clock();
                float totalTime = (tFim - tInicio) / (CLOCKS_PER_SEC / 1000);
                totalTime /= 1000;
                std::cout << totalTime << std::endl;
            }
        }
        else if (opcao.compare("3"))
            opcao = "";
    } while (opcao == "");
    return 0;
}
