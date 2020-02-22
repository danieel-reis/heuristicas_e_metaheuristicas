# Heurísticas e Metaheurísticas aplicadas TSP

* Parte 1 (Heurística Construtiva): Heurística construtiva do vizinho mais próximo. Inicia sempre pelo vértice 0 e move-se para o vizinho com menor distância que ainda não foi visitado.

* Parte 2 (VND): Utilizando a solução inicial do vizinho mais próximo, realiza uma busca por melhores soluções usando a busca 2-opt swap. Lmax = 500

* Parte 3 (Tabu Search): Gera uma solução inicial através da heurística do vizinho mais próximo. Posteriormente utiliza 2-OPT marcando os vértices no tabu. Executa 1 iteração mantendo em 10000 soluções na memória.

* Parte 4 (GRASP): A solução inicial é gerada pela heurística do vizinho mais próximo gulosa. Depois disso, o ótimo local é encontrado usando a vizinhança 2OPT. Tamanho RCL=1000 executada por 10 iterações, selecionando 25% das vezes um candidato aleatório da LCR e 75% guloso.

* Parte 5 (ILS): Parte da solução inicial utilizando a heurística do vizinho mais próximo. Executa um VND com lmax = 500 por 50 vezes, sendo que em cada execução realiza 5 permutações consecutivas utilizando a 2OPT-SWAP.

### Compilação

Segue um exemplo de compilação válido para qualquer arquivo deste projeto:

```
g++ nomedoarquivo.cpp -o nomedoarquivo
```

Logo, fica:

```
g++ tp1_daniel_reis.cpp -o tp1
g++ tp2_daniel_reis.cpp -o tp2
g++ tp3_daniel_reis.cpp -o tp3
g++ tp4_daniel_reis.cpp -o tp4
g++ tp5_daniel_reis.cpp -o tp5
```

## Execução e testes

Segue um exemplo de execução válido para qualquer arquivo deste projeto:

```
./nomedoarquivo
```

Logo, fica:

```
./tp1
./tp2
./tp3
./tp4
./tp5
```

Basta setar a instância de entrada que o software irá computar o menor caminho que passa por todos os pontos seguindo os passos da heurística e/ou metaheurística escolhida.