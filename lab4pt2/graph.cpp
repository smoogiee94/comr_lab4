#include <StandardCplusplus.h>
#include <list>

using namespace std;

//------------------------------
// Adjacency graph created
//------------------------------
// by using doubly linked lists
class Graph{
  list<int> *adj;

  public:
  Graph(){adj = new list<int>[16];}
  void addEdge(int v, int w){adj[v].push_back(w);}
  list<int> BFS(int s, int g){
    int *prev = new int[16];
    bool *checked = new bool[16];
    for (int i = 0; i < 16; ++i){
      checked[i] = false;
      prev[i] = -1;
    }
    list<int> listOfNeighbors;
    checked[s] = true;
    listOfNeighbors.push_back(s);
    list<int>::iterator i;
    list<int> shortestPath;

    while(!listOfNeighbors.empty()){
      s = listOfNeighbors.front();
      listOfNeighbors.pop_front();

      for (i = adj[s].begin(); i != adj[s].end(); ++i){
        if(!checked[*i]){
          checked[*i] = true;
          listOfNeighbors.push_back(*i);
          prev[*i] = s;
        }
      }
    }

    int q = g;
    shortestPath.push_back(q);
    while(prev[q] != -1){
      shortestPath.push_back(prev[q]);
      q = prev[q];
    }
    return shortestPath;
  }
};

