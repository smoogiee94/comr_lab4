#include "Arduino.h"
#define QSIZE 16
class Queue{
  private: 
    int queueArr[QSIZE];
    int first;
    int last;
    int fuckingSize = 0;

  public:
    Queue(){
      first = -1;
      last = -1;
    }

    bool isEmpty(){
      return (first == -1 && last == -1);
    }

    bool isFull(){
      return (last + 1)%QSIZE == first ? true : false;
    }

    void enqueue(int x){
      if (isFull())
        return;

      if (isEmpty()){
        first = 0;
        last = 0;
      }

      else{
        last = (last + 1)%QSIZE;
      }
      queueArr[last] = x;
      ++fuckingSize;
      return;
    }

    void dequeue(){
      if (isEmpty()){
        return;
      }
      else if(first == 0 && last == 0){
        first = -1;
        last = -1;
      }
      else{
        first = (first + 1)%QSIZE;
      }
      --fuckingSize;
    }

    int front(){
      if (first == -1){
        return -1;
      }
      else
        return queueArr[first];
    }

    int getSize(){
      return fuckingSize;
    }

    int getItem(int x){
      return queueArr[x];
    }

};
