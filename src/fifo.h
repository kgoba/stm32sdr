#pragma once

#include <stdlib.h>
#include <stdint.h>

template<typename T, int size>
class FIFO
{
    T buffer[size];
    size_t head, count;
 
public:
    FIFO() {
      head = count = 0;
    }

    bool put(const T& data) {
      if (count == size) return false;
      size_t tail = head + count;
      if (tail >= size) tail -= size;
      buffer[tail] = data;
      count++;
      return true;
    }

    bool get(T& data) {
      if (count == 0) return false;
      data = buffer[head];
      head++;
      if (head >= size) head -= size;
      count--;
      return true;
    }

    size_t available() {
      return count;
    }

    size_t free() {
      return size - count;
    }
    
    bool empty() {
      return count == 0;
    }

    bool full() {
      return count == size;
    }
};
