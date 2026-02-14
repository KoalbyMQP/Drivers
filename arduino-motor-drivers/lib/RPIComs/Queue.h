#ifndef QUEUE_H
#define QUEUE_H

#include <Arduino.h>
#include <string.h>

template <size_t MAX_PACKETS, size_t MAX_LEN>
class PacketQueue {
public:
  PacketQueue() : head(0), tail(0), count(0) {}

  // returns false if queue is full
  bool enqueue(const char* s) {
    if (count >= MAX_PACKETS) return false;

    // copy into slot
    strncpy(buf[tail], s, MAX_LEN);
    buf[tail][MAX_LEN - 1] = '\0';

    tail = (tail + 1) % MAX_PACKETS;
    count++;
    return true;
  }

  // returns false if queue is empty
  bool dequeue(char* out, size_t outLen) {
    if (count == 0) return false;

    strncpy(out, buf[head], outLen);
    out[outLen - 1] = '\0';

    head = (head + 1) % MAX_PACKETS;
    count--;
    return true;
  }

  bool isEmpty() const { return count == 0; }
  bool isFull()  const { return count == MAX_PACKETS; }
  uint16_t size() const { return count; }

private:
  char buf[MAX_PACKETS][MAX_LEN];
  uint16_t head, tail, count;
};

#endif