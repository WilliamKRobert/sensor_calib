#ifndef HASH_H
#define HASH_H

#include <iostream>

class HashEntry {

private:
      unsigned long key;
      int value;

public:
      HashEntry(unsigned long key, int value) {
		this->key = key;
		this->value = value;

      }

      unsigned long getKey() {
        return key;
      }

      int getValue() {
        return value;

      }
};

const int TABLE_SIZE = 3079;


 

class HashMap {

private:
      HashEntry **table;
      int count; 
public:
      HashMap() {
            table = new HashEntry*[TABLE_SIZE];
            for (int i = 0; i < TABLE_SIZE; i++)
                  table[i] = NULL;
            count = 0;
      }

 

      int get(unsigned long key) {
            unsigned long hash = (key % TABLE_SIZE);

            while (table[hash] != NULL && table[hash]->getKey() != key)
                  hash = (hash + 1) % TABLE_SIZE;

            if (table[hash] == NULL)
                return -1;
            else
                return table[hash]->getValue();

      }

 

      void put(unsigned long key, int value) {
            unsigned long hash = (key % TABLE_SIZE);
            while (table[hash] != NULL && table[hash]->getKey() != key)
                  hash = (hash + 1) % TABLE_SIZE;
            if (table[hash] != NULL)
                  delete table[hash];
            table[hash] = new HashEntry(key, value);
            count++;
      }     

 
      int size(){
          return count;
      }
      ~HashMap() {
            for (int i = 0; i < TABLE_SIZE; i++)
                  if (table[i] != NULL)
                    delete table[i];
            delete[] table;
      }



};


#endif