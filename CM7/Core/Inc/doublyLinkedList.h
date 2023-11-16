#ifndef __DOUBLYLINKEDLIST_H
#define __DOUBLYLINKEDLIST_H

/* header file contents go here */


#include <stdlib.h>
#include <stdio.h>


struct Node {
    double data;
    struct Node* next;
    struct Node* prev;
};

struct doubleLinkedList {
    int size;
    int maxSize;
    int currSum;
    int mean;
    struct Node* head;
    struct Node* tail;
};

void DBLL_init(struct doubleLinkedList* list, int n);
void pop_front(struct doubleLinkedList* list);
void push_back(struct doubleLinkedList* list, double data);
void traverse(struct doubleLinkedList* list);

#endif /* __DOUBLYLINKEDLIST_H */