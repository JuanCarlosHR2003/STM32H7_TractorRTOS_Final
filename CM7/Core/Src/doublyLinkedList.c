#include "doublyLinkedList.h"

void DBLL_init(struct doubleLinkedList* list, int n) {
    list->size = 0;
    list->maxSize = n;
    list->currSum = 0;
    list->mean = 0;
    list->head = NULL;
    list->tail = NULL;
}

void pop_front(struct doubleLinkedList* list) {
    if (list->size == 0) 
        return;

    if (list->head != NULL) {
        struct Node* node = list->head;
        list->head = list->head->next;
        if (list->head != NULL) {
            list->head->prev = NULL;
        }
        if (list->tail == node) {
            list->tail = NULL;
        }
        list->currSum -= node->data;
        free(node);
        list->size--;
    }
}

void push_back(struct doubleLinkedList* list, double data) {
    if (list->size == list->maxSize) {
        pop_front(list);
    }

    struct Node* node = (struct Node*) malloc(sizeof(struct Node));
    node->data = data;
    node->next = NULL;
    node->prev = NULL;

    if (list->head == NULL && list->tail == NULL) {
        list->head = node;
        list->tail = node;
    } 
    else if ( list->head == list->tail ) {
        list->head->next = node;
        node->prev = list->head;
        list->tail = node;
    }
    else {
        node->prev = list->tail;
        list->tail->next = node;
        list->tail = node;
    }
    
    list->size++;
    list->currSum += data;
    list->mean = list->currSum / (double)list->size;
}


void traverse(struct doubleLinkedList* list) {
    for (int i = 0; i < list->maxSize - list->size; i++) {
        printf("0,");
    }
    struct Node* node = list->head;
    while (node != NULL) {
        printf("%f,", node->data);
        node = node->next;
    }
    printf("%f\n", list->mean);
}
