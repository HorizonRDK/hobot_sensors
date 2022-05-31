#ifndef __USER_LIST_H__
#define __USER_LIST_H__


#include <unistd.h>
#include <fcntl.h>
#include <sched.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <stddef.h>


#ifdef __cplusplus
extern "C"{
#endif


struct list_head 
{
	struct list_head *next, *prev;
};


#define LIST_HEAD_INIT(name) { &(name), &(name) }

#define LIST_HEAD(name) \
    (struct list_head name = LIST_HEAD_INIT(name))
    
#define INIT_LIST_HEAD(ptr) do { (ptr)->next = (ptr); (ptr)->prev = (ptr); } while (0)

#define container_of(ptr, type, member) ({			\
	const decltype( ((type *)0)->member ) *__mptr = (ptr);	\
	(type *)( (char *)__mptr - offsetof(type,member) );})

#define list_entry(ptr, type, member) \
    container_of(ptr, type, member)


static inline void
__list_add(struct list_head *entry,
                struct list_head *prev, struct list_head *next)
{
    next->prev = entry;
    entry->next = next;
    entry->prev = prev;
    prev->next = entry;
}

static inline void 
list_add_tail(struct list_head *new_node, struct list_head *head)
{
	__list_add(new_node, head->prev, head);
}

static inline void
__list_del(struct list_head *prev, struct list_head *next)
{
    next->prev = prev;
    prev->next = next;
}

static inline void
list_del(struct list_head *entry)
{
    __list_del(entry->prev, entry->next);
}

static inline bool
list_empty(struct list_head *head)
{
    return head->next == head;
}


#ifdef __cplusplus
}
#endif


#endif
