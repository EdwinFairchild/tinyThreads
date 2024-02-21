#include "tinyThreads_timers.h"

// Timer ready list
tinyThread_ready_timers_list_t ready_timers_list;

static bool tinythread_DoesTimerExist(tinyThread_timer_t *timer)
{
    tinyThread_timer_node_t *current = ready_timers_list.head;
    while (current != NULL)
    {
        if (current->timer == timer)
        {
            return true;
        }
        current = current->next;
    }
    return false;
}

void tt_TimerAdd(tinyThread_timer_t *timer)
{
    if (!tinythread_DoesTimerExist(timer))
    {
        tinyThread_timer_node_t *new_node =
            (tinyThread_timer_node_t *)tt_MemoryAllocBuf(sizeof(tinyThread_timer_node_t));
        if (new_node == NULL)
        {
            return;
        }
        new_node->timer = timer;
        new_node->next = NULL;
        new_node->prev = NULL;

        if (ready_timers_list.head == NULL)
        {
            ready_timers_list.head = new_node;
            ready_timers_list.tail = new_node;
        }
        else
        {
            ready_timers_list.tail->next = new_node;
            new_node->prev = ready_timers_list.tail;
            ready_timers_list.tail = new_node;
        }
    }
}

void tt_TimerRemove(tinyThread_timer_t *timer)
{
    tinyThread_timer_node_t *current = ready_timers_list.head;
    tinyThread_timer_node_t *nodeToRemove = NULL;
    while (current != NULL)
    {
        if (current->timer == timer)
        {
            // handle head, the only one with no previous
            if (current->prev == NULL)
            {
                ready_timers_list.head = current->next;
            }
            // handle tail, the only one with no next
            else if (current->next == NULL)
            {
                ready_timers_list.tail = current->prev;
                ready_timers_list.tail->next = NULL;
            }
            // handle middle
            else
            {
                current->next->prev = current->prev;
                current->prev->next = current->next;
            }
            nodeToRemove = current;
            break;
            ;
        }
        current = current->next;
    }
    if (nodeToRemove != NULL)
    {
        tt_MemoryFreeBuf(nodeToRemove);
    }
}

void tt_TimerUpdate()
{
    tinyThread_timer_node_t *current = ready_timers_list.head;
    while (current != NULL)
    {
        if (current->timer->active)
        {
            current->timer->countDown--;
            if (current->timer->countDown == 0)
            {
                current->timer->callback();
                if (current->timer->timerMode == TIMER_TYPE_SINGLE_SHOT)
                {
                    current->timer->active = false;
                    tt_TimerRemove(current->timer);
                }
                else
                {
                    current->timer->countDown = current->timer->period;
                }
            }
        }
        current = current->next;
    }
}

void tt_TimerInit()
{
    ready_timers_list.head = NULL;
    ready_timers_list.tail = NULL;
}

void tt_TimerStart(tinyThread_timer_t *timer)
{
    timer->active = true;
    if (timer->timerMode == TIMER_TYPE_SINGLE_SHOT)
    {
        timer->countDown = timer->period;
    }
    tt_TimerAdd(timer);
}