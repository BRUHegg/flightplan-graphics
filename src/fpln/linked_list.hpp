/*
	This project is licensed under
	Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License (CC BY-NC-SA 4.0).

	A SUMMARY OF THIS LICENSE CAN BE FOUND HERE: https://creativecommons.org/licenses/by-nc-sa/4.0/

	Author: discord/bruh4096#4512

	This file contains structs used for the linked list implementation. Pointers to 
    all of the available nodes of this linked list are stored on a stack. The nodes 
    themselves belong to a continuois block of memory.
*/


#pragma once

#include <cstdint>
#include <vector>
#include <mutex>
#include <chrono>
#include <assert.h>


namespace struct_util
{
    template <class T>
    struct list_node_t
    {
        list_node_t *prev, *next;
        T data;

        list_node_t() = default;
    };

    template <class T>
    struct linked_list_t
    {
        list_node_t<T> head, tail;
        std::size_t size;
        std::chrono::time_point<std::chrono::steady_clock> start;
        double id;


        linked_list_t();

        list_node_t<T> get_node(list_node_t<T>* ptr);

        void push_front(list_node_t<T> *node);

        void push_back(list_node_t<T> *node);

        /*
            Function:
            insert_before
            @param *node: node before which to insert
            @param *node_insert: node to be inserted
        */

        void insert_before(list_node_t<T> *node, list_node_t<T> *node_insert);

        void pop(list_node_t<T> *node, std::vector<list_node_t<T>*>& release_stack);

        void release_all(std::vector<list_node_t<T>*>& release_stack);

        void update_id();
    };

    template <class T>
    struct ll_node_stack_t
    {
        list_node_t<T> *nodes;
        std::vector<list_node_t<T>*> ptr_stack;
        std::size_t m_size;


        ll_node_stack_t(std::size_t sz);

        list_node_t<T>* get_new();

        void destroy();
    };

    template <class T>
    void copy_list(ll_node_stack_t<T>& s_src, ll_node_stack_t<T>& s_dst,
            linked_list_t<T>& l_src, linked_list_t<T>& l_dst)
    {
        assert(s_src.m_size == s_dst.m_size);
        for(std::size_t i = 0; i < s_src.m_size; i++)
        {
            s_dst.nodes[i] = s_src.nodes[i];

            if(s_dst.nodes[i].prev == &l_src.head)
            {
                s_dst.nodes[i].prev = &l_dst.head;
                l_dst.head.next = &s_dst.nodes[i];
            }
            else
            {
                s_dst.nodes[i].prev = s_dst.nodes[i].prev - s_src.nodes + s_dst.nodes;
            }

            if(s_dst.nodes[i].next == &l_src.tail)
            {
                s_dst.nodes[i].next = &l_dst.tail;
                l_dst.tail.prev = &s_dst.nodes[i];
            }
            else
            {
                s_dst.nodes[i].next = s_dst.nodes[i].next - s_src.nodes + s_dst.nodes;
            }
        }

        while (!s_dst.ptr_stack.empty())
        {
            s_dst.ptr_stack.pop_back();
        }
        
        for(std::size_t i = 0; i < s_src.ptr_stack.size(); i++)
        {
            s_dst.ptr_stack.push_back(s_src.ptr_stack[i] - s_src.nodes + s_dst.nodes);
        }
    }


    // linked_list_t definitions:

    template <class T>
    linked_list_t<T>::linked_list_t()
    {
        head.next = &tail;
        head.prev = nullptr;

        tail.prev = &head;
        tail.next = nullptr;

        size = 2;
        id = 0;
        start = std::chrono::steady_clock::now();
    }

    template <class T>
    void linked_list_t<T>::push_front(list_node_t<T> *node)
    {
        node->next = head.next;
        node->prev = &head;
        head.next = node;
        size++;

        update_id();
    }

    template <class T>
    void linked_list_t<T>::push_back(list_node_t<T> *node)
    {
        node->prev = tail.prev;
        node->next = &tail;
        tail.prev = node;
        size++;

        update_id();
    }

    template <class T>
    void linked_list_t<T>::insert_before(list_node_t<T> *node, list_node_t<T> *node_insert)
    {
        node_insert->prev = node->prev;
        node_insert->next = node;
        node->prev->next = node_insert;
        node->prev = node_insert;
        size++;

        update_id();
    }

    template <class T>
    void linked_list_t<T>::pop(list_node_t<T> *node, 
        std::vector<list_node_t<T>*>& release_stack)
    {
        if(node->prev != nullptr && node->next != nullptr)
        {
            node->prev->next = node->next;
            node->next->prev = node->prev;
            release_stack.push_back(node);
            size--;
        }

        update_id();
    }

    template <class T>
    void linked_list_t<T>::release_all(std::vector<list_node_t<T>*>& release_stack)
    {
        list_node_t<T>* curr = head.next;

        while(curr != &tail)
        {
            release_stack.push_back(curr);
            curr = curr->next;
        }

        size = 2;

        update_id();
    }

    template <class T>
    void linked_list_t<T>::update_id()
    {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> dur = now - start;
        id = dur.count();
    }

    // ll_node_stack_t definitions:

    template <class T>
    ll_node_stack_t<T>::ll_node_stack_t(std::size_t sz)
    {
        m_size = sz;
        nodes = new list_node_t<T>[sz];
        ptr_stack.reserve(sz);
        for(std::size_t i = 0; i < sz; i++)
        {
            ptr_stack.push_back(nodes + i);
        }
    }

    template <class T>
    list_node_t<T>* ll_node_stack_t<T>::get_new()
    {
        if(ptr_stack.size())
        {   
            list_node_t<T>* out = ptr_stack.back();
            ptr_stack.pop_back();
            return out;
        }
        return nullptr;
    }

    template <class T>
    void ll_node_stack_t<T>::destroy()
    {
        while(!ptr_stack.empty())
            ptr_stack.pop_back();
        delete[] nodes;
    }
} // namespace struct_util
